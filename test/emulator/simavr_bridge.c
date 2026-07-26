/* simavr-based klipper firmware bridge
 *
 * Loads the klipper firmware ELF, runs it under simavr (a cycle-accurate
 * AVR simulator), and exposes the firmware's UART0 as a pty so klippy
 * can connect to it.
 *
 * The point of running the actual firmware (instead of re-implementing
 * its protocol in Python) is fidelity: every endstop sample loop, every
 * trsync timer, every software-PWM pulse runs the same C code that
 * ships on real hardware. Tests that exercise complex stateful
 * subsystems (bltouch, multi-sample probe, load_cell, ...) get correct
 * behavior for free instead of needing per-feature emulator hacks.
 *
 * Usage:
 *   simavr_bridge --elf out/klipper.elf --slave-link /tmp/klipper.tty \
 *       [--mcu atmega2560] [--duration <seconds>] [-v] \
 *       [--control-socket /tmp/klipper.ctl]
 *
 * The slave-link file receives the path of the pty's slave end so the
 * parent process can configure klippy to connect there.
 *
 * The control socket (when provided) accepts newline-terminated text
 * commands from a single connected client and applies them to
 * simavr's peripheral models. Each command is one of:
 *
 *   adc <channel> <millivolts>
 *       Set ADC channel <0-15> to a constant millivolt level. Klipper
 *       reads this through the AVR's ADC peripheral and oversamples
 *       to a 13-bit sum.
 *   gpio <port> <pin> <0|1>
 *       Drive PORT<port> pin <0-7> high or low. Useful for endstop
 *       pin transitions ("simulated probe touch").
 *   spi <hexbytes>
 *       Replace the global SPI response queue with a sequence of
 *       bytes (e.g. "0000080000ffff..."). Whenever the firmware
 *       writes a byte to SPDR the bridge raises SPI_IRQ_INPUT with
 *       the next byte from this queue, wrapping around at the end.
 *       Tests with multiple SPI slaves on the bus generally want
 *       their queue length to match the total per-poll byte count
 *       so the round-robin lines up with the firmware's per-device
 *       command sequence.
 *   i2c <hexbytes>
 *       Replace the global I2C response queue. Each i2c_read by
 *       the firmware pops one byte from this queue (round-robin).
 *       The bridge auto-ACKs all addressing and writes; the queue
 *       only matters for read responses. Tests scripting multiple
 *       I2C slaves (e.g. SHT3X status query then measurement read)
 *       concatenate their bytes in firmware-read order.
 *   step_trigger <step_port> <step_pin> <count> <trig_port> <trig_pin> <val>
 *       Count rising edges on <step_port><step_pin>; once <count>
 *       have been seen, drive <trig_port><trig_pin> to <val>. One-
 *       shot: re-issue the command between samples to re-arm. Used
 *       by the multi-sample probe tests to simulate "physical touch
 *       after N steps into the bed" without reimplementing klipper's
 *       endstop sample loop in Python.
 *   bltouch <ctrl_port> <ctrl_pin> <sensor_port> <sensor_pin> <invert>
 *       Configure the BLTouch state machine. Hooks the control pin
 *       to decode klippy's PWM commands (pin_up/down, touch_mode,
 *       reset, self_test) by their high-pulse durations and drives
 *       the sensor pin to match. Self-pulses the sensor briefly
 *       (~150 ms) on pin_down/touch_mode so klippy's verify_state
 *       sees "triggered" then settles back to "not triggered" for
 *       the subsequent probe move. invert=1 mirrors klippy's `^!`
 *       flag on the sensor pin in [bltouch].
 *   spi_tmc_chip <cs_port> <cs_pin> <tmc2130|tmc5160|tmc2240|tmc2660>
 *       Register a TMC SPI chip on the shared bus. The bridge hooks
 *       the chip's CS pin to track which chip is selected; the active
 *       chip's per-chip register file backs the 5-byte register-access
 *       protocol so two chips on the same bus see independent writes
 *       (`SET_TMC_FIELD` on chip A no longer poisons chip B's reads).
 *       For `tmc2660`, MOSI bytes also switch to the 3-byte (20-bit)
 *       datagram decoder while CS is asserted; the other variants stay
 *       on the 5-byte path. Without a registration, 5-byte transfers
 *       share a single fallback register file - the legacy behaviour
 *       for tests that only init one chip on the bus. Up to
 *       TMC_CHIP_MAX chips supported.
 *   spi_ads1220_chip <cs_port> <cs_pin> <drdy_port> <drdy_pin> <rate_hz>
 *       Register an ADS1220 chip's CS + DRDY pins so the bridge can
 *       pulse DRDY active-low at the chip's configured sample rate
 *       via a simavr cycle timer; on each 3-byte ADC read in
 *       continuous mode the bridge de-asserts DRDY (drive HIGH) so
 *       the firmware reads at exactly the chip's rate instead of
 *       its full poll rate. Replaces the previous "hold DRDY low
 *       forever" gpio fixture approach which overflowed the
 *       firmware's wake-task drain under stepper load.
 *
 * Lines that don't parse are silently ignored to avoid breaking the
 * simulator on a fixture typo.
 *
 * Time-base policy
 * ----------------
 * Every state machine in this file (BLTouch pulse decoder, probe_step
 * ramp, step_trigger counter, sw_i2c bit-bang, the SPI/TWI queues, and
 * the cycle-timer-driven BLTouch self-pulse end) timestamps events in
 * MCU sim time -- `avr->cycle` -- never host wall clock. The firmware
 * itself reads `timer_read_time()` off simavr's emulated TCNT, which
 * is also driven by `avr->cycle`, so any windowed/timed comparison the
 * bridge does against firmware-observed events is in the same time
 * base regardless of how fast simavr is running on the host.
 *
 * The only `clock_gettime(CLOCK_MONOTONIC)` calls are the bridge's
 * main-loop wall-clock throttle (caps simavr at host wall-clock; in
 * sim_time mode it acts as a CEILING only) and the `--duration`
 * deadline safety net. Both are intentionally host-time. Control
 * socket select() timeouts are host-time too; they're polling, not
 * state-machine timing.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <pty.h>

#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/sim_io.h>
#include <simavr/sim_cycle_timers.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_adc.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_spi.h>
#include <simavr/avr_twi.h>
#include <simavr/parts/uart_pty.h>

static volatile int g_running = 1;
static avr_t *g_crash_avr = NULL;

/* Set by the control thread when the fixture-setup `barrier` completes
 * (the test runner's last interaction before it launches klippy). The
 * main loop sees this and restarts its wall-clock --duration baseline so
 * the duration window covers klippy's work time only, not the startup
 * gap. See the start_ts handling in main() for the full rationale. */
static volatile int g_restart_deadline = 0;

/* Deterministic setup (TICK_PROTOCOL_DESIGN.md 4). In tick mode the bridge
 * does NOT free-run on wall-clock before klippy connects; it advances the AVR
 * ONLY toward a pending fixture-setup `barrier` (the control thread sets this)
 * and pauses otherwise, so avr->cycle - and hence every fixture cycle-timer's
 * phase (ADS1220 DRDY, sw_uart, ...) - at tick-connect is identical across
 * runs, independent of host scheduling. 0 = no barrier pending. */
static volatile uint64_t g_barrier_target = 0;

static void
on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

static void
on_crash(int sig)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "BRIDGE CRASH sig=%d cyc=%llu\n", sig,
                     g_crash_avr ? (unsigned long long)g_crash_avr->cycle : 0);
    if (write(2, buf, n) < 0) { /* nothing else to do */ }
    _exit(139);
}

/* ----------------------------- Control plane -----------------------
 *
 * The control socket is a single-client unix-domain stream socket that
 * the test harness opens to push fixture state into simavr's
 * peripheral models. We run the listener in a dedicated thread so the
 * main thread can keep the simulator advancing - the only shared
 * state is the avr_t, and avr_raise_irq is documented as thread-safe
 * (it just appends to an IRQ event queue).
 */

/* Global SPI response state. The firmware's SPI byte-out IRQ fires
 * our hook; we raise the input IRQ with the next byte from this
 * queue. Mutated from the control thread (when the test pushes new
 * data) and read from the simulator main thread - access is guarded
 * by spi_mutex. avr_raise_irq is itself thread-safe; the lock is
 * just for the queue contents. */
struct spi_response_state {
    pthread_mutex_t lock;
    uint8_t bytes[256];
    size_t len;
    size_t pos;
    /* TMC SPI mode: when enabled, the bridge interprets each 5-byte
     * SPI datagram as a TMC2130/5160/2240/2660 register access:
     *   - byte 0: register address (high bit = write/read flag)
     *   - bytes 1..4: 4-byte register value (big-endian)
     * On a write the bytes are stored in tmc_regs[addr]; on a read
     * the next datagram's MISO bytes 1..4 echo back the stored value
     * for that register (TMC's "shift register" semantics, where the
     * response carries the PREVIOUSLY-addressed register's data, mean
     * klippy's read-after-write verify pattern works out as long as
     * we serve the value tied to the address that was just written
     * or read in the prior datagram). MISO byte 0 is the SPI status
     * which we always return as 0 = no error. */
    int tmc_mode;
    uint8_t tmc_in_buf[5];     /* MOSI bytes accumulated this datagram */
    uint8_t tmc_in_pos;        /* 0..4 */
    uint8_t tmc_out_buf[5];    /* MISO bytes the firmware will see */
    uint8_t tmc_out_pos;       /* 0..4 */
    uint32_t tmc_regs[256];    /* default 5-byte TMC register file used
                                * when no spi_tmc_chip registration is
                                * active for the chip currently selected
                                * on the bus. Tests that register every
                                * chip see per-chip register state via
                                * the tmc_chip[].regs tables; tests that
                                * register none (or whose CS pin the
                                * fixture never declared) fall through
                                * to this shared table. */
    /* ADS1220 mode: byte 0 is a command. Top nibble selects:
     *   0x4n WREG: write bytes to register n>>2, count = (n & 3) + 1
     *   0x2n RREG: read bytes from register n>>2, count = (n & 3) + 1
     *   0x0n other: NOOP / commands that don't care about MISO
     * Register data is stored in ads_regs[reg][byte_idx] (4 bytes
     * per register max - the chip has 4 8-bit config registers).
     * On RREG, the bridge serves register bytes back over the
     * subsequent NOOP transfers; the first response byte after the
     * cmd is don't-care from klippy's POV (params['response'][1:]). */
    int ads_mode;
    uint8_t ads_remaining;     /* bytes still to send/recv this cmd */
    uint8_t ads_reg;           /* register index for active RREG/WREG */
    uint8_t ads_byte_idx;      /* offset into the register payload */
    uint8_t ads_is_read;       /* 1 = RREG (serving), 0 = WREG (storing) */
    uint8_t ads_streaming;     /* 1 = currently serving 3-byte ADC sample */
    uint8_t ads_sample[3];     /* current 3-byte ADC reading being served */
    uint8_t ads_regs[16][4];   /* register file: 4 regs x 4 bytes (the
                                * chip only has 4 8-bit config regs;
                                * shared across all ADS1220 chips on
                                * the bus, klippy probes them in turn) */
};

static struct spi_response_state spi_state = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .len = 0,
    .pos = 0,
};

/* Per-chip state for TMC SPI bus members. Two protocols share the
 * bus and are selected at registration time:
 *
 *   TMC_PROTO_5BYTE - tmc2130 / tmc5160 / tmc2240: 5-byte/40-bit
 *     datagrams (address byte with read/write flag + 4 data bytes).
 *     The chip's register file lives in `regs[]` here; the 5-byte
 *     accumulator (tmc_in_buf / tmc_out_buf) stays in spi_state
 *     because only one chip can be CS-active at a time and the byte
 *     counter must stay continuous across the transaction. Writes
 *     during this transaction land in the active chip's `regs[]`;
 *     reads return that chip's value. A test that does NOT register
 *     a chip falls back to the legacy shared `spi_state.tmc_regs`
 *     table (the simpler one-chip-per-bus case).
 *
 *   TMC_PROTO_TMC2660 - 3-byte/20-bit datagrams: upper 4 bits dummy,
 *     bits 19..17 = 3-bit register id (0/4/5/6/7), bits 16..0 = data.
 *     MISO is the chip's READRSP@RDSEL<n> register packed the same
 *     way (response<<4 in 24 bits) so klippy's
 *     MCU_TMC2660_SPI.get_register_raw decodes it via
 *       data = (pr[0] << 16) | (pr[1] << 8) | pr[2]
 *     which leaves the 20-bit response shifted up by 4 (matching the
 *     field offsets in tmc2660.py: stallguard at bit 4, mstep at
 *     14..23). MOSI/MISO bytes accumulate in this struct's in_buf /
 *     out_buf; the 5-byte accumulator is bypassed entirely while a
 *     tmc2660 CS is low (so its byte counter stays aligned for the
 *     other 5-byte chips on the same bus).
 *
 * On every CS falling edge we set tmc_active_chip = this slot so the
 * spi_out_hook routes the transaction's bytes through this chip's
 * state. CS rising clears it. The CS hook also performs the
 * proto-specific finalize: 5-byte does nothing on rising edge
 * (spi_out_hook decodes at byte-4); tmc2660 decodes the 3
 * accumulated MOSI bytes and stashes any DRVCONF.RDSEL change for
 * the next response. */
#define TMC_CHIP_MAX 8
enum tmc_proto {
    TMC_PROTO_5BYTE = 0,
    TMC_PROTO_TMC2660 = 1,
};
struct tmc_chip {
    char cs_port;             /* 'A'..'L' */
    int cs_pin;               /* 0..7 */
    int cs_active;            /* 1 while CS is low */
    enum tmc_proto proto;     /* TMC_PROTO_5BYTE or TMC_PROTO_TMC2660 */
    /* 5-byte protocol: per-chip register file. */
    uint32_t regs[256];
    /* tmc2660 (3-byte) per-transaction buffers; unused for 5-byte. */
    uint8_t in_buf[3];        /* MOSI bytes accumulated this datagram */
    uint8_t in_pos;           /* 0..3 */
    uint8_t out_buf[3];       /* MISO bytes the firmware will see */
    uint8_t out_pos;          /* 0..3 */
    uint8_t rdsel;            /* most recent DRVCONF.RDSEL value (0..2) */
};
static struct tmc_chip tmc_chips[TMC_CHIP_MAX];
static int tmc_chips_count = 0;
static int tmc_active_chip = -1;  /* index into tmc_chips, or -1 */

/* ADS1220 DRDY pulsing. With DRDY held perpetually low (the previous
 * fixture-gpio approach), the firmware's ads1220_event polls and
 * reads at its full poll rate - on heavy stepper load this overflows
 * the firmware's wake-task drain. Each registered chip gets its CS
 * pin tracked (so we know which chip is being addressed during a
 * transaction) and its DRDY pin pulsed by a simavr cycle timer at
 * the chip's configured sample rate; on each 3-byte ADC read in
 * continuous mode the bridge de-asserts DRDY (drive HIGH) so the
 * firmware backs off until the next periodic assertion. The firmware
 * then sees the chip running at exactly its configured rate and the
 * test cfg can use the chip's real default (660 SPS) instead of the
 * 175 SPS workaround. All scheduling is in `avr->cycle` so timing is
 * deterministic regardless of host load. */
#define ADS1220_CHIP_MAX 4
struct ads1220_chip {
    char cs_port;             /* 'A'..'L' */
    int cs_pin;               /* 0..7 */
    char drdy_port;           /* 'A'..'L' */
    int drdy_pin;             /* 0..7 */
    uint32_t period_cycles;   /* MCU cycles between DRDY assertions */
    int drdy_low;             /* 1 = currently asserted (low) */
    struct avr_t *avr;        /* set on first chip register */
    /* Per-chip ADS1220 config register file (4 regs x 4 bytes). Each chip
     * must read back its OWN registers: a load_cell config can carry two
     * ADS1220 chips (e.g. [load_cell] + [load_cell_probe] on separate CS
     * lines), and klippy resets + reads each chip's regs to validate the
     * post-reset all-zero state. A single bus-wide file lets one chip's
     * WREG corrupt the other's readback ("Invalid ads1220 reset state") -
     * the same shared-state hazard the per-chip TMC SPI decoder fixed.
     * Selected by ads1220_active_chip in spi_out_hook. */
    uint8_t regs[16][4];
};
static struct ads1220_chip ads1220_chips[ADS1220_CHIP_MAX];
static int ads1220_chips_count = 0;

static int ads1220_active_chip = -1; /* index of chip whose CS is low */

/* ADS131M0x (init-only model). klippy's ads131m0x.py runs an ID read,
 * RESET + ack readback, and four register write-verifies at connect
 * for every [load_cell] section with an ads131m02/m04 sensor. The
 * test suite uses those sections for parse+init coverage only (no
 * client ever subscribes), so the bridge models just the init
 * protocol: 3-byte words, >=4-word frames, a 16-bit command in word0,
 * and the response to a frame served in word0 of the NEXT frame (the
 * chip is full-duplex one-frame-delayed). No DRDY pacing and no
 * sample streaming - the DRDY pin is never asserted, which is fine
 * because the firmware only watches it after a query the test never
 * issues. Registering the chip's CS pin also keeps its frames out of
 * the ADS1220 byte decoder, which shares the SPI bus. */
#define ADS131_CHIP_MAX 4
#define ADS131_REG_MAX 16
struct ads131_chip {
    char cs_port;              /* 'A'..'L' */
    int cs_pin;                /* 0..7 */
    uint8_t id_hi;             /* ID register high byte (0x22 = M02) */
    uint16_t regs[ADS131_REG_MAX];
    uint16_t next_resp;        /* word0 of the next frame's MISO */
    uint8_t frame_pos;         /* byte index within current frame */
    uint8_t cmd_hi, cmd_lo;    /* word0 = command */
    uint8_t d0_hi, d0_lo;      /* word1 = first data word (WREG) */
};
static struct ads131_chip ads131_chips[ADS131_CHIP_MAX];
static int ads131_chips_count = 0;
static int ads131_active_chip = -1;  /* index of chip whose CS is low */

/* Reset-default register file: ID (klippy verifies the high byte),
 * STATUS with the WORD24 bit set (klippy's post-init check masks with
 * 0xBFFC and expects 0x0100), MODE reset default. */
static void
ads131_reset_regs(struct ads131_chip *c)
{
    memset(c->regs, 0, sizeof(c->regs));
    c->regs[0x00] = (uint16_t)(((uint16_t)c->id_hi << 8) | 0x02);
    c->regs[0x01] = 0x0100;
    c->regs[0x02] = 0x0100;
}

/* ADXL345 accelerometer (full streaming model). klippy's adxl345.py
 * verifies DEVID (0xe5), write-verifies BW_RATE / POWER_CTL /
 * DATA_FORMAT / FIFO_CTL, then the firmware's sensor_adxl345.c reads
 * one 9-byte burst per wake: [0x32|READ|MULTI] followed by DATAX0..
 * DATAZ1 (six bytes), FIFO_CTL (address auto-increment reaches 0x38;
 * firmware validates it still reads SET_FIFO_CTL=0x90), and
 * FIFO_STATUS (0x39, entries remaining). Samples accrue in a modeled
 * 32-deep FIFO at the rate klippy programmed into BW_RATE, paced by
 * avr->cycle so tick-mode runs are deterministic. Sample values are a
 * synthetic vibration: x = amp_raw * sin(2*pi * vib_freq_hz * t),
 * y = 0, z = base_z_raw (~1 g), with t taken from the SAMPLE INDEX
 * (served / rate) rather than the read cycle, so the waveform is
 * exact regardless of when the firmware drains the FIFO. That gives
 * ACCELEROMETER_MEASURE real 13-bit data and TEST_RESONANCES a
 * clean spectral line to find, while staying independent of host
 * scheduling. */
#define ADXL_CHIP_MAX 2
#define ADXL_REG_MAX 0x40
struct adxl_chip {
    char cs_port;              /* 'A'..'L' */
    int cs_pin;                /* 0..7 */
    struct avr_t *avr;
    uint8_t regs[ADXL_REG_MAX];
    /* transaction state (valid while CS low) */
    uint8_t addr;              /* current register (auto-inc if MULTI) */
    int multi, is_read;
    int pos;                   /* byte index in transaction */
    /* streaming state */
    int powered;               /* POWER_CTL measure bit */
    uint64_t start_cycle;      /* cycle of the 0->1 measure transition */
    uint64_t served;           /* samples consumed since start */
    uint8_t sample[6];         /* latched burst data (x0x1 y0y1 z0z1) */
    uint8_t fifo_after;        /* FIFO entries left after this burst */
    /* synthesis knobs (spi_adxl345_chip command) */
    int vib_freq_hz;
    int amp_raw;
    int base_z_raw;
};
static struct adxl_chip adxl_chips[ADXL_CHIP_MAX];
static int adxl_chips_count = 0;
static int adxl_active_chip = -1;    /* index of chip whose CS is low */

static void
adxl_reset_regs(struct adxl_chip *c)
{
    memset(c->regs, 0, sizeof(c->regs));
    c->regs[0x00] = 0xe5;             /* DEVID */
}

/* Data rate from the BW_RATE code klippy wrote (adxl345.py
 * QUERY_RATES: 0x8=25 ... 0xf=3200). */
static int
adxl_rate_hz(const struct adxl_chip *c)
{
    int code = c->regs[0x2C] & 0x0f;
    if (code < 0x8)
        code = 0x8;
    if (code > 0xf)
        code = 0xf;
    return 3200 >> (0xf - code);
}

/* Latch one burst worth of data: pick the next FIFO sample (indexed
 * time base), encode 13-bit sign-extended little-endian pairs, and
 * compute the post-read FIFO count. */
static void
adxl_latch_burst(struct adxl_chip *c)
{
    int rate = adxl_rate_hz(c);
    uint64_t now = c->avr ? c->avr->cycle : 0;
    uint64_t produced = 0;
    if (c->powered && c->avr && now > c->start_cycle)
        produced = (now - c->start_cycle) * (uint64_t)rate
            / c->avr->frequency;
    if (produced > c->served + 32) {
        /* FIFO overflow: drop the oldest (real chip keeps newest 32;
         * exact indices matter less than keeping time monotonic). */
        c->served = produced - 32;
    }
    uint64_t avail = produced > c->served ? produced - c->served : 0;
    int16_t vx = 0, vy = 0, vz = (int16_t)c->base_z_raw;
    if (avail > 0) {
        double t = (double)c->served / (double)rate;
        vx = (int16_t)(c->amp_raw
                       * sin(2.0 * M_PI * (double)c->vib_freq_hz * t));
        c->served++;
        avail--;
    }
    /* 13-bit two's complement, low byte first; high nibble must be
     * 0x0 or 0xf (sign extension) for the firmware's glitch check. */
    c->sample[0] = (uint8_t)(vx & 0xff);
    c->sample[1] = (uint8_t)((vx >> 8) & 0xff);
    c->sample[2] = (uint8_t)(vy & 0xff);
    c->sample[3] = (uint8_t)((vy >> 8) & 0xff);
    c->sample[4] = (uint8_t)(vz & 0xff);
    c->sample[5] = (uint8_t)((vz >> 8) & 0xff);
    c->fifo_after = (uint8_t)(avail > 31 ? 31 : avail);
}

/* Global I2C (TWI) read response queue. Klippy talks to I2C
 * peripherals (SHT3X, LDC1612, MAX31865, ...) by writing a command
 * byte sequence then reading N bytes per measurement; tests script
 * the expected bytes via fixture so klippy's drivers see plausible
 * data and don't shutdown. The bridge serves these reads by hooking
 * the firmware's TWI peripheral output IRQ - on each read by the
 * firmware, raise the input IRQ with the next byte from this queue,
 * wrapping around when exhausted. */
struct twi_response_state {
    pthread_mutex_t lock;
    uint8_t bytes[1024];
    size_t len;
    size_t pos;
    uint8_t current_addr;  /* last addressed slave for diagnostics */
    int verbose;
    /* Register-aware mode: when reg_count > 0, the bridge tracks
     * the last byte the firmware wrote (klippy's "register address"
     * preface), then on the subsequent read serves the bytes
     * configured for that register. Set up via the i2c_reg control
     * command; falls back to the flat bytes[] queue when no reg
     * configured for the current address. */
    uint8_t reg_addrs[64];          /* register addresses (one per chip) */
    uint8_t reg_data[64][8];        /* up to 8 bytes per register */
    uint8_t reg_lens[64];           /* bytes configured per register */
    int reg_count;                  /* slots in use */
    uint8_t pending_reg;            /* most recent write (register select) */
    int pending_reg_valid;          /* 1 if pending_reg holds a valid select */
    uint8_t pending_pos;            /* read offset into the matched register */
};

static struct twi_response_state twi_state = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .len = 0,
    .pos = 0,
    .current_addr = 0,
    .verbose = 0,
};

/* BLTouch state machine.
 *
 * Klippy controls a real BLTouch via PWM duty cycles encoding commands
 * (pin_up / pin_down / touch_mode / reset / self_test). Each command's
 * "high" pulse duration is unique:
 *   pin_down    0.000650 s
 *   touch_mode  0.001165 s
 *   pin_up      0.001475 s
 *   self_test   0.001780 s
 *   reset       0.002190 s
 *
 * We hook the firmware's GPIO output on the configured control pin,
 * record each rising edge's MCU cycle, and on the matching falling
 * edge compute the high-pulse duration and decode the command.
 *
 * Sensor-pin behaviour mirrors a real BLTouch:
 *   - RAISED (default after pin_up/reset): sensor "not triggered"
 *     (raw 0, or 1 if invert=true).
 *   - DEPLOYED / ARMED: sensor self-pulses to "triggered" briefly,
 *     then settles back to "not triggered" while waiting for an
 *     actual touch. Klippy's verify_state runs within 100 ms of the
 *     command, so a ~150 ms pulse is enough to satisfy the verify
 *     while leaving plenty of headroom for the subsequent probe
 *     move to start with the pin in its "not triggered" state. The
 *     actual "touch" trigger comes from the step_trigger hook driving
 *     the sensor pin to triggered after N stepper edges.
 *
 * Keeping the pulse short avoids "probe triggered prior to movement"
 * errors klippy's check_no_movement raises if the sensor pin is
 * triggered at home_start time. */
#define BLT_RAISED   0
#define BLT_DEPLOYED 1
#define BLT_ARMED    2

/* 50 ms self-pulse for DEPLOYED/ARMED. Klippy's verify_state has a
 * 100 ms timeout window after each PWM command, so 50 ms is well
 * within - the firmware samples the pin every few ms during verify.
 * Keeping it short matters because klippy issues home_start for
 * the actual probe move shortly after touch_mode + verify_state;
 * if the pulse outlasts that gap, the probe home triggers
 * immediately at 0 displacement (the firmware's sample loop sees
 * the still-asserted pin and fires the trsync). */
#define BLT_PULSE_USEC 50000

struct bltouch_state {
    int configured;
    char ctrl_port;           /* 'A'..'L' */
    int ctrl_pin;             /* 0..7 */
    char sensor_port;
    int sensor_pin;
    int invert;               /* 1 if klippy's `^!` flag set */
    uint64_t rise_cycle;      /* MCU cycle of last ctrl rising edge */
    int state;                /* BLT_* */
    int sensor_pulsing;       /* 1 if a self-pulse timer is queued */
};

static pthread_mutex_t bltouch_lock = PTHREAD_MUTEX_INITIALIZER;
static struct bltouch_state bltouch = {0};
static avr_t *g_avr_for_bltouch = NULL;

/* Forward decl - definition lives further down with the other hooks. */
static void bltouch_ctrl_hook(struct avr_irq_t *irq, uint32_t value,
                              void *param);

/* Drive the BLTouch sensor pin. raw=1 means the firmware sees
 * "triggered" before klippy's `_invert` is applied (i.e. matches
 * klippy's pin_value=1 when its config has plain `^pinX` and not
 * `^!pinX`). The bridge driver applies our `invert` flag the same
 * way klippy does: when invert is true, "triggered" maps to raw 0.
 * Called with bltouch_lock held. */
static void
bltouch_drive_sensor(int triggered)
{
    if (!g_avr_for_bltouch || !bltouch.configured)
        return;
    int raw = bltouch.invert ? !triggered : triggered;
    avr_irq_t *sensor = avr_io_getirq(
        g_avr_for_bltouch,
        AVR_IOCTL_IOPORT_GETIRQ(bltouch.sensor_port),
        bltouch.sensor_pin);
    if (sensor)
        avr_raise_irq(sensor, raw);
}

/* Cycle timer that fires when the BLTouch's brief self-pulse should
 * end. Returns 0 (one-shot) so simavr clears the timer slot. */
static avr_cycle_count_t
bltouch_pulse_end(struct avr_t *avr, avr_cycle_count_t when, void *param)
{
    (void)avr;
    (void)when;
    (void)param;
    pthread_mutex_lock(&bltouch_lock);
    bltouch.sensor_pulsing = 0;
    /* Pin returns to "not triggered" while the probe waits for an
     * actual touch (which the step_trigger hook will inject by
     * raising the sensor IRQ when N stepper pulses are seen). */
    bltouch_drive_sensor(0);
    pthread_mutex_unlock(&bltouch_lock);
    return 0;
}

/* Step-triggered GPIO drive: count rising edges on a "step" pin and,
 * when the count crosses a configured threshold, drive a "trigger"
 * pin to a configured level. Used for tests with an external probe
 * (bltouch, contact-sensor, ...) where physical touch happens after
 * the steppers have run a known distance into the bed. The python
 * emulator's auto_trigger_after_steps does this counting on top of
 * its protocol re-impl; here we do it on simavr's actual GPIO IRQs
 * so the firmware's endstop sample loop sees a real pin transition.
 *
 * Multi-sample probes (screws_tilt_adjust, bed_mesh) re-arm by
 * inverting their probe-up command between samples; we reset
 * count_seen and trigger_armed on a falling edge of the trigger
 * pin (the firmware lifts it back high after each retract) so the
 * next sample starts from zero. */
struct step_trigger {
    int armed;                /* 1: count edges; 0: ignore */
    char step_port;           /* 'A'..'L' */
    int step_pin;             /* 0..7 */
    uint32_t count_seen;
    uint32_t count_threshold;
    char trigger_port;
    int trigger_pin;
    int trigger_value;        /* 0 or 1 */
};

/* Up to STEP_TRIG_MAX concurrent step-counter entries: one per
 * stepper that the test wants tracked. Sequential G28 X/Y/Z homes
 * each trigger their own stepper, so independent entries (one per
 * axis) let all homes succeed without per-home reconfiguration.
 * The hook fires for ALL configured pins; the lookup tries each
 * entry to find the one matching the port:pin that just changed. */
#define STEP_TRIG_MAX 16
static pthread_mutex_t step_lock = PTHREAD_MUTEX_INITIALIZER;
static struct step_trigger step_trigs[STEP_TRIG_MAX];
static int step_trigs_count = 0;
static avr_t *g_avr_for_step = NULL;

/* Cache of (port,pin) hook registrations to avoid double-registering.
 * Indexed by port_idx*8 + pin. 12 ports * 8 pins = 96 slots. */
static int g_step_hook_registered[96] = {0};
/* Same cache for the trigger-pin notify hooks (for auto-rearm on
 * transitions back to NOT-triggered between multi-sample probes). */
static int g_trigger_hook_registered[96] = {0};

/* Forward decl - definition lives further down with the SPI hook. */
static void step_pin_hook(struct avr_irq_t *irq, uint32_t value, void *param);
static void trigger_pin_hook(struct avr_irq_t *irq, uint32_t value,
                             void *param);

/* Software I2C bit-bang slave-side ACK emulation. self_driving
 * suppresses recursive SDA/SCL hook fires while we're inside our own
 * sw_i2c_drive_sda call - simavr's avr_raise_irq updates irq->value
 * AFTER running notify hooks, so any update_irqs we trigger
 * synchronously (e.g. via the patched SET_EXTERNAL) re-raises the
 * unchanged-but-currently-being-dispatched value, retriggering our
 * own hooks. Without the flag this counts an extra SCL falling
 * inside the 8th-edge handler and prematurely fires release before
 * the firmware reads the ACK slot. */
struct sw_i2c_state {
    int configured;
    char scl_port; int scl_pin;
    char sda_port; int sda_pin;
    int active;
    int bit_count;
    int last_sda;
    int last_scl;
    int self_driving;
    avr_t *avr;
};

static pthread_mutex_t sw_i2c_lock = PTHREAD_MUTEX_INITIALIZER;
static struct sw_i2c_state sw_i2c = {0};
static int g_sw_i2c_hook_registered[96] = {0};

static void sw_i2c_scl_hook(struct avr_irq_t *irq, uint32_t value, void *param);
static void sw_i2c_sda_hook(struct avr_irq_t *irq, uint32_t value, void *param);

/* Software-UART slave model for TMC2208/2209 single-wire UART.
 *
 * klippy bit-bangs the UART line via src/tmcuart.c: tmcuart_send_event
 * toggles the tx_pin at cfg_bit_time MCU cycles per bit, sending a
 * pre-encoded bit stream where each chip-level byte is wrapped as
 * start(0) + 8 data bits LSB-first + stop(1) (klippy's
 * MCU_TMC_uart_bitbang._add_serial_bits). For a read request the
 * firmware then reconfigures the pin as input (single-wire) and
 * samples for the slave's response; for a write it just goes idle.
 *
 * The bridge:
 *   1. Hooks the UART pin's port IRQ. On a falling edge from idle
 *      (line was high) we have a start bit - schedule a simavr cycle
 *      timer at +1.5 * bit_time so the first sample lands in the
 *      middle of data bit 0.
 *   2. The cycle timer reads the pin, stores the bit, schedules the
 *      next sample at +bit_time. After 8 data bits and 1 stop bit
 *      we have a complete byte; back to idle, wait for next falling
 *      edge.
 *   3. Once we've seen 4 bytes (read req) or 8 bytes (write), decode
 *      the TMC datagram (sync 0x05, node addr, reg addr, [4 data],
 *      crc8). The CRC validates the frame.
 *   4. WRITE: store data[0..3] in the chip's register file.
 *   5. READ: synthesize a response (8 bytes) with the requested
 *      register's stored value, then encode each byte as 10 bits
 *      (start + data LSB-first + stop) and drive the pin via
 *      SET_EXTERNAL on a cycle timer that fires at bit_time intervals
 *      to pulse the response back to the firmware. firmware's
 *      tmcuart_read_sync_event sees the response and decodes it.
 *
 * All timing is in MCU sim cycles (avr->cycle) so the protocol
 * timing is deterministic regardless of host CPU load. */
#define SW_UART_MAX 4
#define SW_UART_RX_BUF 16    /* encoded bytes; we expect <=8 */
#define SW_UART_TX_BUF 16    /* response bytes (8 data + slack) */
struct sw_uart_state {
    int active;
    char port; int pin;          /* shared TX/RX in single-wire mode */
    avr_t *avr;
    uint32_t bit_time;           /* MCU cycles per bit */
    uint8_t addr;                /* expected node address */

    /* RX state machine. Driven by the pin IRQ for the start-bit edge,
     * then by a simavr cycle timer (rx_timer) for sampling subsequent
     * bits. */
    int rx_armed;                /* 1 when expecting next falling edge */
    int rx_in_byte;              /* 1 when sampling bits of current byte */
    int rx_bit_pos;              /* next data-bit index 0..7 */
    uint8_t rx_byte;             /* accumulating byte */
    uint8_t rx_buf[SW_UART_RX_BUF];
    int rx_len;
    uint64_t rx_last_cycle;      /* cycle of last byte (for idle gap) */

    /* TX state machine. Once an RX read request decodes, we populate
     * tx_buf with the response and a cycle timer drives bits at
     * bit_time intervals via SET_EXTERNAL. */
    int tx_active;
    uint8_t tx_buf[SW_UART_TX_BUF];
    int tx_len;
    int tx_byte_pos;             /* current byte being sent */
    int tx_bit_pos;              /* 0..9: 0=start, 1..8=data, 9=stop */

    /* Per-chip register file for TMC2208/2209. 7-bit address space. */
    uint32_t regs[128];
};

static pthread_mutex_t sw_uart_lock = PTHREAD_MUTEX_INITIALIZER;
static struct sw_uart_state sw_uart[SW_UART_MAX];
static int sw_uart_count = 0;
static int g_sw_uart_hook_registered[96] = {0};

static void sw_uart_pin_hook(struct avr_irq_t *irq, uint32_t value,
                             void *param);
static avr_cycle_count_t sw_uart_rx_sample(struct avr_t *avr,
                                           avr_cycle_count_t when,
                                           void *param);
static avr_cycle_count_t sw_uart_tx_bit(struct avr_t *avr,
                                        avr_cycle_count_t when,
                                        void *param);

/* Forward decl - definition lives further down with the SPI hook. */
static void tmc_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param);
static void ads1220_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param);
static void ads131_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param);
static avr_cycle_count_t ads1220_drdy_assert(struct avr_t *avr,
                                             avr_cycle_count_t when,
                                             void *param);

/* Load-cell-probe analog-trigger emulation. The load_cell_probe driver
 * arms a trigger_analog on the MCU and waits for an ADC sample to
 * cross a force threshold. Real hardware closes that loop by the
 * probe physically pushing into the bed - force grows over many ms
 * as the load cell flexes. The bridge synthesizes that by holding
 * data_ready_pin low (firmware reads continuously) and counting Z
 * step rising edges: each ADC read returns
 *   sample = (step_count - step_count_at_burst_start) * force_per_step
 * for a linear ramp that builds while Z is descending. When no step
 * has fired for `reset_cycles` (a quiet stretch i.e. a tare phase),
 * the next edge starts a fresh burst with sample=0, so each probe
 * point begins from zero. All timing is in MCU sim cycles
 * (`avr->cycle`) so behaviour is deterministic regardless of host
 * load - the firmware's SOS filter sees the same sequence of samples
 * in the same MCU time base every run. */
struct probe_step_state {
    pthread_mutex_t lock;
    int active;
    int step_port_ord;        /* (int)(unsigned char)'L' etc. */
    int step_pin;
    avr_t *avr;                            /* for cycle access in hooks */
    uint64_t last_step_cycle;              /* avr->cycle at last edge */
    uint64_t reset_cycles;                 /* quiet -> burst reset */
    uint32_t step_count;                   /* total rising edges */
    uint32_t step_count_at_burst_start;    /* baseline for current ramp */
    int32_t force_per_step;                /* raw 24-bit counts per step */
};

static struct probe_step_state probe_step = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
};
static int g_probe_step_hook_registered[96] = {0};

static void probe_step_hook(struct avr_irq_t *irq, uint32_t value,
                            void *param);

/* eddy-current probe ramp (LDC1612 I2C virtual endstop).
 *
 * The probe_eddy_current virtual endstop arms a trigger_analog on the
 * raw LDC1612 frequency count and waits for it to cross a threshold as
 * the coil nears the bed. Real hardware closes that loop physically:
 * descending Z raises the measured oscillator frequency (and thus the
 * 28-bit count) past the threshold the driver set from the calibrated
 * descend_z. A static fixture sample can't trigger - the count never
 * moves. The bridge synthesizes that for the LDC1612 DATA0 register: it
 * tracks the Z stepper position (descending step edges +1, ascending -1,
 * read via the DIR pin) and serves a 28-bit count that climbs as Z
 * descends and crosses the driver's gt threshold, firing the endstop
 * for G28 homing, bed-mesh probing, and PROBE. net_descent is the
 * absolute (never reset) stepper position, so the count is one
 * continuous function of Z - quiet bed-mesh scan / rapid_scan reads at
 * a settled height map to a valid in-range frequency. The count is
 * floored at the calibration's lowest frequency so a read at very high
 * Z stays in range rather than going out-of-range / zero (error). All
 * timing is in MCU sim cycles, so it is deterministic regardless of
 * host load.
 *
 * `PROBE METHOD=tap` adds two further pieces: piecewise contact and
 * uniform-period sampling.
 *
 * Piecewise contact: real hardware's tap signal is a SLOPE change. In
 * free air the coil approaches the bed and the count climbs at one
 * rate (free_per_step); once the toolhead contacts and compresses, the
 * coil distance stops shrinking and the count climbs much more slowly
 * (depress_per_step). The diff_peak detector picks up that drop in
 * derivative output. The bridge models contact at a fixed absolute
 * net_descent (`contact_descent`): below contact, slope =
 * free_per_step; above contact, slope = depress_per_step.
 *   raw(N) = baseline_raw + free_per_step * min(N, contact_descent)
 *          + depress_per_step * max(0, N - contact_descent)
 * gt-only probes (homing/scan/PROBE) still fire when raw crosses the
 * driver's gt threshold inside the free-air segment, so adding the
 * piecewise knee is a strict superset of the original ramp.
 *
 * Uniform-period sampling: the tap's diff_peak detector requires that
 * consecutive samples represent uniformly-spaced moments in time -
 * otherwise the per-sample derivative carries quantization noise above
 * the tap threshold and fires spuriously. Two sources of jitter exist
 * in the naive ramp:
 *   1. The firmware polls STATUS every period/2 (~2ms at 250 SPS) and
 *      reads DATA on the next poll that finds it set, so the sample
 *      moment slips ~2ms relative to the conceptual "conversion ready"
 *      instant. Fix: drive STATUS-ready off the sim clock at exact
 *      multiples of period_cycles and LATCH the value AT that period
 *      boundary (not at firmware read time).
 *   2. net_descent only changes at integer step edges. Step rate isn't
 *      a clean integer multiple of the sample rate, so each period
 *      window catches +/-1 step of jitter (~6 % per-sample noise at
 *      typical descent speeds). Fix: at latch time, interpolate the
 *      step count sub-step by extrapolating from the most recent two
 *      same-direction step edges (last_step_cycle - prev_step_cycle is
 *      the current step interval; the fraction since the last edge is
 *      how many sub-steps to add). The interpolation is reset when the
 *      step direction reverses so it never overshoots.
 *
 * Together these collapse the sample stream to one count value per
 * exact period_cycles boundary, with sub-step-accurate values, so the
 * derivative output is essentially noise-free in free air and the
 * diff_peak detector only fires on the real (piecewise) slope change. */
struct ldc1612_ramp_state {
    pthread_mutex_t lock;
    int active;
    int step_port_ord;
    int step_pin;
    /* Z DIR pin, read at each step edge so the count tracks signed Z
     * displacement: it climbs while descending and falls while
     * retracting (so a settled height reads a stable frequency). */
    int dir_port_ord;
    int dir_pin;
    int descend_level;                     /* DIR level meaning "descend" */
    avr_t *avr;
    uint64_t last_step_cycle;
    /* ABSOLUTE descent: net_descent is the running Z stepper position,
     * NEVER reset. The bed sits at a fixed absolute stepper position, so
     * the frequency is a single global function of net_descent. */
    int64_t net_descent;
    uint32_t baseline_raw;                 /* count at net_descent == 0 */
    uint32_t free_per_step;                /* count climb per descend step */
    /* Piecewise contact model: above contact_descent the per-step
     * climb is depress_per_step instead of free_per_step, so the
     * derivative drops and the tap diff_peak detector fires. contact
     * is in absolute net_descent units (the bed is at a fixed absolute
     * stepper position). contact_descent == 0 disables the piecewise
     * model (single linear slope, as in the original ramp). */
    int64_t contact_descent;
    uint32_t depress_per_step;             /* count climb after contact */
    /* Sub-step interpolation for noise-free per-sample derivative:
     * step intervals (~250us at 10mm/s descent) are several cycles off
     * a clean integer multiple of period_cycles, so per-period the
     * integer step delta jitters by +/-1. prev_step_cycle is the cycle
     * of the previous step that matched prev_descending. At latch time
     * we use (latch_cycle - last_step_cycle) / (last_step_cycle -
     * prev_step_cycle) as the fractional sub-step to add to
     * net_descent. Reset when the direction reverses. */
    uint64_t prev_step_cycle;
    int prev_descending;
    /* Uniform-period sampling: next_period_cycle is the sim cycle at
     * which the next conversion completes. STATUS reports
     * UNREADCONV0=1 once now>=next_period_cycle, at which point we
     * latch the count AT next_period_cycle (interpolated) and advance
     * next_period_cycle += period_cycles. has_pending_sample says a
     * latched value is awaiting consumption by DATA0_MSB; the firmware
     * polls STATUS at period/2 so it normally consumes within one poll
     * and the gating is exactly period_cycles between successive
     * DATA0_MSB reads regardless of firmware-side poll-grid jitter. */
    uint64_t period_cycles;
    uint64_t next_period_cycle;
    int has_pending_sample;
    /* Legacy field retained for the original (contact-less) STATUS
     * gating path; only consulted when contact_descent == 0. */
    uint64_t last_data_cycle;
    /* MSB/LSB are read in two separate i2c transactions; latch the
     * 32-bit value when the firmware reads DATA0_MSB so the paired LSB
     * read serves the same sample even if a step fires between them. */
    uint32_t latched;
    int latch_valid;
};

static struct ldc1612_ramp_state ldc1612_ramp = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
};
static int g_ldc1612_ramp_hook_registered[96] = {0};

static void ldc1612_ramp_hook(struct avr_irq_t *irq, uint32_t value,
                              void *param);

/* Optional per-step-edge diagnostic trace for the ldc1612_ramp hook,
 * matched on the KLIPPY_TICK_TRACE pattern. When
 * KLIPPY_LDC1612_RAMP_TRACE=<path> is set, every step-rising-edge
 * appends one line:
 *   cycle dir_irq_value descend_level descending net_descent_after
 * (decimal). Used to bisect dir-pin polarity vs. fixture-configured
 * descend_level (see PR8 Known limitations). Ship off; setting it
 * does not alter ramp counting. */
static FILE *g_ldc1612_ramp_trace = NULL;

struct control_ctx {
    avr_t *avr;
    char socket_path[PATH_MAX];
    int verbose;
};

static int
parse_int(const char *s, int *out)
{
    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (end == s || (*end && *end != '\n' && *end != ' '))
        return -1;
    *out = (int)v;
    return 0;
}

static void
apply_control_command(struct control_ctx *ctx, const char *line)
{
    char op[32] = {0};
    int a = 0, b = 0, c = 0;
    int n = sscanf(line, "%31s %d %d %d", op, &a, &b, &c);
    if (n < 1)
        return;
    if (strcmp(op, "adc") == 0 && n >= 3) {
        /* adc <channel> <millivolts>: drive ADC channel <a> to <b>
         * mV. simavr's ADC model converts the value internally using
         * the configured VREF. Channel must be 0..15. */
        if (a < 0 || a > 15)
            return;
        avr_irq_t *irq = avr_io_getirq(
            ctx->avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0 + a);
        if (irq)
            avr_raise_irq(irq, b);
        if (ctx->verbose)
            fprintf(stderr, "simavr_bridge: control adc %d <- %d mV\n",
                    a, b);
    } else if (strcmp(op, "spi") == 0) {
        /* spi <hexbytes>: replace the SPI response queue. Re-parse
         * the rest of the line as ASCII hex pairs. */
        const char *hex = strchr(line, ' ');
        if (!hex)
            return;
        hex++;
        while (*hex == ' ')
            hex++;
        pthread_mutex_lock(&spi_state.lock);
        spi_state.len = 0;
        spi_state.pos = 0;
        while (*hex && spi_state.len < sizeof(spi_state.bytes)) {
            char hi = *hex++;
            if (!hi || hi == '\n' || hi == '\r')
                break;
            char lo = *hex++;
            if (!lo || lo == '\n' || lo == '\r')
                break;
            int hv = hi >= '0' && hi <= '9' ? hi - '0'
                   : hi >= 'a' && hi <= 'f' ? hi - 'a' + 10
                   : hi >= 'A' && hi <= 'F' ? hi - 'A' + 10
                   : -1;
            int lv = lo >= '0' && lo <= '9' ? lo - '0'
                   : lo >= 'a' && lo <= 'f' ? lo - 'a' + 10
                   : lo >= 'A' && lo <= 'F' ? lo - 'A' + 10
                   : -1;
            if (hv < 0 || lv < 0)
                break;
            spi_state.bytes[spi_state.len++] = (uint8_t)((hv << 4) | lv);
        }
        size_t set_len = spi_state.len;
        pthread_mutex_unlock(&spi_state.lock);
        if (ctx->verbose)
            fprintf(stderr,
                    "simavr_bridge: control spi queue len=%zu\n",
                    set_len);
    } else if (strcmp(op, "i2c_reg") == 0) {
        /* i2c_reg <reg_hex> <byte_hex> <byte_hex> ...: register a
         * register-keyed I2C response. The bridge tracks the register
         * address the firmware writes preceding a read and serves the
         * matching bytes here. Used for chips like LDC1612 / SHT3X
         * where the register-select byte determines which fixed-size
         * value the chip returns on the subsequent read.  Up to 64
         * registers, 8 bytes per register. */
        const char *p = strchr(line, ' ');
        if (!p)
            return;
        p++;
        unsigned int reg = 0;
        if (sscanf(p, "%x", &reg) != 1)
            return;
        while (*p && *p != ' ')
            p++;
        while (*p == ' ')
            p++;
        pthread_mutex_lock(&twi_state.lock);
        if (twi_state.reg_count < 64) {
            int slot = twi_state.reg_count;
            twi_state.reg_addrs[slot] = (uint8_t)(reg & 0xff);
            int len = 0;
            while (*p && len < 8) {
                unsigned int b = 0;
                if (sscanf(p, "%x", &b) != 1)
                    break;
                twi_state.reg_data[slot][len++] = (uint8_t)(b & 0xff);
                while (*p && *p != ' ')
                    p++;
                while (*p == ' ')
                    p++;
            }
            twi_state.reg_lens[slot] = (uint8_t)len;
            twi_state.reg_count++;
        }
        pthread_mutex_unlock(&twi_state.lock);
    } else if (strcmp(op, "spi_ads1220") == 0) {
        /* spi_ads1220: switch the SPI hook into ADS1220 mode. The
         * bridge decodes WREG/RREG commands and maintains a 4x4 byte
         * register file so klippy's write-then-verify pattern in
         * setup_chip succeeds. RESET clears the file so the post-
         * reset readback returns zeros as the driver expects. */
        pthread_mutex_lock(&spi_state.lock);
        spi_state.ads_mode = 1;
        spi_state.ads_remaining = 0;
        spi_state.ads_byte_idx = 0;
        memset(spi_state.ads_regs, 0, sizeof(spi_state.ads_regs));
        pthread_mutex_unlock(&spi_state.lock);
        if (ctx->verbose)
            fprintf(stderr, "simavr_bridge: spi_ads1220 mode enabled\n");
    } else if (strcmp(op, "spi_tmc") == 0) {
        /* spi_tmc: switch the SPI hook into TMC SPI register-file mode.
         * Each 5-byte SPI datagram is decoded as a TMC2130/5160/2240
         * register access; writes update the shared register file
         * and reads return the stored value. Klippy's write-then-read
         * verify pattern matches as a result. TMC2660 chips on the
         * same bus need a per-chip spi_tmc_chip command (different
         * datagram length) before init runs. */
        pthread_mutex_lock(&spi_state.lock);
        spi_state.tmc_mode = 1;
        spi_state.tmc_in_pos = 0;
        spi_state.tmc_out_pos = 0;
        memset(spi_state.tmc_in_buf, 0, sizeof(spi_state.tmc_in_buf));
        memset(spi_state.tmc_out_buf, 0, sizeof(spi_state.tmc_out_buf));
        memset(spi_state.tmc_regs, 0, sizeof(spi_state.tmc_regs));
        /* DRV_STATUS default for tmc2130 / tmc5160 / tmc2240: stst=1
         * (standstill) and cs_actual=5 (a non-zero value any of klippy's
         * driver field tables decodes as a healthy current scaler).
         * Without this default, klippy's periodic _do_periodic_check
         * reads cs_actual=0 once motion starts and shuts down with
         * "DRV_STATUS: 00000000 cs_actual=0(Reset?)" before the test
         * gets to its DUMP_TMC / SET_TMC_FIELD commands. The register
         * file is shared across the bus; klippy initializes each chip
         * sequentially so a single shared default suffices. */
        spi_state.tmc_regs[0x6f] = 0xc0050000;
        /* MSCNT (tmc2130 / tmc5160 / tmc2240, reg 0x6a) defaults to 0
         * from the memset above - left explicit here so callers see
         * the contract. klippy's _query_phase reads this register at
         * stepper-enable time to derive its mcu_phase_offset; with a
         * stable value (zero) every read, [endstop_phase] sees a
         * consistent phase on every G28 and never trips its
         * "incorrect phase" check. The same applies to the tmc2660's
         * MSTEP (served via RDSEL=0 in the per-chip 3-byte path - the
         * pre-seeded response there is also zero for that field) and
         * to the per-chip sw_uart register file for tmc2208/tmc2209
         * (zero-initialized in the static sw_uart[] array). */
        spi_state.tmc_regs[0x6a] = 0;
        pthread_mutex_unlock(&spi_state.lock);
        if (ctx->verbose)
            fprintf(stderr, "simavr_bridge: spi_tmc mode enabled\n");
    } else if (strcmp(op, "i2c") == 0) {
        /* i2c <hexbytes>: replace the I2C read response queue. Same
         * round-robin semantics as spi. The hexbytes are returned to
         * the firmware byte-by-byte on each i2c_read transaction. */
        const char *hex = strchr(line, ' ');
        if (!hex)
            return;
        hex++;
        while (*hex == ' ')
            hex++;
        pthread_mutex_lock(&twi_state.lock);
        twi_state.len = 0;
        twi_state.pos = 0;
        while (*hex && twi_state.len < sizeof(twi_state.bytes)) {
            char hi = *hex++;
            if (!hi || hi == '\n' || hi == '\r')
                break;
            char lo = *hex++;
            if (!lo || lo == '\n' || lo == '\r')
                break;
            int hv = hi >= '0' && hi <= '9' ? hi - '0'
                   : hi >= 'a' && hi <= 'f' ? hi - 'a' + 10
                   : hi >= 'A' && hi <= 'F' ? hi - 'A' + 10
                   : -1;
            int lv = lo >= '0' && lo <= '9' ? lo - '0'
                   : lo >= 'a' && lo <= 'f' ? lo - 'a' + 10
                   : lo >= 'A' && lo <= 'F' ? lo - 'A' + 10
                   : -1;
            if (hv < 0 || lv < 0)
                break;
            twi_state.bytes[twi_state.len++] = (uint8_t)((hv << 4) | lv);
        }
        size_t set_len = twi_state.len;
        pthread_mutex_unlock(&twi_state.lock);
        if (ctx->verbose)
            fprintf(stderr,
                    "simavr_bridge: control i2c queue len=%zu\n",
                    set_len);
    } else if (strcmp(op, "gpio") == 0 && n >= 4) {
        /* gpio <port> <pin> <level>: drive PORT<port> pin<pin> high
         * (level=1) or low (level=0). port is 'A' encoded as ASCII.
         * Sets external.pull_mask / pull_value via SET_EXTERNAL so
         * the drive PERSISTS across firmware PORT/DDR writes (a
         * bare avr_raise_irq update only stays put until the next
         * update_irqs from the firmware overwrites r_pin from
         * PORT). Also raises the per-pin IRQ for an immediate
         * effect. */
        char port = (char)a;
        if (port < 'A' || port > 'L')
            return;
        if (b < 0 || b > 7)
            return;
        avr_ioport_external_t ext = {
            .name = (uint8_t)port,
            .mask = (uint8_t)(1U << b),
            .value = c ? (uint8_t)(1U << b) : 0,
        };
        avr_ioctl(ctx->avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(port), &ext);
        avr_irq_t *irq = avr_io_getirq(
            ctx->avr, AVR_IOCTL_IOPORT_GETIRQ(port), b);
        if (irq)
            avr_raise_irq(irq, c ? 1 : 0);
        if (ctx->verbose)
            fprintf(stderr, "simavr_bridge: control gpio %c%d <- %d\n",
                    port, b, c);
    }
    /* Unknown commands silently ignored - the test harness is allowed
     * to send forward-compatible commands the bridge doesn't yet
     * understand. */
}

/* step_trigger <step_port> <step_pin> <count> <trig_port> <trig_pin> <val>
 * Configure (and arm) the step-edge -> trigger-pin path. step_port and
 * trig_port arrive here as ASCII letters (rewritten by the caller into
 * their integer ord values, same as the gpio command). The hook is
 * installed lazily the first time we see a port; re-issuing the
 * command updates the targets and resets count_seen. */
static void
apply_step_trigger(struct control_ctx *ctx,
                   int step_port_ord, int step_pin,
                   uint32_t count_threshold,
                   int trig_port_ord, int trig_pin,
                   int trig_val)
{
    if (step_port_ord < 'A' || step_port_ord > 'L')
        return;
    if (trig_port_ord < 'A' || trig_port_ord > 'L')
        return;
    if (step_pin < 0 || step_pin > 7 || trig_pin < 0 || trig_pin > 7)
        return;

    pthread_mutex_lock(&step_lock);
    /* Find an existing entry for this stepper (re-issue replaces),
     * otherwise allocate a new slot. */
    int slot = -1;
    for (int i = 0; i < step_trigs_count; i++) {
        if (step_trigs[i].step_port == (char)step_port_ord
                && step_trigs[i].step_pin == step_pin) {
            slot = i;
            break;
        }
    }
    if (slot < 0 && step_trigs_count < STEP_TRIG_MAX) {
        slot = step_trigs_count++;
    }
    if (slot < 0) {
        pthread_mutex_unlock(&step_lock);
        return;  /* table full */
    }
    step_trigs[slot].armed = 1;
    step_trigs[slot].step_port = (char)step_port_ord;
    step_trigs[slot].step_pin = step_pin;
    step_trigs[slot].count_seen = 0;
    step_trigs[slot].count_threshold = count_threshold;
    step_trigs[slot].trigger_port = (char)trig_port_ord;
    step_trigs[slot].trigger_pin = trig_pin;
    step_trigs[slot].trigger_value = trig_val ? 1 : 0;
    pthread_mutex_unlock(&step_lock);

    /* Register a hook on this specific pin's IRQ. simavr's IRQ system
     * lets us add multiple notifiers per IRQ; we always register since
     * each step pin we care about needs its own hook. (Using a per-port
     * registration cache like before doesn't work for multi-stepper
     * configs sharing a port - we'd skip pins after the first.) */
    avr_irq_t *step_irq = avr_io_getirq(
        ctx->avr,
        AVR_IOCTL_IOPORT_GETIRQ((char)step_port_ord),
        step_pin);
    if (step_irq && !g_step_hook_registered[(step_port_ord - 'A') * 8
                                            + step_pin]) {
        g_avr_for_step = ctx->avr;
        avr_irq_register_notify(step_irq, step_pin_hook, NULL);
        g_step_hook_registered[(step_port_ord - 'A') * 8 + step_pin] = 1;
    }

    /* Hook the trigger pin so we can auto-rearm this entry when the
     * pin transitions back to NOT-triggered (e.g. the BLTouch state
     * machine resets the sensor between multi-sample probes). For
     * one-shot endstops nothing else drives the pin so it stays at
     * trigger_value forever and never re-arms - that's correct. */
    avr_irq_t *trig_irq = avr_io_getirq(
        ctx->avr,
        AVR_IOCTL_IOPORT_GETIRQ((char)trig_port_ord),
        trig_pin);
    if (trig_irq && !g_trigger_hook_registered[(trig_port_ord - 'A') * 8
                                                + trig_pin]) {
        g_avr_for_step = ctx->avr;
        avr_irq_register_notify(trig_irq, trigger_pin_hook, NULL);
        g_trigger_hook_registered[(trig_port_ord - 'A') * 8 + trig_pin] = 1;
    }

    if (ctx->verbose)
        fprintf(stderr,
            "simavr_bridge: step_trigger step=%c%d count=%u "
            "trig=%c%d val=%d\n",
            (char)step_port_ord, step_pin,
            (unsigned)count_threshold,
            (char)trig_port_ord, trig_pin, trig_val);
}

/* probe_step <step_port> <step_pin> <reset_us> <force_per_step>
 * Wire up the load-cell-probe analog-trigger emulation: hook a step
 * pin (typically Z) and synthesize a ramped ADC sample on each
 * continuous-mode ADS1220 read. Each rising edge after a quiet
 * stretch of `reset_us` microseconds starts a new burst with sample
 * baseline 0; subsequent reads return (steps_in_burst *
 * force_per_step) raw counts. force_per_step is in raw ADC counts
 * per step - pick it so the trigger threshold (counts_per_gram *
 * trigger_force grams) is reached after enough probe-descent steps
 * for the SOS filter to settle, but stays well below the safety
 * range (counts_per_gram * force_safety_limit). */
static void
apply_probe_step(struct control_ctx *ctx,
                 int step_port_ord, int step_pin,
                 int32_t reset_us,
                 int32_t force_per_step)
{
    if (step_port_ord < 'A' || step_port_ord > 'L')
        return;
    if (step_pin < 0 || step_pin > 7)
        return;
    uint64_t cycles_per_us = ctx->avr->frequency / 1000000ULL;
    uint64_t reset_cycles = (uint64_t)(reset_us > 0 ? reset_us : 50000)
                            * cycles_per_us;
    pthread_mutex_lock(&probe_step.lock);
    probe_step.active = 1;
    probe_step.step_port_ord = step_port_ord;
    probe_step.step_pin = step_pin;
    probe_step.avr = ctx->avr;
    probe_step.last_step_cycle = 0;
    probe_step.reset_cycles = reset_cycles;
    probe_step.step_count = 0;
    probe_step.step_count_at_burst_start = 0;
    probe_step.force_per_step = force_per_step;
    pthread_mutex_unlock(&probe_step.lock);

    avr_irq_t *step_irq = avr_io_getirq(
        ctx->avr,
        AVR_IOCTL_IOPORT_GETIRQ((char)step_port_ord),
        step_pin);
    if (step_irq && !g_probe_step_hook_registered[(step_port_ord - 'A') * 8
                                                  + step_pin]) {
        avr_irq_register_notify(step_irq, probe_step_hook, NULL);
        g_probe_step_hook_registered[(step_port_ord - 'A') * 8
                                     + step_pin] = 1;
    }
    if (ctx->verbose)
        fprintf(stderr,
            "simavr_bridge: probe_step step=%c%d reset_us=%d"
            " force_per_step=%d\n",
            (char)step_port_ord, step_pin,
            (int)reset_us, (int)force_per_step);
}

static void
probe_step_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)param;
    /* Z step rising edges drive the load-cell ramp. simavr passes
     * the new value as `value` while irq->value still holds the
     * previous value (notify hooks fire before the IRQ updates).
     * After a quiet stretch (reset_cycles with no edges) the next
     * edge restarts the burst from sample=0, so each tare->descent
     * cycle starts fresh. */
    int prev = (int)irq->value;
    int next = value ? 1 : 0;
    if (prev == next || next == 0)
        return;
    pthread_mutex_lock(&probe_step.lock);
    if (probe_step.active && probe_step.avr) {
        uint64_t now = probe_step.avr->cycle;
        uint64_t last = probe_step.last_step_cycle;
        if (last == 0 || now - last > probe_step.reset_cycles) {
            probe_step.step_count_at_burst_start = probe_step.step_count;
        }
        probe_step.step_count++;
        probe_step.last_step_cycle = now;
    }
    pthread_mutex_unlock(&probe_step.lock);
}

/* ldc1612_ramp <step_port> <step_pin> <dir_port> <dir_pin> <descend_level>
 *              <baseline_raw> <free_per_step> <sample_rate>
 *              [<contact_descent> <depress_per_step>]
 * Hook the Z stepper's step + dir pins and arm the LDC1612 ramp so the
 * eddy virtual endstop fires on descent (G28 / bed-mesh / PROBE). The
 * contact_descent / depress_per_step pair is optional (defaults to 0
 * == disabled) and turns on the piecewise contact model that makes
 * `PROBE METHOD=tap` work: the count climbs at free_per_step until
 * net_descent reaches contact_descent, then climbs at depress_per_step
 * (much smaller), giving the diff_peak detector a clean slope change
 * to fire on. */
static void
apply_ldc1612_ramp(struct control_ctx *ctx,
                   int step_port_ord, int step_pin,
                   int dir_port_ord, int dir_pin, int descend_level,
                   uint32_t baseline_raw, uint32_t free_per_step,
                   int sample_rate_hz,
                   int64_t contact_descent, uint32_t depress_per_step)
{
    if (step_port_ord < 'A' || step_port_ord > 'L')
        return;
    if (step_pin < 0 || step_pin > 7)
        return;
    if (sample_rate_hz <= 0)
        sample_rate_hz = 400;             /* LDC1612 default data_rate
                                             (upstream d2aa4bd7e) */
    pthread_mutex_lock(&ldc1612_ramp.lock);
    ldc1612_ramp.active = 1;
    ldc1612_ramp.step_port_ord = step_port_ord;
    ldc1612_ramp.step_pin = step_pin;
    ldc1612_ramp.dir_port_ord = dir_port_ord;
    ldc1612_ramp.dir_pin = dir_pin;
    ldc1612_ramp.descend_level = descend_level ? 1 : 0;
    ldc1612_ramp.avr = ctx->avr;
    ldc1612_ramp.last_step_cycle = 0;
    ldc1612_ramp.prev_step_cycle = 0;
    ldc1612_ramp.prev_descending = 0;
    ldc1612_ramp.net_descent = 0;
    ldc1612_ramp.baseline_raw = baseline_raw & 0x0fffffff;
    ldc1612_ramp.free_per_step = free_per_step;
    ldc1612_ramp.contact_descent = contact_descent;
    ldc1612_ramp.depress_per_step = depress_per_step;
    ldc1612_ramp.period_cycles =
        (uint64_t)(ctx->avr->frequency / (uint32_t)sample_rate_hz);
    ldc1612_ramp.next_period_cycle = 0;
    ldc1612_ramp.has_pending_sample = 0;
    ldc1612_ramp.last_data_cycle = 0;
    ldc1612_ramp.latched = 0;
    ldc1612_ramp.latch_valid = 0;
    pthread_mutex_unlock(&ldc1612_ramp.lock);

    avr_irq_t *step_irq = avr_io_getirq(
        ctx->avr, AVR_IOCTL_IOPORT_GETIRQ(step_port_ord), step_pin);
    if (step_irq && !g_ldc1612_ramp_hook_registered[(step_port_ord - 'A') * 8
                                                     + step_pin]) {
        avr_irq_register_notify(step_irq, ldc1612_ramp_hook, NULL);
        g_ldc1612_ramp_hook_registered[(step_port_ord - 'A') * 8
                                       + step_pin] = 1;
    }
    if (ctx->verbose)
        fprintf(stderr,
            "simavr_bridge: ldc1612_ramp step=%c%d dir=%c%d desc_lvl=%d"
            " baseline_raw=%u free=%u\n",
            (char)step_port_ord, step_pin, (char)dir_port_ord, dir_pin,
            descend_level, baseline_raw, free_per_step);
}

static void
ldc1612_ramp_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)param;
    int prev = (int)irq->value;
    int next = value ? 1 : 0;
    if (prev == next || next == 0)
        return;
    /* Read the DIR pin level now (the step edge samples direction). */
    int dir_level = ldc1612_ramp.descend_level;  /* default if no dir irq */
    if (ldc1612_ramp.avr) {
        avr_irq_t *dir_irq = avr_io_getirq(
            ldc1612_ramp.avr,
            AVR_IOCTL_IOPORT_GETIRQ(ldc1612_ramp.dir_port_ord),
            ldc1612_ramp.dir_pin);
        if (dir_irq)
            dir_level = dir_irq->value ? 1 : 0;
    }
    int descending = (dir_level == ldc1612_ramp.descend_level);
    pthread_mutex_lock(&ldc1612_ramp.lock);
    if (ldc1612_ramp.active && ldc1612_ramp.avr) {
        /* Absolute Z stepper position, never reset. */
        if (descending)
            ldc1612_ramp.net_descent++;
        else
            ldc1612_ramp.net_descent--;
        if (g_ldc1612_ramp_trace) {
            fprintf(g_ldc1612_ramp_trace,
                    "%" PRIu64 " %d %d %d %" PRId64 "\n",
                    (uint64_t)ldc1612_ramp.avr->cycle,
                    dir_level, ldc1612_ramp.descend_level,
                    descending, (int64_t)ldc1612_ramp.net_descent);
            fflush(g_ldc1612_ramp_trace);
        }
        /* Maintain the (prev_step_cycle, last_step_cycle) window only
         * while the step direction is consistent; the period of two
         * same-direction edges is the current step interval, and
         * `(now - last) / (last - prev)` is the sub-step fraction the
         * sampler interpolates. On a direction reversal we discard the
         * prior pair so interpolation never crosses the inversion (the
         * descent doesn't know how long the retract step will be until
         * it's complete). */
        if (ldc1612_ramp.last_step_cycle
            && descending == ldc1612_ramp.prev_descending) {
            ldc1612_ramp.prev_step_cycle = ldc1612_ramp.last_step_cycle;
        } else {
            ldc1612_ramp.prev_step_cycle = 0;
        }
        ldc1612_ramp.prev_descending = descending;
        ldc1612_ramp.last_step_cycle = ldc1612_ramp.avr->cycle;
    }
    pthread_mutex_unlock(&ldc1612_ramp.lock);
}

/* 28-bit LDC1612 count at a given sim cycle: piecewise-linear in the
 * (sub-step-interpolated) net_descent. Fractional net_descent is fixed-
 * point Q.10 (*1024) so the threshold/clamp arithmetic stays in 64-bit
 * integers. Caller must NOT hold the ramp lock. */
#define LDC_FRAC_SHIFT 10
#define LDC_FRAC_ONE   ((int64_t)1 << LDC_FRAC_SHIFT)

static uint32_t
ldc1612_count_at_cycle(uint64_t cycle)
{
    uint32_t raw = 0;
    pthread_mutex_lock(&ldc1612_ramp.lock);
    if (ldc1612_ramp.avr) {
        /* Start from the integer net_descent at the most recent step
         * edge. Add a sub-step fraction projected forward at the
         * current step rate ONLY if (a) we have two same-direction
         * edges to estimate the interval from, AND (b) the projection
         * window is still inside one step interval (`since` <
         * `interval` - past that the toolhead is most likely stationary
         * and a clamped +1 step extrapolation would bias every settled
         * read up by one step, which the gt-only ramp callers care
         * about). When the projection window is past one interval, we
         * just return the integer count - byte-identical to the
         * original ramp's read-time-of-count behaviour. */
        int64_t frac = (int64_t)ldc1612_ramp.net_descent << LDC_FRAC_SHIFT;
        if (ldc1612_ramp.last_step_cycle && ldc1612_ramp.prev_step_cycle
            && ldc1612_ramp.last_step_cycle > ldc1612_ramp.prev_step_cycle
            && cycle > ldc1612_ramp.last_step_cycle) {
            uint64_t interval =
                ldc1612_ramp.last_step_cycle - ldc1612_ramp.prev_step_cycle;
            uint64_t since = cycle - ldc1612_ramp.last_step_cycle;
            if (since < interval) {
                int64_t step_frac =
                    (int64_t)((since << LDC_FRAC_SHIFT) / interval);
                if (ldc1612_ramp.prev_descending)
                    frac += step_frac;
                else
                    frac -= step_frac;
            }
        }
        /* Piecewise-linear: free_per_step until net_descent crosses
         * contact_descent, then depress_per_step. contact_descent <=
         * 0 disables the knee, so the original single-slope ramp
         * stays exactly equivalent for fixtures that don't opt in. */
        int64_t v;
        int64_t contact_frac =
            (int64_t)ldc1612_ramp.contact_descent << LDC_FRAC_SHIFT;
        if (ldc1612_ramp.contact_descent <= 0 || frac <= contact_frac) {
            v = (int64_t)ldc1612_ramp.baseline_raw
                + (frac * (int64_t)ldc1612_ramp.free_per_step
                   >> LDC_FRAC_SHIFT);
        } else {
            int64_t pre = (int64_t)ldc1612_ramp.baseline_raw
                + (contact_frac * (int64_t)ldc1612_ramp.free_per_step
                   >> LDC_FRAC_SHIFT);
            v = pre + ((frac - contact_frac)
                        * (int64_t)ldc1612_ramp.depress_per_step
                       >> LDC_FRAC_SHIFT);
        }
        /* Floor just above the calibration's lowest frequency (the
         * cfg's furthest cal point is z=5.0mm at 2.7 MHz; 0x1CEA000 =
         * 2.711 MHz). At high Z the count would fall below the
         * calibrated range and a scan / rapid_scan sample there maps to
         * "out of range"; clamping keeps every settled read a valid
         * in-range height. The floor must stay BELOW the descend
         * trigger frequency (height_to_freq(descend_z=0.4) = 2.875 MHz
         * = raw 0x1EAB6AA) or the 'gt' trigger is already satisfied at
         * arm time and homing fires instantly at any height - the
         * previous 0x2010000 (3.006 MHz) floor sat above it and pegged
         * the whole ramp, which also starved the tap's diff_peak
         * detector of any signal. */
        if (v < 0x1CEA000) v = 0x1CEA000;
        if (v > 0x03ffffff) v = 0x03ffffff;  /* MAX_VALID_RAW_VALUE */
        raw = (uint32_t)v;
    } else {
        raw = ldc1612_ramp.baseline_raw;
    }
    pthread_mutex_unlock(&ldc1612_ramp.lock);
    return raw;
}

/* Original ramp_current() entry point retained for callers that don't
 * want to track the read cycle (the bridge always has one available,
 * but a few non-time-sensitive paths still use it). */
static uint32_t
ldc1612_ramp_current(void)
{
    uint64_t now = ldc1612_ramp.avr ? ldc1612_ramp.avr->cycle : 0;
    return ldc1612_count_at_cycle(now);
}

/* bltouch <ctrl_port> <ctrl_pin> <sensor_port> <sensor_pin> <invert>
 * Configure the BLTouch state machine. Hooks the control pin to
 * decode klippy's PWM commands and drives the sensor pin to match.
 * invert mirrors klippy's `^!` flag on the sensor_pin in [bltouch]. */
static void
apply_bltouch(struct control_ctx *ctx,
              int ctrl_port_ord, int ctrl_pin,
              int sensor_port_ord, int sensor_pin,
              int invert)
{
    if (ctrl_port_ord < 'A' || ctrl_port_ord > 'L'
            || sensor_port_ord < 'A' || sensor_port_ord > 'L')
        return;
    if (ctrl_pin < 0 || ctrl_pin > 7
            || sensor_pin < 0 || sensor_pin > 7)
        return;
    pthread_mutex_lock(&bltouch_lock);
    bltouch.configured = 1;
    bltouch.ctrl_port = (char)ctrl_port_ord;
    bltouch.ctrl_pin = ctrl_pin;
    bltouch.sensor_port = (char)sensor_port_ord;
    bltouch.sensor_pin = sensor_pin;
    bltouch.invert = invert ? 1 : 0;
    bltouch.rise_cycle = 0;
    bltouch.state = BLT_RAISED;
    bltouch.sensor_pulsing = 0;
    g_avr_for_bltouch = ctx->avr;
    /* Initial sensor pin reflects RAISED (probe up, not triggered). */
    bltouch_drive_sensor(0);
    pthread_mutex_unlock(&bltouch_lock);

    avr_irq_t *ctrl_irq = avr_io_getirq(
        ctx->avr, AVR_IOCTL_IOPORT_GETIRQ((char)ctrl_port_ord),
        ctrl_pin);
    if (ctrl_irq)
        avr_irq_register_notify(ctrl_irq, bltouch_ctrl_hook, NULL);

    if (ctx->verbose)
        fprintf(stderr,
            "simavr_bridge: bltouch ctrl=%c%d sensor=%c%d invert=%d\n",
            (char)ctrl_port_ord, ctrl_pin,
            (char)sensor_port_ord, sensor_pin, invert);
}

/* Configure the software-I2C bit-bang ACK emulator on the given
 * SCL/SDA pin pair. Hooks both pins; on each ACK slot the bridge
 * drives SDA low. */
static void
apply_sw_i2c(struct control_ctx *ctx,
             int scl_port_ord, int scl_pin,
             int sda_port_ord, int sda_pin)
{
    if (scl_port_ord < 'A' || scl_port_ord > 'L'
            || sda_port_ord < 'A' || sda_port_ord > 'L')
        return;
    if (scl_pin < 0 || scl_pin > 7 || sda_pin < 0 || sda_pin > 7)
        return;
    pthread_mutex_lock(&sw_i2c_lock);
    sw_i2c.configured = 1;
    sw_i2c.scl_port = (char)scl_port_ord;
    sw_i2c.scl_pin = scl_pin;
    sw_i2c.sda_port = (char)sda_port_ord;
    sw_i2c.sda_pin = sda_pin;
    sw_i2c.active = 0;
    sw_i2c.bit_count = 0;
    sw_i2c.last_sda = 1;
    sw_i2c.last_scl = 1;
    sw_i2c.avr = ctx->avr;
    pthread_mutex_unlock(&sw_i2c_lock);

    avr_irq_t *scl_irq = avr_io_getirq(
        ctx->avr, AVR_IOCTL_IOPORT_GETIRQ((char)scl_port_ord), scl_pin);
    if (scl_irq && !g_sw_i2c_hook_registered[(scl_port_ord - 'A') * 8
                                              + scl_pin]) {
        avr_irq_register_notify(scl_irq, sw_i2c_scl_hook, NULL);
        g_sw_i2c_hook_registered[(scl_port_ord - 'A') * 8 + scl_pin] = 1;
    }
    avr_irq_t *sda_irq = avr_io_getirq(
        ctx->avr, AVR_IOCTL_IOPORT_GETIRQ((char)sda_port_ord), sda_pin);
    if (sda_irq && !g_sw_i2c_hook_registered[(sda_port_ord - 'A') * 8
                                              + sda_pin]) {
        avr_irq_register_notify(sda_irq, sw_i2c_sda_hook, NULL);
        g_sw_i2c_hook_registered[(sda_port_ord - 'A') * 8 + sda_pin] = 1;
    }

    if (ctx->verbose)
        fprintf(stderr,
            "simavr_bridge: sw_i2c scl=%c%d sda=%c%d\n",
            (char)scl_port_ord, scl_pin,
            (char)sda_port_ord, sda_pin);
}

static avr_irq_t *g_spi_in_irq = NULL;

/* Hook on the configured step pin. Fires every time simavr propagates
 * a write on that GPIO. We count 0->1 transitions and, once we've
 * seen <count_threshold> of them, raise the trigger pin's IRQ to the
 * configured value so the firmware's endstop sample loop reads a
 * "touch". A control command issued between samples re-arms by
 * resetting count_seen and armed. */
static void
step_pin_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)param;
    /* IRQ struct stores last-raised value as irq->value; simavr passes
     * the new value as `value`. Rising edge = previous 0, new 1. The
     * notify hook fires *before* irq->value is updated, so we can
     * compare directly. */
    int prev = (int)irq->value;
    int next = value ? 1 : 0;
    if (prev == next)
        return;  /* not a real edge - same level re-asserted */
    if (next == 0)
        return;  /* falling edge - klipper steps on rising only */

    /* simavr's IRQ doesn't tell us which pin we're on (one notifier
     * per (port,pin)), but the IRQ struct's name encodes it. Easier
     * to just check all entries against irq->irq number which is the
     * pin within the port - except we don't know the port that way.
     * Instead: scan all configured entries, advance the one(s) whose
     * stepper match this IRQ. We match by checking the IRQ pointer
     * via avr_io_getirq for each entry, but that's expensive. The
     * simpler approach: irq->irq is the pin number; combined with
     * irq->name lookup we could derive the port. Easiest: track which
     * entries are armed and use the IRQ's `irq` field (pin) plus a
     * walk to find the matching port. avr_io_getirq's returned irq
     * has its `irq` field = the pin number for IOPORT IRQs. The port
     * isn't directly accessible here, so we walk and check each entry
     * by re-querying the IRQ - if it matches our `irq` parameter, it's
     * this entry's pin. */
    avr_t *avr = g_avr_for_step;
    if (!avr)
        return;
    pthread_mutex_lock(&step_lock);
    char trig_port = 0;
    int trig_pin = 0, trig_val = 0;
    int should_fire = 0;
    for (int i = 0; i < step_trigs_count; i++) {
        if (!step_trigs[i].armed)
            continue;
        avr_irq_t *expect = avr_io_getirq(
            avr, AVR_IOCTL_IOPORT_GETIRQ(step_trigs[i].step_port),
            step_trigs[i].step_pin);
        if (expect != irq)
            continue;
        step_trigs[i].count_seen++;
        if (step_trigs[i].count_seen >= step_trigs[i].count_threshold) {
            trig_port = step_trigs[i].trigger_port;
            trig_pin = step_trigs[i].trigger_pin;
            trig_val = step_trigs[i].trigger_value;
            step_trigs[i].armed = 0;  /* one-shot per arm */
            should_fire = 1;
        }
        break;
    }
    pthread_mutex_unlock(&step_lock);

    if (should_fire) {
        avr_irq_t *t = avr_io_getirq(
            avr, AVR_IOCTL_IOPORT_GETIRQ(trig_port), trig_pin);
        if (t)
            avr_raise_irq(t, trig_val);
    }
}

/* Hook on a step_trigger entry's trigger pin. Whenever the pin
 * transitions back to its NOT-triggered state (the inverse of
 * trigger_value), reset count_seen and re-arm any matching entries
 * so the next round of step pulses can fire the trigger again.
 * Multi-sample probes (screws_tilt_adjust, bed_mesh) need this:
 * the BLTouch state machine resets the sensor to NOT-triggered
 * between samples, and we re-arm so the second/third probe can
 * see another touch.
 *
 * For one-shot endstops nothing else drives the trigger pin, so
 * after the home fires once the pin stays at trigger_value forever
 * and trigger_pin_hook never re-arms - matching the one-shot
 * semantics that home loops want. */
static void
trigger_pin_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)param;
    int prev = (int)irq->value;
    int next = value ? 1 : 0;
    if (prev == next)
        return;
    avr_t *avr = g_avr_for_step;
    if (!avr)
        return;
    pthread_mutex_lock(&step_lock);
    for (int i = 0; i < step_trigs_count; i++) {
        avr_irq_t *expect = avr_io_getirq(
            avr, AVR_IOCTL_IOPORT_GETIRQ(step_trigs[i].trigger_port),
            step_trigs[i].trigger_pin);
        if (expect != irq)
            continue;
        /* We only re-arm on transitions AWAY from trigger_value.
         * Transitions TO trigger_value are us firing the trigger. */
        if (next != step_trigs[i].trigger_value) {
            step_trigs[i].count_seen = 0;
            step_trigs[i].armed = 1;
        }
    }
    pthread_mutex_unlock(&step_lock);
}

/* Drive the SDA pin to a value (0 = ACK, 1 = release). The two-step
 * dance is required because simavr models per-pin IRQs vs. an
 * external-pull table separately:
 *   - SET_EXTERNAL persists across the firmware's PORT/DDR writes
 *     (the next update_irqs reads pull_value through external) so
 *     the firmware's "release SDA via input + pull-up" doesn't snap
 *     the line back high mid-ACK-slot. With the bridge's vendored
 *     simavr patch SET_EXTERNAL ALSO calls update_irqs immediately,
 *     so the new value lands in r_pin without waiting for a future
 *     PORT/DDR write
 *   - avr_raise_irq still updates the per-pin IRQ chain immediately
 *     (belt-and-suspenders for cases where the firmware reads PIN
 *     before any update_irqs has fired)
 * Sets self_driving so our SDA hook ignores the resulting IRQ
 * notification (otherwise our own ACK pulse looks like a START or
 * STOP and resets the bit counter). */
static void
sw_i2c_drive_sda(int value)
{
    if (!sw_i2c.avr || !sw_i2c.configured)
        return;
    pthread_mutex_lock(&sw_i2c_lock);
    sw_i2c.self_driving = 1;
    pthread_mutex_unlock(&sw_i2c_lock);
    avr_ioport_external_t ext = {
        .name = sw_i2c.sda_port,
        .mask = (uint8_t)(1U << sw_i2c.sda_pin),
        .value = value ? (uint8_t)(1U << sw_i2c.sda_pin) : 0,
    };
    avr_ioctl(sw_i2c.avr,
              AVR_IOCTL_IOPORT_SET_EXTERNAL(sw_i2c.sda_port), &ext);
    avr_irq_t *sda = avr_io_getirq(
        sw_i2c.avr,
        AVR_IOCTL_IOPORT_GETIRQ(sw_i2c.sda_port),
        sw_i2c.sda_pin);
    if (sda)
        avr_raise_irq(sda, value ? 1 : 0);
    pthread_mutex_lock(&sw_i2c_lock);
    sw_i2c.self_driving = 0;
    pthread_mutex_unlock(&sw_i2c_lock);
}

static void
sw_i2c_sda_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    int next = value ? 1 : 0;
    pthread_mutex_lock(&sw_i2c_lock);
    if (sw_i2c.self_driving) {
        sw_i2c.last_sda = next;
        pthread_mutex_unlock(&sw_i2c_lock);
        return;
    }
    int prev = sw_i2c.last_sda;
    sw_i2c.last_sda = next;
    if (prev != next && sw_i2c.last_scl == 1) {
        if (prev == 1 && next == 0) {
            sw_i2c.active = 1;
            sw_i2c.bit_count = 0;
        } else if (prev == 0 && next == 1) {
            sw_i2c.active = 0;
            sw_i2c.bit_count = 0;
        }
    }
    pthread_mutex_unlock(&sw_i2c_lock);
}

static void
sw_i2c_scl_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    int next = value ? 1 : 0;
    int drive_low = 0, release = 0;
    pthread_mutex_lock(&sw_i2c_lock);
    if (sw_i2c.self_driving) {
        sw_i2c.last_scl = next;
        pthread_mutex_unlock(&sw_i2c_lock);
        return;
    }
    int prev = sw_i2c.last_scl;
    sw_i2c.last_scl = next;
    if (prev == next || !sw_i2c.active) {
        pthread_mutex_unlock(&sw_i2c_lock);
        return;
    }
    if (prev == 1 && next == 0) {
        sw_i2c.bit_count++;
        if (sw_i2c.bit_count == 8)
            drive_low = 1;
        else if (sw_i2c.bit_count == 9) {
            /* DON'T release SDA on the 9th SCL falling. simavr's
             * non-FILTERED IRQ chain re-fires our SCL hook on
             * unchanged-value raises during firmware PORT/DDR writes
             * that hit the same pin, which can spuriously advance
             * bit_count past 8 before the firmware actually samples
             * the ACK slot. Holding SDA low until the next START
             * keeps the ACK valid through the firmware's read; the
             * SDA hook releases via the STOP path or the next START
             * resets the counter. */
            sw_i2c.bit_count = 0;
        }
    }
    pthread_mutex_unlock(&sw_i2c_lock);
    if (drive_low)
        sw_i2c_drive_sda(0);
    else if (release)
        sw_i2c_drive_sda(1);
}

/* Hook on the BLTouch control pin. Klippy drives this with software
 * PWM whose high-pulse duration encodes the command. We capture the
 * cycle on each rising edge and decode the duration on the falling
 * edge, advancing the state machine and pulsing the sensor pin. */
static void
bltouch_ctrl_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)param;
    avr_t *avr = g_avr_for_bltouch;
    if (!avr)
        return;
    int prev = (int)irq->value;
    int next = value ? 1 : 0;
    if (prev == next)
        return;
    pthread_mutex_lock(&bltouch_lock);
    if (next == 1) {
        bltouch.rise_cycle = avr->cycle;
        pthread_mutex_unlock(&bltouch_lock);
        return;
    }
    /* Falling edge: decode pulse width. */
    uint64_t fall = avr->cycle;
    uint64_t pulse_cycles = fall - bltouch.rise_cycle;
    double pulse_s = (double)pulse_cycles / (double)avr->frequency;
    static const struct {
        const char *name;
        double seconds;
        int new_state;        /* -1 = no state change */
        int self_pulse;       /* 1 = pulse sensor "triggered" briefly */
    } cmds[] = {
        { "pin_down",   0.000650, BLT_DEPLOYED, 1 },
        { "touch_mode", 0.001165, BLT_ARMED,    1 },
        { "pin_up",     0.001475, BLT_RAISED,   0 },
        { "self_test",  0.001780, -1,           0 },
        { "reset",      0.002190, BLT_RAISED,   0 },
    };
    int best_idx = -1;
    double best_err = 1e9;
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        double err = pulse_s - cmds[i].seconds;
        if (err < 0) err = -err;
        if (err < best_err) {
            best_err = err;
            best_idx = (int)i;
        }
    }
    /* Tolerance: pulses farther than 0.3 ms from any known command
     * are PWM "off" pulses (duty=0) or measurement noise - ignore. */
    if (best_idx < 0 || best_err > 0.0003) {
        pthread_mutex_unlock(&bltouch_lock);
        return;
    }
    if (cmds[best_idx].new_state >= 0)
        bltouch.state = cmds[best_idx].new_state;
    if (cmds[best_idx].self_pulse) {
        /* Triggered briefly. Schedule the revert. (If a previous
         * pulse is still in flight, simavr's cycle timer queue
         * keeps both registrations - the latest one's revert wins
         * because each fires bltouch_drive_sensor(0) and that's
         * idempotent. To be tidy we mark sensor_pulsing=1; the
         * second timer just re-clears.) */
        bltouch.sensor_pulsing = 1;
        bltouch_drive_sensor(1);
        avr_cycle_count_t when = avr->cycle
            + (avr->frequency / 1000) * (BLT_PULSE_USEC / 1000);
        avr_cycle_timer_register(avr, when - avr->cycle,
                                 bltouch_pulse_end, NULL);
    } else {
        bltouch.sensor_pulsing = 0;
        bltouch_drive_sensor(0);
    }
    pthread_mutex_unlock(&bltouch_lock);
}

/* sw_uart helpers ----------------------------------------------- */

/* CRC8-ATM (poly 0x07, MSB-first), matching klippy/extras/tmc_uart.py
 * MCU_TMC_uart_bitbang._calc_crc8 byte-for-byte. */
static uint8_t
sw_uart_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if (((crc >> 7) ^ (b & 0x01)) & 0x01)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

/* Drive the (single-wire) UART pin low or high via SET_EXTERNAL.
 * Used to pulse out a response. The pin is in firmware-input mode at
 * this point, so SET_EXTERNAL controls what gpio_in_read returns. */
static void
sw_uart_drive(struct sw_uart_state *u, int high)
{
    if (!u->avr)
        return;
    avr_ioport_external_t ext = {
        .name = u->port,
        .mask = (uint8_t)(1U << u->pin),
        .value = high ? (uint8_t)(1U << u->pin) : 0,
    };
    avr_ioctl(u->avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(u->port), &ext);
    avr_irq_t *irq = avr_io_getirq(u->avr,
                                   AVR_IOCTL_IOPORT_GETIRQ(u->port), u->pin);
    if (irq)
        avr_raise_irq(irq, high ? 1 : 0);
}

/* Decode the buffered RX bytes as a TMC2208/2209 datagram and react.
 *   Read req:  4 bytes [sync, addr, reg, crc]
 *   Write req: 8 bytes [sync, addr, reg|0x80, d0, d1, d2, d3, crc]
 * On read req we populate tx_buf with an 8-byte response and arm
 * the TX cycle timer. On write we just store the data in the
 * register file. */
static void
sw_uart_handle_frame(struct sw_uart_state *u)
{
    if (u->rx_len < 4)
        return;
    uint8_t sync = u->rx_buf[0];
    if (sync != 0x05 && sync != 0xf5)
        return;          /* not a recognized TMC sync byte - ignore */
    uint8_t reg_field = u->rx_buf[2];
    int is_write = (reg_field & 0x80) != 0;
    int expected = is_write ? 8 : 4;
    if (u->rx_len != expected)
        return;
    uint8_t crc = sw_uart_crc8(u->rx_buf, expected - 1);
    if (crc != u->rx_buf[expected - 1])
        return;
    uint8_t reg = (uint8_t)(reg_field & 0x7f);
    if (is_write) {
        uint32_t v = ((uint32_t)u->rx_buf[3] << 24)
                   | ((uint32_t)u->rx_buf[4] << 16)
                   | ((uint32_t)u->rx_buf[5] << 8)
                   |  (uint32_t)u->rx_buf[6];
        if (reg == 0x01) {
            /* GSTAT: write-1-to-clear semantics on real silicon. Each
             * 1 bit in the write clears the corresponding flag.
             * klippy's start_checks calls _query_register with
             * try_clear=True, which writes back the read value to
             * clear pending flags - if we just stored the written
             * value verbatim the reset bit would stay set and the
             * next read would shutdown with "GSTAT: 00000001
             * reset=1(Reset)" once motion starts. */
            u->regs[reg] = u->regs[reg] & ~v;
        } else {
            u->regs[reg] = v;
        }
        /* Bump IFCNT so klippy's write-verify ("did the chip ack the
         * write?") sees the counter advance after each WREG. The
         * firmware's tmc_uart driver reads IFCNT before and after the
         * write and raises "Unable to write tmc uart ... register X"
         * if they're equal. Real silicon increments IFCNT on every
         * successful write to a control register; mirror that here. */
        if (reg != 0x02)
            u->regs[0x02] = (u->regs[0x02] + 1) & 0xff;
        return;
    }
    /* Build 8-byte read response: [sync, master_addr=0xff, reg, d0..3, crc]
     * matches klippy._encode_write(0x05, 0xff, reg, val) - the firmware
     * round-trips this through _decode_read which calls _encode_write
     * with sync=0x05 to compare bytes. */
    uint32_t v = u->regs[reg];
    uint8_t resp[8];
    resp[0] = 0x05;
    resp[1] = 0xff;
    resp[2] = reg;
    resp[3] = (uint8_t)((v >> 24) & 0xff);
    resp[4] = (uint8_t)((v >> 16) & 0xff);
    resp[5] = (uint8_t)((v >> 8) & 0xff);
    resp[6] = (uint8_t)(v & 0xff);
    resp[7] = sw_uart_crc8(resp, 7);
    memcpy(u->tx_buf, resp, 8);
    u->tx_len = 8;
    u->tx_byte_pos = 0;
    u->tx_bit_pos = 0;
    u->tx_active = 1;
    /* Explicitly assert idle HIGH on the line so the firmware's
     * tmcuart_read_sync_event sees a HIGH-then-LOW transition (it
     * latches TU_READ_SYNC on HIGH, syncs on the subsequent LOW).
     * Without this it can race a stale EXTERNAL=0 from a prior chip
     * and the very first sample lands on our start bit, which
     * doesn't set TU_READ_SYNC and shifts the byte alignment by 1. */
    sw_uart_drive(u, 1);
    /* Phase-lock the first TX bit to the firmware's RX poll grid.
     *
     * tmcuart_send_finish_event runs at T_send_finish = bit 40 of the
     * encoded request stream = u->rx_last_cycle + bit_time/2 (we
     * record rx_last_cycle at the centre of the last byte's stop
     * slot, T_first_falling_last + 9.5*bt; firmware's send_finish
     * fires at T_first_falling_last + 10*bt). It then schedules
     * tmcuart_read_sync_event at T_send_finish + 4*bt, so the
     * firmware's poll grid is rx_last_cycle + (4.5 + n)*bt for
     * n = 0, 1, 2 ... .
     *
     * If we drove the start bit at any cycle aligned with that grid
     * we'd race against the firmware's two back-to-back gpio_in_read
     * calls (read_sync_event reads the pin, then recursively calls
     * read_event which reads it again a few AVR cycles later). With
     * the bridge transitioning from start bit (LOW) to data bit 0
     * between those two reads, the first read latches LOW (triggers
     * the sync transition) and the second latches data bit 0 - so
     * data[0][0] gets the data 0 value instead of the start bit's
     * LOW. _decode_read fails by exactly one bit at position 0.
     *
     * Targeting rx_last_cycle + 8*bt puts our first LOW drive at
     * T_firmware_first_poll + 3.5*bt: the firmware sees HIGH idle
     * for four polls (k = 0..3) - plenty to set TU_READ_SYNC - then
     * catches LOW at k = 4 with T_caught - C = 0.5*bt phase, so
     * every subsequent sample lands in the middle of the bridge's
     * bit drive period. Both gpio_in_read calls see the same
     * stable bit value. */
    uint64_t target_cycle = u->rx_last_cycle + (uint64_t)u->bit_time * 8;
    uint64_t now = u->avr->cycle;
    avr_cycle_count_t delay;
    if (target_cycle > now)
        delay = (avr_cycle_count_t)(target_cycle - now);
    else
        delay = u->bit_time;
    avr_cycle_timer_register(u->avr, delay, sw_uart_tx_bit, u);
}

/* simavr cycle timer callback: drive one bit of the response. */
static avr_cycle_count_t
sw_uart_tx_bit(struct avr_t *avr, avr_cycle_count_t when, void *param)
{
    (void)avr;
    struct sw_uart_state *u = (struct sw_uart_state *)param;
    pthread_mutex_lock(&sw_uart_lock);
    if (!u->tx_active || u->tx_byte_pos >= u->tx_len) {
        u->tx_active = 0;
        pthread_mutex_unlock(&sw_uart_lock);
        sw_uart_drive(u, 1);    /* idle line */
        return 0;
    }
    uint8_t b = u->tx_buf[u->tx_byte_pos];
    int bit_value;
    if (u->tx_bit_pos == 0)
        bit_value = 0;            /* start bit */
    else if (u->tx_bit_pos <= 8)
        bit_value = (b >> (u->tx_bit_pos - 1)) & 0x01;
    else
        bit_value = 1;            /* stop bit */
    u->tx_bit_pos++;
    if (u->tx_bit_pos > 9) {
        u->tx_bit_pos = 0;
        u->tx_byte_pos++;
    }
    uint32_t bt = u->bit_time;
    pthread_mutex_unlock(&sw_uart_lock);
    /* Drive after releasing the lock so the synchronous IRQ
     * dispatch (avr_raise_irq) inside the hook can briefly take
     * the lock without recursing on this thread. The hook checks
     * tx_active while we hold it elsewhere - safe because we set
     * tx_active=1 on entry and only clear it after the loop ends. */
    sw_uart_drive(u, bit_value);
    return when + bt;
}

/* simavr cycle timer callback: sample the next RX bit. */
static avr_cycle_count_t
sw_uart_rx_sample(struct avr_t *avr, avr_cycle_count_t when, void *param)
{
    (void)when;
    struct sw_uart_state *u = (struct sw_uart_state *)param;
    pthread_mutex_lock(&sw_uart_lock);
    if (!u->rx_in_byte) {
        pthread_mutex_unlock(&sw_uart_lock);
        return 0;
    }
    avr_irq_t *irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(u->port),
                                   u->pin);
    int bit = irq ? (int)(irq->value & 1) : 1;
    if (u->rx_bit_pos < 8) {
        if (bit)
            u->rx_byte |= (uint8_t)(1U << u->rx_bit_pos);
        u->rx_bit_pos++;
        uint32_t bt = u->bit_time;
        pthread_mutex_unlock(&sw_uart_lock);
        return when + bt;
    }
    /* Just sampled stop bit slot - byte complete. */
    if (u->rx_len < SW_UART_RX_BUF)
        u->rx_buf[u->rx_len++] = u->rx_byte;
    u->rx_in_byte = 0;
    u->rx_armed = 1;
    u->rx_last_cycle = avr->cycle;
    /* Schedule a frame-completion check: if no further start bit
     * within ~3 bit_times, treat the buffer as a complete frame. */
    pthread_mutex_unlock(&sw_uart_lock);
    return 0;
}

/* simavr cycle timer callback: if the line has been idle for long
 * enough, decode the buffered frame.  Re-arms itself on every fire
 * until either the frame is decoded or no bytes are pending. */
static avr_cycle_count_t
sw_uart_frame_check(struct avr_t *avr, avr_cycle_count_t when, void *param)
{
    struct sw_uart_state *u = (struct sw_uart_state *)param;
    pthread_mutex_lock(&sw_uart_lock);
    if (u->rx_in_byte) {
        uint32_t bt = u->bit_time;
        pthread_mutex_unlock(&sw_uart_lock);
        return when + bt * 4;
    }
    if (u->rx_len == 0) {
        pthread_mutex_unlock(&sw_uart_lock);
        return 0;
    }
    if (avr->cycle - u->rx_last_cycle < u->bit_time * 3) {
        uint32_t bt = u->bit_time;
        pthread_mutex_unlock(&sw_uart_lock);
        return when + bt * 4;
    }
    sw_uart_handle_frame(u);
    u->rx_len = 0;
    u->rx_armed = 1;
    pthread_mutex_unlock(&sw_uart_lock);
    return 0;
}

/* Pin IRQ hook: catches the start-bit falling edge and arms the RX
 * sample timer. Falling edges from our own TX driving land here too;
 * tx_active filters those out. */
static void
sw_uart_pin_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    struct sw_uart_state *u = (struct sw_uart_state *)param;
    if (!u || !u->avr)
        return;
    int next = value ? 1 : 0;
    if (next != 0)
        return;     /* only care about falling edges (start bits) */
    int register_frame_check = 0;
    pthread_mutex_lock(&sw_uart_lock);
    if (u->tx_active || u->rx_in_byte || !u->rx_armed) {
        pthread_mutex_unlock(&sw_uart_lock);
        return;
    }
    u->rx_in_byte = 1;
    u->rx_armed = 0;
    u->rx_bit_pos = 0;
    u->rx_byte = 0;
    if (u->rx_len == 0)
        register_frame_check = 1;
    avr_cycle_count_t delay = u->bit_time + u->bit_time / 2;
    pthread_mutex_unlock(&sw_uart_lock);
    avr_cycle_timer_register(u->avr, delay, sw_uart_rx_sample, u);
    if (register_frame_check)
        avr_cycle_timer_register(u->avr, u->bit_time * 4,
                                 sw_uart_frame_check, u);
}

/* sw_uart <port> <pin> <bit_time_cycles> <addr>
 * Configure a single-wire TMC UART slave on <port><pin>. */
static void
apply_sw_uart(struct control_ctx *ctx,
              int port_ord, int pin,
              int bit_time, int addr)
{
    if (port_ord < 'A' || port_ord > 'L' || pin < 0 || pin > 7)
        return;
    if (bit_time <= 0)
        bit_time = 1778;    /* TMC_BAUD_RATE_AVR=9000 baud at 16 MHz */
    if (sw_uart_count >= SW_UART_MAX)
        return;
    struct sw_uart_state *u = &sw_uart[sw_uart_count++];
    memset(u, 0, sizeof(*u));
    u->active = 1;
    u->port = (char)port_ord;
    u->pin = pin;
    u->avr = ctx->avr;
    u->bit_time = (uint32_t)bit_time;
    u->addr = (uint8_t)addr;
    u->rx_armed = 1;

    /* TMC2208/2209 default register values that the chip reports
     * after reset.  klippy probes a handful during init / DUMP_TMC;
     * supplying plausible values keeps the driver from flagging the
     * chip as unresponsive. IOIN reports the chip variant so klippy
     * can tell them apart - 0x21 = TMC2208 silentstepstick rev.
     * IFCNT is a write counter; klippy expects it to advance after
     * each WREG.  GSTAT reset value is 1 (reset flag set). */
    u->regs[0x00] = 0x00000040;   /* GCONF: pdn_disable=1 (UART mode) */
    u->regs[0x01] = 0x00000001;   /* GSTAT: reset flag */
    u->regs[0x06] = 0x21000040;   /* IOIN: version=0x21 (TMC2208) */
    u->regs[0x6f] = 0xc0000000;   /* DRV_STATUS: stst=1 (standstill) */

    avr_irq_t *p = avr_io_getirq(ctx->avr,
                                 AVR_IOCTL_IOPORT_GETIRQ((char)port_ord),
                                 pin);
    int idx = (port_ord - 'A') * 8 + pin;
    if (p && !g_sw_uart_hook_registered[idx]) {
        avr_irq_register_notify(p, sw_uart_pin_hook, u);
        g_sw_uart_hook_registered[idx] = 1;
    }
    if (ctx->verbose)
        fprintf(stderr, "simavr_bridge: sw_uart pin=%c%d bit_time=%u addr=%u\n",
                (char)port_ord, pin, u->bit_time, u->addr);
}

/* spi_tmc_chip <port> <pin> <proto>: register a TMC SPI chip on the
 * shared bus. The bridge hooks the CS pin so the active chip's
 * per-chip state (5-byte register file or 3-byte transaction buffer)
 * backs the bus while CS is low for this chip. Re-issuing for the
 * same pin updates the proto in place (idempotent for fixtures that
 * re-push their setup). */
static void
apply_spi_tmc_chip(struct control_ctx *ctx, int port_ord, int pin,
                   enum tmc_proto proto)
{
    if (port_ord < 'A' || port_ord > 'L')
        return;
    if (pin < 0 || pin > 7)
        return;
    char port = (char)port_ord;
    pthread_mutex_lock(&spi_state.lock);
    int existing = -1;
    for (int i = 0; i < tmc_chips_count; i++) {
        if (tmc_chips[i].cs_port == port && tmc_chips[i].cs_pin == pin) {
            existing = i;
            break;
        }
    }
    int slot = existing;
    if (slot < 0 && tmc_chips_count < TMC_CHIP_MAX) {
        slot = tmc_chips_count++;
        tmc_chips[slot].cs_port = port;
        tmc_chips[slot].cs_pin = pin;
        tmc_chips[slot].cs_active = 0;
        tmc_chips[slot].in_pos = 0;
        tmc_chips[slot].out_pos = 0;
        memset(tmc_chips[slot].in_buf, 0, sizeof(tmc_chips[slot].in_buf));
        memset(tmc_chips[slot].out_buf, 0, sizeof(tmc_chips[slot].out_buf));
        tmc_chips[slot].rdsel = 0;
        memset(tmc_chips[slot].regs, 0, sizeof(tmc_chips[slot].regs));
        /* Seed the per-chip 5-byte register defaults that spi_tmc
         * sets on the shared table - so a chip registered AFTER
         * spi_tmc init reads the same DRV_STATUS / MSCNT contract.
         * Harmless for tmc2660 (its decode uses different reg ids
         * and never indexes into this table). */
        tmc_chips[slot].regs[0x6f] = 0xc0050000;
        tmc_chips[slot].regs[0x6a] = 0;
    }
    if (slot >= 0)
        tmc_chips[slot].proto = proto;
    pthread_mutex_unlock(&spi_state.lock);
    if (slot < 0)
        return;
    if (existing < 0) {
        avr_irq_t *cs_irq = avr_io_getirq(
            ctx->avr, AVR_IOCTL_IOPORT_GETIRQ(port), pin);
        if (cs_irq) {
            avr_irq_register_notify(
                cs_irq, tmc_cs_hook, (void *)(intptr_t)slot);
        }
    }
    if (ctx->verbose) {
        const char *pname = (proto == TMC_PROTO_TMC2660) ? "tmc2660"
                                                         : "tmc5byte";
        fprintf(stderr,
                "simavr_bridge: spi_tmc_chip %c%d %s (slot %d)\n",
                port, pin, pname, slot);
    }
}

/* CS-pin hook for a registered TMC SPI chip. CS is active-low: on
 * the falling edge we set tmc_active_chip so spi_out_hook routes the
 * transaction through this chip's per-chip state; on the rising
 * edge we clear it (5-byte) or decode the accumulated 3-byte
 * datagram (tmc2660). */
static void
tmc_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    int idx = (int)(intptr_t)param;
    if (idx < 0 || idx >= tmc_chips_count)
        return;
    pthread_mutex_lock(&spi_state.lock);
    struct tmc_chip *c = &tmc_chips[idx];
    int was_active = c->cs_active;
    int now_active = (value == 0);
    c->cs_active = now_active;
    if (now_active && !was_active) {
        /* CS just went low: this chip's state backs the bus until
         * CS rises. For 5-byte chips that means writes/reads decode
         * into c->regs; the 5-byte accumulator in spi_state stays
         * authoritative for byte counting (the transaction is
         * always exactly 5 bytes long, started and ended by this
         * CS, so the accumulator cleanly resets when in_pos rolls
         * over at byte 4). For tmc2660 we additionally reset the
         * per-transaction MOSI buffer and pre-load the MISO
         * response (READRSP@RDSEL<rdsel>). When rdsel=2
         * (READRSP@RDSEL2 in tmc2660.py) klippy's periodic
         * _do_periodic_check uses the "se" field as the
         * cs_actual / current-scaler healthiness gate - if it reads
         * zero, klippy decodes it as "0(Reset?)" and shuts down
         * once motion starts. klippy's tmc2660 init sets RDSEL=2
         * first (see tmc2660.py "Must set RDSEL value first") so by
         * the time the periodic check runs we're already serving
         * RDSEL2. Pack a non-zero se: the field sits at data bits
         * 14..18 (after klippy's data = pr[0]<<16 | pr[1]<<8 | pr[2]
         * decode shifts the 20-bit response up by 4), so 5 << 10
         * in response_value lands as se=5 in the field-decoded
         * value. Other fields (stallguard / ot / sg_result / etc.)
         * stay zero - good enough for pretty_format and for init
         * not to flag anything. */
        tmc_active_chip = idx;
        if (c->proto == TMC_PROTO_TMC2660) {
            c->in_pos = 0;
            c->out_pos = 0;
            memset(c->in_buf, 0, sizeof(c->in_buf));
            uint32_t response_value = (c->rdsel == 2) ? (5U << 10) : 0;
            uint32_t packed = response_value << 4;
            c->out_buf[0] = (uint8_t)((packed >> 16) & 0xff);
            c->out_buf[1] = (uint8_t)((packed >> 8) & 0xff);
            c->out_buf[2] = (uint8_t)(packed & 0xff);
        } else {
            /* TMC_PROTO_5BYTE: clear the bus-level 5-byte
             * accumulator so a half-finished transaction from an
             * unregistered chip on the bus can't bleed into ours.
             * (In practice klippy CS-frames every transaction
             * cleanly, but staying defensive keeps the per-chip
             * state guarantee.) */
            spi_state.tmc_in_pos = 0;
            spi_state.tmc_out_pos = 0;
        }
    } else if (!now_active && was_active) {
        if (c->proto == TMC_PROTO_TMC2660 && c->in_pos == 3) {
            /* CS just went high on tmc2660: decode the 3 accumulated
             * MOSI bytes. The 20-bit datagram occupies the low 20
             * bits of the 24-bit stream (upper 4 bits dummy/zero):
             *   bits 19..17 = 3-bit register address
             *   bit  16     = LSB of (val>>16) -- always 0 except
             *                 for the optional bit-16 fields klippy
             *                 carries in DRVCONF/SGCSCONF/CHOPCONF
             *   bits 15..0  = val[15..0]
             * We only act on DRVCONF (reg-id=7) writes here,
             * latching the rdsel field for the next response. Other
             * writes are accepted but not stored - the test never
             * reads them back. */
            uint8_t b0 = c->in_buf[0];
            uint8_t reg_id = (uint8_t)((b0 >> 1) & 0x7);
            uint32_t val = ((uint32_t)(b0 & 1) << 16)
                         | ((uint32_t)c->in_buf[1] << 8)
                         | (uint32_t)c->in_buf[2];
            if (reg_id == 7) {
                /* DRVCONF: rdsel is the 2-bit field at val[5..4] */
                c->rdsel = (uint8_t)((val >> 4) & 0x3);
            }
        }
        if (tmc_active_chip == idx)
            tmc_active_chip = -1;
    }
    pthread_mutex_unlock(&spi_state.lock);
}

/* Drive a registered ADS1220 chip's DRDY pin high or low. Active-low
 * "data ready" semantics: drive LOW to tell the firmware a sample is
 * waiting, drive HIGH to make it back off. Mirrors the SET_EXTERNAL
 * + raise_irq pattern used by the gpio control command and sw_uart
 * so the value persists across firmware PORT/DDR writes and the
 * per-pin IRQ fires synchronously for any hooks listening. */
static void
ads1220_drive_drdy(struct ads1220_chip *c, int high)
{
    if (!c->avr)
        return;
    avr_ioport_external_t ext = {
        .name = (uint8_t)c->drdy_port,
        .mask = (uint8_t)(1U << c->drdy_pin),
        .value = high ? (uint8_t)(1U << c->drdy_pin) : 0,
    };
    avr_ioctl(c->avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(c->drdy_port), &ext);
    avr_irq_t *irq = avr_io_getirq(
        c->avr, AVR_IOCTL_IOPORT_GETIRQ(c->drdy_port), c->drdy_pin);
    if (irq)
        avr_raise_irq(irq, high ? 1 : 0);
    c->drdy_low = high ? 0 : 1;
}

/* CS-pin hook for a registered ADS1220 chip. CS is active-low so we
 * track which chip is currently being addressed; spi_out_hook uses
 * this to know whose DRDY to de-assert at the end of a 3-byte ADC
 * read in continuous mode. Multiple ADS1220 chips on the same SPI
 * bus assert their CS one at a time, so the single global
 * ads1220_active_chip suffices. */
static void
ads1220_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    int idx = (int)(intptr_t)param;
    if (idx < 0 || idx >= ads1220_chips_count)
        return;
    pthread_mutex_lock(&spi_state.lock);
    if (value == 0) {
        ads1220_active_chip = idx;
    } else if (ads1220_active_chip == idx) {
        ads1220_active_chip = -1;
    }
    pthread_mutex_unlock(&spi_state.lock);
}

/* CS-pin hook for a registered ADS131M0x chip. Falling edge selects
 * the chip and resets the frame position; rising edge closes the
 * frame - decode word0 (and word1 for WREG) and stage word0 of the
 * NEXT frame's MISO, which is how the real chip's one-frame-delayed
 * full-duplex response behaves. klippy's driver reads a response by
 * sending the command frame (response discarded) followed by a NULL
 * frame (response = staged value). */
static void
ads131_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    int idx = (int)(intptr_t)param;
    if (idx < 0 || idx >= ads131_chips_count)
        return;
    pthread_mutex_lock(&spi_state.lock);
    if (value == 0) {
        ads131_active_chip = idx;
        ads131_chips[idx].frame_pos = 0;
    } else if (ads131_active_chip == idx) {
        struct ads131_chip *c = &ads131_chips[idx];
        if (c->frame_pos >= 2) {
            uint16_t cmd = ((uint16_t)c->cmd_hi << 8) | c->cmd_lo;
            if ((cmd & 0xE000) == 0xA000) {          /* RREG */
                uint8_t reg = (cmd >> 7) & 0x3F;
                c->next_resp = reg < ADS131_REG_MAX ? c->regs[reg] : 0;
            } else if ((cmd & 0xE000) == 0x6000) {   /* WREG, 1 reg */
                uint8_t reg = (cmd >> 7) & 0x3F;
                uint16_t v = ((uint16_t)c->d0_hi << 8) | c->d0_lo;
                if (reg < ADS131_REG_MAX && reg != 0x00 && reg != 0x01)
                    c->regs[reg] = v;
                c->next_resp = (uint16_t)(0x4000 | (cmd & 0x1FFF));
            } else if (cmd == 0x0011) {              /* RESET */
                ads131_reset_regs(c);
                c->next_resp = 0xFF22;               /* RESET_ACK */
            } else {                                 /* NULL / other */
                c->next_resp = c->regs[0x01];        /* STATUS */
            }
        }
        ads131_active_chip = -1;
    }
    pthread_mutex_unlock(&spi_state.lock);
}

static void
adxl_cs_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    int idx = (int)(intptr_t)param;
    if (idx < 0 || idx >= adxl_chips_count)
        return;
    pthread_mutex_lock(&spi_state.lock);
    if (value == 0) {
        adxl_active_chip = idx;
        adxl_chips[idx].pos = 0;
        adxl_chips[idx].multi = 0;
        adxl_chips[idx].is_read = 0;
    } else if (adxl_active_chip == idx) {
        adxl_active_chip = -1;
    }
    pthread_mutex_unlock(&spi_state.lock);
}

/* spi_adxl345_chip <cs_port> <cs_pin> <vib_freq_hz> <amp_raw> <base_z_raw>
 * Register an ADXL345 chip's CS pin. vib_freq_hz / amp_raw give the
 * synthetic x-axis vibration (a pure tone TEST_RESONANCES can find);
 * base_z_raw is the constant z reading (256 = 1 g at the 3.9 mg/LSB
 * full-resolution mode klippy always configures). */
static void
apply_spi_adxl345_chip(struct control_ctx *ctx,
                       int cs_port_ord, int cs_pin,
                       int vib_freq_hz, int amp_raw, int base_z_raw)
{
    if (cs_port_ord < 'A' || cs_port_ord > 'L')
        return;
    if (cs_pin < 0 || cs_pin > 7)
        return;
    char cs_port = (char)cs_port_ord;
    pthread_mutex_lock(&spi_state.lock);
    int existing = -1;
    for (int i = 0; i < adxl_chips_count; i++) {
        if (adxl_chips[i].cs_port == cs_port
                && adxl_chips[i].cs_pin == cs_pin) {
            existing = i;
            break;
        }
    }
    int slot = existing;
    if (slot < 0 && adxl_chips_count < ADXL_CHIP_MAX) {
        slot = adxl_chips_count++;
        struct adxl_chip *c = &adxl_chips[slot];
        memset(c, 0, sizeof(*c));
        c->cs_port = cs_port;
        c->cs_pin = cs_pin;
        c->avr = ctx->avr;
        c->vib_freq_hz = vib_freq_hz > 0 ? vib_freq_hz : 45;
        c->amp_raw = amp_raw > 0 ? amp_raw : 256;
        c->base_z_raw = base_z_raw > 0 ? base_z_raw : 256;
        adxl_reset_regs(c);
    }
    pthread_mutex_unlock(&spi_state.lock);
    if (slot < 0)
        return;
    if (existing < 0) {
        avr_irq_t *cs_irq = avr_io_getirq(
            ctx->avr, AVR_IOCTL_IOPORT_GETIRQ(cs_port), cs_pin);
        if (cs_irq) {
            avr_irq_register_notify(
                cs_irq, adxl_cs_hook, (void *)(intptr_t)slot);
        }
        fprintf(stderr,
                "simavr_bridge: spi_adxl345_chip cs=%c%d vib=%dHz "
                "amp=%d base_z=%d\n",
                cs_port, cs_pin, adxl_chips[slot].vib_freq_hz,
                adxl_chips[slot].amp_raw, adxl_chips[slot].base_z_raw);
    }
}

/* simavr cycle timer callback: assert this chip's DRDY low to signal
 * "sample ready", then re-arm one period out. The firmware's poll
 * loop sees DRDY low and schedules a wake task that does the 3-byte
 * SPI read; spi_out_hook then drives DRDY back high. If the firmware
 * is too slow to read before the next period (sample dropped), DRDY
 * just stays low - benign, the overflow path the firmware already
 * handles for missed samples will catch it. */
static avr_cycle_count_t
ads1220_drdy_assert(struct avr_t *avr, avr_cycle_count_t when, void *param)
{
    (void)avr;
    struct ads1220_chip *c = (struct ads1220_chip *)param;
    if (!c->avr)
        return 0;
    pthread_mutex_lock(&spi_state.lock);
    if (!c->drdy_low) {
        pthread_mutex_unlock(&spi_state.lock);
        ads1220_drive_drdy(c, 0);
    } else {
        pthread_mutex_unlock(&spi_state.lock);
    }
    return when + c->period_cycles;
}

/* spi_ads131_chip <cs_port> <cs_pin> <id_hi>
 * Register an ADS131M0x chip's CS pin so the bridge serves its
 * init-sequence protocol (ID / RESET-ack / register write-verify).
 * id_hi is the ID register's high byte (0x22 = ADS131M02, 0x24 =
 * ADS131M04). */
static void
apply_spi_ads131_chip(struct control_ctx *ctx,
                      int cs_port_ord, int cs_pin, int id_hi)
{
    if (cs_port_ord < 'A' || cs_port_ord > 'L')
        return;
    if (cs_pin < 0 || cs_pin > 7)
        return;
    char cs_port = (char)cs_port_ord;
    pthread_mutex_lock(&spi_state.lock);
    int existing = -1;
    for (int i = 0; i < ads131_chips_count; i++) {
        if (ads131_chips[i].cs_port == cs_port
                && ads131_chips[i].cs_pin == cs_pin) {
            existing = i;
            break;
        }
    }
    int slot = existing;
    if (slot < 0 && ads131_chips_count < ADS131_CHIP_MAX) {
        slot = ads131_chips_count++;
        ads131_chips[slot].cs_port = cs_port;
        ads131_chips[slot].cs_pin = cs_pin;
        ads131_chips[slot].id_hi = (uint8_t)id_hi;
        ads131_reset_regs(&ads131_chips[slot]);
        ads131_chips[slot].next_resp = ads131_chips[slot].regs[0x01];
        ads131_chips[slot].frame_pos = 0;
    }
    pthread_mutex_unlock(&spi_state.lock);
    if (slot < 0)
        return;
    if (existing < 0) {
        avr_irq_t *cs_irq = avr_io_getirq(
            ctx->avr, AVR_IOCTL_IOPORT_GETIRQ(cs_port), cs_pin);
        if (cs_irq) {
            avr_irq_register_notify(
                cs_irq, ads131_cs_hook, (void *)(intptr_t)slot);
        }
        fprintf(stderr,
                "simavr_bridge: spi_ads131_chip cs=%c%d id=%02x\n",
                cs_port, cs_pin, id_hi & 0xff);
    }
}

/* spi_ads1220_chip <cs_port> <cs_pin> <drdy_port> <drdy_pin> <sample_rate_hz>
 * Register an ADS1220 chip's CS + DRDY pins so the bridge can pulse
 * DRDY at the chip's configured sample rate (replacing the previous
 * "hold DRDY low forever" approach which made the firmware read at
 * its full poll rate and overflow under stepper load). Re-issuing
 * for the same CS pin is a no-op (we keep the first registration). */
static void
apply_spi_ads1220_chip(struct control_ctx *ctx,
                       int cs_port_ord, int cs_pin,
                       int drdy_port_ord, int drdy_pin,
                       int sample_rate_hz)
{
    if (cs_port_ord < 'A' || cs_port_ord > 'L')
        return;
    if (cs_pin < 0 || cs_pin > 7)
        return;
    if (drdy_port_ord < 'A' || drdy_port_ord > 'L')
        return;
    if (drdy_pin < 0 || drdy_pin > 7)
        return;
    if (sample_rate_hz <= 0)
        sample_rate_hz = 660;
    char cs_port = (char)cs_port_ord;
    char drdy_port = (char)drdy_port_ord;
    pthread_mutex_lock(&spi_state.lock);
    int existing = -1;
    for (int i = 0; i < ads1220_chips_count; i++) {
        if (ads1220_chips[i].cs_port == cs_port
                && ads1220_chips[i].cs_pin == cs_pin) {
            existing = i;
            break;
        }
    }
    int slot = existing;
    if (slot < 0 && ads1220_chips_count < ADS1220_CHIP_MAX) {
        slot = ads1220_chips_count++;
        ads1220_chips[slot].cs_port = cs_port;
        ads1220_chips[slot].cs_pin = cs_pin;
        ads1220_chips[slot].drdy_port = drdy_port;
        ads1220_chips[slot].drdy_pin = drdy_pin;
        ads1220_chips[slot].period_cycles =
            (uint32_t)(ctx->avr->frequency / (uint32_t)sample_rate_hz);
        ads1220_chips[slot].drdy_low = 0;
        ads1220_chips[slot].avr = ctx->avr;
    }
    pthread_mutex_unlock(&spi_state.lock);
    if (slot < 0)
        return;
    if (existing < 0) {
        avr_irq_t *cs_irq = avr_io_getirq(
            ctx->avr, AVR_IOCTL_IOPORT_GETIRQ(cs_port), cs_pin);
        if (cs_irq) {
            avr_irq_register_notify(
                cs_irq, ads1220_cs_hook, (void *)(intptr_t)slot);
        }
        /* Drive DRDY HIGH initially so the firmware doesn't latch a
         * stale low from the fixture's earlier gpio command (or from
         * the pin's default state). The cycle timer will assert it
         * one period from now - staggered between chips so two
         * ads1220s on the same SPI bus don't fire DRDY in lockstep.
         *
         * Why stagger: each chip has its own poll timer and pending_flag
         * (src/sensor_ads1220.c ads1220_event); when DRDY goes low the
         * timer sets pending_flag and wakes the shared capture task.
         * The task processes EVERY pending_flag in one tick via
         * foreach_oid(oid, ads1220, command_config_ads1220), so if two
         * chips assert DRDY simultaneously the task issues two back-
         * to-back ads1220_read_adc -> add_sample -> sensor_bulk_report
         * sendf calls. The AVR's 96-byte transmit_buf
         * (src/generic/serial_irq.c) holds one sensor_bulk_data at a
         * time and CPU is far faster than UART, so the second sendf
         * arrives while the first is still queued and the new
         * command_encode_and_frame "encode if space" check drops it
         * (msglen=0 path, ed1f9963c). The SPI bus also stays busy
         * back-to-back, so the firmware's next DRDY poll observes
         * pending_flag still set and bumps possible_overflows -
         * load_cell_probe.py:check_sensor_errors then errors the test
         * out even when load_cell_probe's own samples got through.
         *
         * A half-period offset between slots gives the firmware enough
         * room to process each chip in a separate task tick: while
         * chip 0's pending_flag is being handled chip 1's DRDY is
         * still high, so foreach_oid sees only one pending chip per
         * tick. The shared poll-timer cadence (rest_ticks = clock /
         * (10*sps)) means chip 1's DRDY-low->task-wake hits ~half a
         * period after chip 0's, well clear of the back-to-back
         * window that was dropping messages and stacking SPI reads.
         *
         * Real hardware doesn't see this because most boards run one
         * ads1220 on a given AVR; the fixture-only multi-chip cfg
         * (test/klippy/load_cell.cfg covers both [load_cell ...] and
         * [load_cell_probe] section variants on one mcu) is a test-
         * coverage construct.
         *
         * Stagger size: the buffer holds one sensor_bulk_data
         * (~60 encoded bytes incl framing) at a time. AVR UART at
         * 250000 baud (10 bits/byte) drains 25 bytes/ms. The next
         * chip's sendf must arrive after the prior message has
         * drained far enough that the encoded payload still fits in
         * the post-compact free space (= sizeof(transmit_buf=96) -
         * (tmax - tpos)). 3/4 of a period at 660 SPS is 1.14 ms,
         * leaving ~35 bytes "still in flight" - just under the 38-
         * byte headroom the encode-if-space path can absorb. Smaller
         * fractions (e.g. period/N for N >= 3) overlap the encoded
         * msg with the in-flight bytes and drop. The 3/4 factor
         * spreads up to ADS1220_CHIP_MAX=4 chips across the period
         * with pairwise gaps no smaller than period/4, deferring the
         * "more chips than the bus can serve" regime to a real fix
         * if any test ever needs >2 chips. Bounded above by uint64
         * intermediate math; ADS1220_CHIP_MAX=4 and period_cycles
         * fits in uint32, so first_fire stays well under 2^34. */
        ads1220_drive_drdy(&ads1220_chips[slot], 1);
        uint64_t first_fire =
            (uint64_t)ads1220_chips[slot].period_cycles
            + ((uint64_t)slot * (uint64_t)ads1220_chips[slot].period_cycles
               * 3)
              / (uint64_t)ADS1220_CHIP_MAX;
        avr_cycle_timer_register(ctx->avr,
                                 (avr_cycle_count_t)first_fire,
                                 ads1220_drdy_assert,
                                 &ads1220_chips[slot]);
    }
    if (ctx->verbose)
        fprintf(stderr,
                "simavr_bridge: spi_ads1220_chip cs=%c%d drdy=%c%d "
                "rate=%dHz period=%u cycles (slot %d)\n",
                cs_port, cs_pin, drdy_port, drdy_pin,
                sample_rate_hz, ads1220_chips[slot].period_cycles, slot);
}

/* Hook on the firmware's SPI MOSI byte. Each time the firmware
 * writes SPDR, simavr fires this with the byte going OUT to the
 * (virtual) slave; we synchronously raise SPI_IRQ_INPUT with the
 * next byte from our response queue so the firmware reads back a
 * matching MISO byte on the same transfer. */
static void
spi_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (!g_spi_in_irq)
        return;
    uint8_t mosi = (uint8_t)(value & 0xff);
    uint8_t resp = 0x00;
    pthread_mutex_lock(&spi_state.lock);
    if (adxl_active_chip >= 0 && adxl_active_chip < adxl_chips_count) {
        /* A registered ADXL345's CS is low: byte 0 is the address +
         * READ/MULTI flags; subsequent bytes serve the register file,
         * the latched burst sample (0x32..0x37), or FIFO_STATUS
         * (0x39), with address auto-increment in MULTI mode. Writes
         * store into the register file; the POWER_CTL measure bit
         * transition arms/disarms FIFO accrual. Never let these bytes
         * reach the ADS1220 decoder below (shared bus). */
        struct adxl_chip *c = &adxl_chips[adxl_active_chip];
        if (c->pos == 0) {
            c->addr = mosi & 0x3f;
            c->is_read = (mosi & 0x80) != 0;
            c->multi = (mosi & 0x40) != 0;
            if (c->is_read && c->multi && c->addr == 0x32)
                adxl_latch_burst(c);
        } else if (c->is_read) {
            uint8_t a = c->addr;
            if (a >= 0x32 && a <= 0x37)
                resp = c->sample[a - 0x32];
            else if (a == 0x39)
                resp = c->fifo_after;
            else if (a < ADXL_REG_MAX)
                resp = c->regs[a];
            if (c->multi && c->addr < 0x3f)
                c->addr++;
        } else {
            uint8_t a = c->addr;
            if (a < ADXL_REG_MAX && a != 0x00) {
                c->regs[a] = mosi;
                if (a == 0x2D) {
                    int on = (mosi & 0x08) != 0;
                    if (on && !c->powered) {
                        c->powered = 1;
                        c->start_cycle = c->avr ? c->avr->cycle : 0;
                        c->served = 0;
                    } else if (!on) {
                        c->powered = 0;
                    }
                }
            }
            if (c->multi && c->addr < 0x3f)
                c->addr++;
        }
        c->pos++;
    } else if (ads131_active_chip >= 0
               && ads131_active_chip < ads131_chips_count) {
        /* A registered ADS131M0x chip's CS is low: serve its staged
         * response word in bytes 0..1 of the frame and capture the
         * command/data words; never let these bytes reach the ADS1220
         * decoder below (shared bus, different framing). The frame is
         * finalized in ads131_cs_hook on the CS rising edge. */
        struct ads131_chip *c = &ads131_chips[ads131_active_chip];
        uint8_t pos = c->frame_pos;
        if (pos == 0) {
            c->cmd_hi = mosi;
            resp = (uint8_t)(c->next_resp >> 8);
        } else if (pos == 1) {
            c->cmd_lo = mosi;
            resp = (uint8_t)(c->next_resp & 0xff);
        } else if (pos == 3) {
            c->d0_hi = mosi;
        } else if (pos == 4) {
            c->d0_lo = mosi;
        }
        if (pos < 255)
            c->frame_pos = (uint8_t)(pos + 1);
    } else if (spi_state.ads_mode) {
        /* ADS1220 SPI: variable-length commands.
         * Byte 0 is the command:
         *   0x40|(reg<<2)|(n-1) WREG  : write n bytes (1..4) to reg
         *   0x20|(reg<<2)|(n-1) RREG  : read n bytes (1..4) from reg
         *   0x06 RESET  : clear all regs (klippy reads back zeros)
         *   0x08 START_SYNC : starts continuous conversion mode
         *   0x00 NOOP : when seen as the first byte of an isolated
         *               3-byte transfer, klippy's continuous-mode ADC
         *               read - the bridge synthesizes the 24-bit
         *               sample from probe_step delta-since-last-read.
         * On RREG we serve subsequent MOSI cycles with stored register
         * bytes; on WREG we capture them.  MISO during the command
         * byte itself is don't-care (klippy slices response[1:]). */
        /* Register file for RESET/RREG/WREG: the CS-active chip's own regs
         * (so two ADS1220 chips don't clobber each other's post-reset
         * readback), or the bus-wide fallback when no chip is addressed
         * (legacy single-chip-per-bus case). */
        uint8_t (*ads_regs)[4] =
            (ads1220_active_chip >= 0
             && ads1220_active_chip < ads1220_chips_count)
            ? ads1220_chips[ads1220_active_chip].regs
            : spi_state.ads_regs;
        if (spi_state.ads_remaining > 0 && spi_state.ads_streaming) {
            /* Continue serving an ADC sample started on the cmd byte. */
            resp = spi_state.ads_sample[spi_state.ads_byte_idx];
            spi_state.ads_byte_idx++;
            spi_state.ads_remaining--;
            if (spi_state.ads_remaining == 0) {
                spi_state.ads_streaming = 0;
                /* Sample fully consumed: de-assert DRDY (drive HIGH)
                 * for the addressed chip so the firmware backs off
                 * until the next periodic assertion fires. The DRDY
                 * pin has no bridge-side hooks, so driving it under
                 * spi_state.lock can't recurse. */
                if (ads1220_active_chip >= 0
                        && ads1220_active_chip < ads1220_chips_count) {
                    struct ads1220_chip *c =
                        &ads1220_chips[ads1220_active_chip];
                    if (c->drdy_low)
                        ads1220_drive_drdy(c, 1);
                }
            }
        } else if (spi_state.ads_remaining == 0) {
            /* Command byte. Decode and prime any follow-on phase. */
            uint8_t cmd = mosi;
            uint8_t hi = cmd & 0xf0;
            if (hi == 0x40 || hi == 0x20) {
                /* ADS1220 RREG/WREG layout: 0010_rrnn / 0100_rrnn
                 * where rr is the register (0..3) and nn is byte
                 * count - 1 (0..3). Reg field is just bits 3..2. */
                spi_state.ads_reg = (cmd >> 2) & 0x03;
                spi_state.ads_remaining = (cmd & 0x03) + 1;
                spi_state.ads_byte_idx = 0;
                spi_state.ads_is_read = (hi == 0x20);
            } else if (cmd == 0x06) {
                /* RESET: zero the active chip's register file so the
                 * post-reset read returns the expected all-zero state. */
                memset(ads_regs, 0, sizeof(spi_state.ads_regs));
            } else if (cmd == 0x00 && probe_step.active) {
                /* Continuous-mode ADC read.  Serve a ramped 24-bit
                 * sample = (steps_since_burst_start * force_per_step)
                 * while a step burst is active (last edge within
                 * reset_cycles), or 0 during quiet (tare) stretches.
                 * All timing is in MCU sim cycles so the firmware's
                 * SOS filter sees the same ramp profile every run
                 * regardless of host load. */
                pthread_mutex_lock(&probe_step.lock);
                int32_t sample = 0;
                if (probe_step.avr) {
                    uint64_t now = probe_step.avr->cycle;
                    uint64_t last = probe_step.last_step_cycle;
                    if (last != 0 && now - last <= probe_step.reset_cycles) {
                        uint32_t steps = probe_step.step_count
                            - probe_step.step_count_at_burst_start;
                        int64_t v = (int64_t)steps
                            * (int64_t)probe_step.force_per_step;
                        if (v > 0x7fffff) v = 0x7fffff;
                        if (v < -0x800000) v = -0x800000;
                        sample = (int32_t)v;
                    }
                }
                pthread_mutex_unlock(&probe_step.lock);
                spi_state.ads_sample[0] = (uint8_t)((sample >> 16) & 0xff);
                spi_state.ads_sample[1] = (uint8_t)((sample >> 8) & 0xff);
                spi_state.ads_sample[2] = (uint8_t)(sample & 0xff);
                resp = spi_state.ads_sample[0];
                spi_state.ads_byte_idx = 1;
                spi_state.ads_remaining = 2;
                spi_state.ads_streaming = 1;
            }
        } else if (spi_state.ads_is_read) {
            resp = ads_regs[spi_state.ads_reg][spi_state.ads_byte_idx];
            spi_state.ads_byte_idx++;
            spi_state.ads_remaining--;
        } else {
            ads_regs[spi_state.ads_reg][spi_state.ads_byte_idx] = mosi;
            spi_state.ads_byte_idx++;
            spi_state.ads_remaining--;
            resp = 0;
        }
    } else if (spi_state.tmc_mode && tmc_active_chip >= 0
               && tmc_chips[tmc_active_chip].proto == TMC_PROTO_TMC2660) {
        /* TMC2660 3-byte protocol: a registered tmc2660 chip's CS is
         * currently low. Route this MOSI byte through the per-chip
         * buffer; the pre-loaded out_buf was set on the CS falling
         * edge. The datagram is finalized in tmc_cs_hook on the
         * rising edge - we don't decode here because byte 3 doesn't
         * always close a transaction (e.g. spi_transfer pads, klippy
         * guarantees exactly 3 bytes per transaction so the
         * assumption holds, but doing it on CS keeps the contract
         * explicit). */
        struct tmc_chip *c = &tmc_chips[tmc_active_chip];
        if (c->out_pos < 3)
            resp = c->out_buf[c->out_pos++];
        if (c->in_pos < 3)
            c->in_buf[c->in_pos++] = mosi;
    } else if (spi_state.tmc_mode) {
        /* TMC SPI: 5-byte datagrams. MISO byte_n is the response
         * we computed at the END of the previous datagram. The
         * register file the decode reads/writes is the active
         * chip's per-chip table (if one is CS-active and was
         * registered via spi_tmc_chip), or the bus-default
         * spi_state.tmc_regs (legacy single-chip fallback). The
         * choice is captured once per datagram, at decode time,
         * so a CS toggle mid-byte-counter can't split a single
         * transaction across two register files - klippy CS-frames
         * each 5-byte transaction, so by the time we hit byte 4
         * the active chip is well-defined. */
        resp = spi_state.tmc_out_buf[spi_state.tmc_out_pos];
        if (spi_state.tmc_out_pos < 4)
            spi_state.tmc_out_pos++;
        spi_state.tmc_in_buf[spi_state.tmc_in_pos] = mosi;
        if (spi_state.tmc_in_pos < 4) {
            spi_state.tmc_in_pos++;
        } else {
            /* Datagram complete: decode address + data. */
            uint8_t addr = spi_state.tmc_in_buf[0];
            uint32_t data = ((uint32_t)spi_state.tmc_in_buf[1] << 24)
                          | ((uint32_t)spi_state.tmc_in_buf[2] << 16)
                          | ((uint32_t)spi_state.tmc_in_buf[3] << 8)
                          |  (uint32_t)spi_state.tmc_in_buf[4];
            uint8_t reg = addr & 0x7f;
            int is_write = (addr & 0x80) != 0;
            uint32_t *regs;
            if (tmc_active_chip >= 0
                && tmc_chips[tmc_active_chip].proto == TMC_PROTO_5BYTE)
                regs = tmc_chips[tmc_active_chip].regs;
            else
                regs = spi_state.tmc_regs;
            if (is_write)
                regs[reg] = data;
            uint32_t out = regs[reg];
            spi_state.tmc_out_buf[0] = 0;        /* SPI_STATUS = OK */
            spi_state.tmc_out_buf[1] = (out >> 24) & 0xff;
            spi_state.tmc_out_buf[2] = (out >> 16) & 0xff;
            spi_state.tmc_out_buf[3] = (out >> 8) & 0xff;
            spi_state.tmc_out_buf[4] = out & 0xff;
            spi_state.tmc_in_pos = 0;
            spi_state.tmc_out_pos = 0;
        }
    } else if (spi_state.len > 0) {
        resp = spi_state.bytes[spi_state.pos];
        spi_state.pos = (spi_state.pos + 1) % spi_state.len;
    }
    pthread_mutex_unlock(&spi_state.lock);
    avr_raise_irq(g_spi_in_irq, resp);
}

static avr_irq_t *g_twi_in_irq = NULL;

/* Hook on the firmware's TWI peripheral output. simavr packs the
 * msg/addr/data into a 32-bit value via avr_twi_irq_msg(); we decode
 * it to figure out what the firmware just did (start/addr/read/write)
 * and respond on the input IRQ.
 *
 * For klippy's tests, we model a "permissive slave that always ACKs"
 * combined with a global byte response queue:
 *   - START + ADDR + (READ or WRITE): respond ACK so the firmware
 *     thinks the slave is present.
 *   - WRITE: ACK the byte the firmware sent (we ignore the content -
 *     command bytes from klippy's i2c_write are accepted but not
 *     interpreted; the response queue determines what the next reads
 *     return).
 *   - READ: respond with the next byte from twi_state.bytes (round-
 *     robin), with ACK status. Klippy's i2c_read sees the byte.
 *   - STOP: end of transaction; nothing to do.
 */
static void
twi_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (!g_twi_in_irq)
        return;
    avr_twi_msg_irq_t v = { .u = { .v = value } };
    uint8_t msg = v.u.twi.msg;
    uint8_t addr = v.u.twi.addr;
    /* START condition: firmware is starting a new transaction. ACK
     * the addressing. simavr emits TWI_COND_START (not TWI_COND_ADDR)
     * for the START+address phase; the addr byte is in v.u.twi.addr
     * with the R/W bit in its LSB. Register-select state is reset
     * here only for write-mode addressing - the read-mode addressing
     * after a register-select write must preserve pending_reg. */
    if (msg & TWI_COND_START) {
        pthread_mutex_lock(&twi_state.lock);
        twi_state.current_addr = addr >> 1;
        if (!(addr & 0x01)) {
            /* Write addressing - new transaction, clear register
             * select so the next WRITE byte latches as the new reg. */
            twi_state.pending_reg_valid = 0;
        }
        twi_state.pending_pos = 0;
        pthread_mutex_unlock(&twi_state.lock);
        if (twi_state.verbose)
            fprintf(stderr, "simavr_bridge: twi addr=%02x %s\n",
                    twi_state.current_addr,
                    (msg & TWI_COND_READ) ? "read" : "write");
        avr_raise_irq(g_twi_in_irq,
                      avr_twi_irq_msg(TWI_COND_ACK, addr, 1));
        return;
    }
    /* Firmware writing a data byte. ACK it. The first byte of a
     * write phase is treated as the register-select byte for the
     * subsequent read in register-aware mode. */
    if (msg & TWI_COND_WRITE) {
        uint8_t data = v.u.twi.data;
        pthread_mutex_lock(&twi_state.lock);
        int latched = 0;
        if (twi_state.reg_count > 0 && !twi_state.pending_reg_valid) {
            twi_state.pending_reg = data;
            twi_state.pending_reg_valid = 1;
            latched = 1;
        }
        pthread_mutex_unlock(&twi_state.lock);
        (void)latched;
        avr_raise_irq(g_twi_in_irq,
                      avr_twi_irq_msg(TWI_COND_ACK, addr, 1));
        return;
    }
    /* Firmware reading a byte: in register-aware mode, look up the
     * register that was written in the preceding phase and serve
     * the next byte of its configured response. Falls back to the
     * flat round-robin queue if no register matches. */
    if (msg & TWI_COND_READ) {
        uint8_t resp = 0x00;
        pthread_mutex_lock(&twi_state.lock);
        int matched = -1;
        uint8_t preg = twi_state.pending_reg;
        int pvalid = twi_state.pending_reg_valid;
        uint8_t ppos = twi_state.pending_pos;
        if (twi_state.reg_count > 0 && twi_state.pending_reg_valid) {
            for (int i = 0; i < twi_state.reg_count; i++) {
                if (twi_state.reg_addrs[i] == twi_state.pending_reg) {
                    matched = i;
                    break;
                }
            }
        }
        /* LDC1612 eddy ramp: serve STATUS (paced UNREADCONV0) and the
         * DATA0 MSB/LSB from the climbing count so the virtual endstop
         * triggers as Z descends. The DATA0 MSB read consumes the
         * latch so the paired LSB read serves the same sample even if
         * a step fires between them. Chip-id regs (0x7e/0x7f) fall
         * through to the configured responses. ldc1612_ramp.lock is
         * only ever taken here and in the step hook - never around
         * twi_state.lock - so nesting it under twi_state.lock can't
         * deadlock; touching latched / latch_valid / has_pending_sample
         * is single-threaded (this TWI thread).
         *
         * Two STATUS gating paths share the block:
         *   - contact_descent > 0 (the tap path, eddy.fixture.json):
         *     uniform-period sampling. STATUS reports ready iff a
         *     sample is pending, and a new pending sample is materialized
         *     once now crosses next_period_cycle; latch the count AT
         *     next_period_cycle (sub-step interpolated) so the value
         *     reflects the precise period boundary, not firmware read
         *     time. DATA0_MSB consumes the latch.
         *   - contact_descent == 0 (gt-only fixtures): legacy 3/4-of-
         *     period gating off last_data_cycle, kept verbatim so
         *     fixtures that haven't opted in are byte-for-byte
         *     unchanged. */
        int ramp_served = 0;
        if (ldc1612_ramp.active && twi_state.pending_reg_valid) {
            uint8_t r = twi_state.pending_reg;
            uint8_t pos = twi_state.pending_pos;
            int tap_mode = (ldc1612_ramp.contact_descent > 0);
            if (r == 0x18) {                 /* STATUS: paced UNREADCONV0 */
                int ready = 1;
                if (ldc1612_ramp.avr && ldc1612_ramp.period_cycles) {
                    uint64_t now = ldc1612_ramp.avr->cycle;
                    if (tap_mode) {
                        /* Uniform-period gating: latch at exact
                         * multiples of period_cycles from the first
                         * poll, so successive DATA0 reads are exactly
                         * period_cycles apart in sim time regardless
                         * of firmware poll grid jitter. */
                        if (ldc1612_ramp.next_period_cycle == 0)
                            ldc1612_ramp.next_period_cycle = now;
                        if (ldc1612_ramp.has_pending_sample) {
                            ready = 1;
                        } else if (now >= ldc1612_ramp.next_period_cycle) {
                            uint64_t latch_cycle =
                                ldc1612_ramp.next_period_cycle;
                            /* count_at_cycle takes the ramp lock, so
                             * unlock-and-relock around the call. The
                             * field updates that follow are this thread
                             * only. */
                            pthread_mutex_unlock(&twi_state.lock);
                            uint32_t v = ldc1612_count_at_cycle(latch_cycle);
                            pthread_mutex_lock(&twi_state.lock);
                            ldc1612_ramp.latched = v;
                            ldc1612_ramp.latch_valid = 1;
                            ldc1612_ramp.has_pending_sample = 1;
                            ldc1612_ramp.next_period_cycle +=
                                ldc1612_ramp.period_cycles;
                            /* If we somehow missed a whole period
                             * (heavy host load skewing the tick mode),
                             * snap forward so we don't burst-emit
                             * back-to-back samples. */
                            while (ldc1612_ramp.next_period_cycle <= now)
                                ldc1612_ramp.next_period_cycle +=
                                    ldc1612_ramp.period_cycles;
                            ready = 1;
                        } else {
                            ready = 0;
                        }
                    } else {
                        uint64_t last = ldc1612_ramp.last_data_cycle;
                        /* Legacy gating, 3/4 of a period (see original
                         * comment): the firmware polls STATUS every
                         * period/2; ready every other poll locks a
                         * clean 2-poll cadence. */
                        ready = (last == 0
                                 || now - last
                                    >= ldc1612_ramp.period_cycles * 3 / 4);
                    }
                }
                if (pos == 0)
                    resp = 0x00;
                else
                    resp = ready ? 0x08 : 0x00;  /* STATUS_UNREADCONV0 */
                twi_state.pending_pos++;
                ramp_served = 1;
            } else if (r == 0x00) {          /* DATA0_MSB: latch + top 16b */
                uint32_t v;
                if (tap_mode && ldc1612_ramp.latch_valid) {
                    /* Tap-mode latch is materialized at STATUS-ready
                     * time (at the period boundary); MSB just reads
                     * the already-stored value. */
                    v = ldc1612_ramp.latched;
                } else {
                    pthread_mutex_unlock(&twi_state.lock);
                    v = tap_mode ? ldc1612_count_at_cycle(
                                       ldc1612_ramp.avr->cycle)
                                 : ldc1612_ramp_current();
                    pthread_mutex_lock(&twi_state.lock);
                }
                if (pos == 0) {
                    ldc1612_ramp.latched = v;
                    ldc1612_ramp.latch_valid = 1;
                    ldc1612_ramp.has_pending_sample = 0;
                    /* Mark this conversion consumed so legacy STATUS
                     * won't report another ready sample until
                     * period_cycles elapse. */
                    if (ldc1612_ramp.avr)
                        ldc1612_ramp.last_data_cycle = ldc1612_ramp.avr->cycle;
                    resp = (uint8_t)((v >> 24) & 0xff);
                } else {
                    resp = (uint8_t)((ldc1612_ramp.latched >> 16) & 0xff);
                }
                twi_state.pending_pos++;
                ramp_served = 1;
            } else if (r == 0x01) {          /* DATA0_LSB: latched low 16b */
                uint32_t v;
                if (ldc1612_ramp.latch_valid) {
                    v = ldc1612_ramp.latched;
                } else {
                    pthread_mutex_unlock(&twi_state.lock);
                    v = ldc1612_ramp_current();
                    pthread_mutex_lock(&twi_state.lock);
                }
                resp = (pos == 0) ? (uint8_t)((v >> 8) & 0xff)
                                  : (uint8_t)(v & 0xff);
                twi_state.pending_pos++;
                ramp_served = 1;
            }
        }
        if (ramp_served) {
            /* served above */
        } else if (matched >= 0 && twi_state.reg_lens[matched] > 0) {
            resp = twi_state.reg_data[matched]
                    [twi_state.pending_pos
                        % twi_state.reg_lens[matched]];
            twi_state.pending_pos++;
        } else if (twi_state.len > 0) {
            resp = twi_state.bytes[twi_state.pos];
            twi_state.pos = (twi_state.pos + 1) % twi_state.len;
        }
        pthread_mutex_unlock(&twi_state.lock);
        (void)preg; (void)pvalid; (void)ppos;
        avr_raise_irq(g_twi_in_irq,
                      avr_twi_irq_msg(TWI_COND_READ | TWI_COND_ACK,
                                      addr, resp));
        return;
    }
    /* STOP or other: nothing to send back. */
}

/* sscanf for the gpio command receives the port as a string. The
 * encoder above passes it as ASCII int, but we want the test harness
 * to send "gpio C 7 1" not "gpio 67 7 1". Re-parse the gpio line with
 * %c instead. */
static void
apply_control_line(struct control_ctx *ctx, char *line)
{
    /* Strip trailing newline/cr. */
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
        line[--n] = '\0';
    if (!n)
        return;
    /* Rewrite "gpio X N V" -> "gpio <ord(X)> N V" so the integer
     * scanf path works uniformly. */
    if (strncmp(line, "gpio ", 5) == 0 && line[5]
            && (line[6] == ' ' || line[6] == '\t')) {
        char rewritten[128];
        snprintf(rewritten, sizeof(rewritten), "gpio %d%s",
                 (int)(unsigned char)line[5], line + 6);
        apply_control_command(ctx, rewritten);
        return;
    }
    /* step_trigger has 6 args including two port-letter fields, more
     * than apply_control_command's sscanf format covers. Parse it
     * directly here. Port letters are passed as %c so we don't need
     * the gpio-style rewrite. */
    if (strncmp(line, "step_trigger ", 13) == 0) {
        char step_port = 0, trig_port = 0;
        int step_pin = -1, trig_pin = -1, trig_val = -1;
        unsigned int count = 0;
        int got = sscanf(line + 13, "%c %d %u %c %d %d",
                         &step_port, &step_pin, &count,
                         &trig_port, &trig_pin, &trig_val);
        if (got == 6) {
            apply_step_trigger(ctx,
                (int)(unsigned char)step_port, step_pin, count,
                (int)(unsigned char)trig_port, trig_pin, trig_val);
        }
        return;
    }
    if (strncmp(line, "bltouch ", 8) == 0) {
        char ctrl_port = 0, sensor_port = 0;
        int ctrl_pin = -1, sensor_pin = -1, invert = 0;
        int got = sscanf(line + 8, "%c %d %c %d %d",
                         &ctrl_port, &ctrl_pin,
                         &sensor_port, &sensor_pin, &invert);
        if (got == 5) {
            apply_bltouch(ctx,
                (int)(unsigned char)ctrl_port, ctrl_pin,
                (int)(unsigned char)sensor_port, sensor_pin, invert);
        }
        return;
    }
    if (strncmp(line, "sw_i2c ", 7) == 0) {
        char scl_port = 0, sda_port = 0;
        int scl_pin = -1, sda_pin = -1;
        int got = sscanf(line + 7, "%c %d %c %d",
                         &scl_port, &scl_pin, &sda_port, &sda_pin);
        if (got == 4) {
            apply_sw_i2c(ctx,
                (int)(unsigned char)scl_port, scl_pin,
                (int)(unsigned char)sda_port, sda_pin);
        }
        return;
    }
    if (strncmp(line, "probe_step ", 11) == 0) {
        char step_port = 0;
        int step_pin = -1;
        int reset_us = 0, force_per_step = 0;
        int got = sscanf(line + 11, "%c %d %d %d",
                         &step_port, &step_pin,
                         &reset_us, &force_per_step);
        if (got == 4) {
            apply_probe_step(ctx,
                (int)(unsigned char)step_port, step_pin,
                reset_us, force_per_step);
        }
        return;
    }
    if (strncmp(line, "ldc1612_ramp ", 13) == 0) {
        char step_port = 0, dir_port = 0;
        int step_pin = -1, dir_pin = -1, descend_level = 0, sample_rate = 0;
        unsigned int baseline_raw = 0, free_per_step = 0;
        long long contact_descent = 0;
        unsigned int depress_per_step = 0;
        /* The two contact_* fields are optional; older fixtures
         * (gt-threshold only, no tap) emit 8 fields - the parse just
         * leaves both at 0 and the piecewise/uniform-period path stays
         * disabled. */
        int got = sscanf(line + 13,
                         "%c %d %c %d %d %u %u %d %lld %u",
                         &step_port, &step_pin, &dir_port, &dir_pin,
                         &descend_level, &baseline_raw, &free_per_step,
                         &sample_rate, &contact_descent, &depress_per_step);
        if (got >= 7) {
            apply_ldc1612_ramp(ctx,
                (int)(unsigned char)step_port, step_pin,
                (int)(unsigned char)dir_port, dir_pin, descend_level,
                (uint32_t)baseline_raw, (uint32_t)free_per_step,
                sample_rate,
                (got >= 9) ? (int64_t)contact_descent : 0,
                (got >= 10) ? (uint32_t)depress_per_step : 0);
        }
        return;
    }
    if (strncmp(line, "sw_uart ", 8) == 0) {
        char port = 0;
        int pin = -1, bit_time = 0, addr = 0;
        int got = sscanf(line + 8, "%c %d %d %d",
                         &port, &pin, &bit_time, &addr);
        if (got == 4) {
            apply_sw_uart(ctx, (int)(unsigned char)port, pin,
                          bit_time, addr);
        }
        return;
    }
    if (strncmp(line, "spi_tmc_chip ", 13) == 0) {
        char port = 0;
        int pin = -1;
        char proto[16] = {0};
        int got = sscanf(line + 13, "%c %d %15s", &port, &pin, proto);
        if (got == 3) {
            enum tmc_proto p;
            int known = 1;
            if (strcmp(proto, "tmc2660") == 0)
                p = TMC_PROTO_TMC2660;
            else if (strcmp(proto, "tmc2130") == 0
                     || strcmp(proto, "tmc5160") == 0
                     || strcmp(proto, "tmc2240") == 0)
                p = TMC_PROTO_5BYTE;
            else
                known = 0;
            if (known)
                apply_spi_tmc_chip(ctx,
                                   (int)(unsigned char)port, pin, p);
        }
        return;
    }
    if (strncmp(line, "spi_ads131_chip ", 16) == 0) {
        int csp = 0, cspin = -1, idhi = 0;
        char cs_port_ch = 0;
        if (sscanf(line + 16, " %c %d %i", &cs_port_ch, &cspin, &idhi) == 3) {
            csp = (int)cs_port_ch;
            apply_spi_ads131_chip(ctx, csp, cspin, idhi);
        }
        return;
    }
    if (strncmp(line, "spi_adxl345_chip ", 17) == 0) {
        char cs_port_ch = 0;
        int cspin = -1, vib = 0, amp = 0, base = 0;
        if (sscanf(line + 17, " %c %d %d %d %d",
                   &cs_port_ch, &cspin, &vib, &amp, &base) == 5) {
            apply_spi_adxl345_chip(ctx, (int)cs_port_ch, cspin,
                                   vib, amp, base);
        }
        return;
    }
    if (strncmp(line, "spi_ads1220_chip ", 17) == 0) {
        char cs_port = 0, drdy_port = 0;
        int cs_pin = -1, drdy_pin = -1, rate = 0;
        int got = sscanf(line + 17, "%c %d %c %d %d",
                         &cs_port, &cs_pin, &drdy_port, &drdy_pin, &rate);
        if (got == 5) {
            apply_spi_ads1220_chip(ctx,
                                   (int)(unsigned char)cs_port, cs_pin,
                                   (int)(unsigned char)drdy_port, drdy_pin,
                                   rate);
        }
        return;
    }
    apply_control_command(ctx, line);
}

static void *
control_socket_thread(void *arg)
{
    struct control_ctx *ctx = arg;
    int srv = socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv < 0) {
        fprintf(stderr, "simavr_bridge: control socket(): %s\n",
                strerror(errno));
        return NULL;
    }
    struct sockaddr_un sa;
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", ctx->socket_path);
    unlink(ctx->socket_path);
    if (bind(srv, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        fprintf(stderr, "simavr_bridge: control bind(%s): %s\n",
                ctx->socket_path, strerror(errno));
        close(srv);
        return NULL;
    }
    chmod(ctx->socket_path, 0666);
    if (listen(srv, 1) < 0) {
        fprintf(stderr, "simavr_bridge: control listen: %s\n",
                strerror(errno));
        close(srv);
        return NULL;
    }
    while (g_running) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv, &rfds);
        if (select(srv + 1, &rfds, NULL, NULL, &tv) <= 0)
            continue;
        int cli = accept(srv, NULL, NULL);
        if (cli < 0)
            continue;
        char buf[256];
        size_t fill = 0;
        while (g_running) {
            FD_ZERO(&rfds);
            FD_SET(cli, &rfds);
            tv.tv_sec = 0; tv.tv_usec = 100000;
            int r = select(cli + 1, &rfds, NULL, NULL, &tv);
            if (r < 0) break;
            if (r == 0) continue;
            ssize_t n = read(cli, buf + fill, sizeof(buf) - 1 - fill);
            if (n <= 0) break;
            fill += (size_t)n;
            buf[fill] = '\0';
            char *start = buf;
            char *nl;
            while ((nl = strchr(start, '\n')) != NULL) {
                *nl = '\0';
                /* `barrier <usec>` blocks until simavr's cycle counter
                 * has advanced <usec> microseconds of simulated time
                 * past the moment we received the command, then writes
                 * "OK\n" back to the client. The runner uses this to
                 * ensure pushed IRQ events (avr_raise_irq) have been
                 * dispatched by the simavr loop before klippy starts
                 * sampling - much more reliable than a wall-clock
                 * sleep, which falls behind under host CPU load. */
                if (strncmp(start, "barrier", 7) == 0
                        && (start[7] == '\0' || start[7] == ' ')) {
                    unsigned int usec = 1000;
                    if (start[7] == ' ')
                        sscanf(start + 8, "%u", &usec);
                    /* Tell the main loop to advance the AVR to this cycle (in
                     * tick mode it is otherwise paused pre-tick - see
                     * g_barrier_target), then wait for it. avr->cycle is
                     * stable here: barriers are serialized by the runner's
                     * wait-for-OK, so between them the main loop is paused. */
                    g_barrier_target = ctx->avr->cycle
                        + (avr_cycle_count_t)usec
                          * (ctx->avr->frequency / 1000000ULL);
                    while (g_running && ctx->avr->cycle < g_barrier_target) {
                        struct timespec ts = {0, 1000000};  /* 1 ms */
                        nanosleep(&ts, NULL);
                    }
                    const char *ok = "OK\n";
                    ssize_t w = write(cli, ok, 3);
                    (void)w;
                    /* The runner sends this barrier as the last step of
                     * fixture setup, right before it launches klippy. Ask
                     * the main loop to restart the --duration baseline
                     * here so the wall-clock safety net is measured from
                     * ~klippy-start rather than from bridge start (which
                     * predates the whole setup gap). Without this, on a
                     * loaded host the multi-bridge startup gap can eat the
                     * 5 s --duration slack, the bridge exits mid-run, and
                     * klippy reports "Got EOF when reading from device".
                     * In tick mode the main loop also restarts at
                     * tick-client connect, which is tighter still. */
                    g_restart_deadline = 1;
                } else {
                    apply_control_line(ctx, start);
                }
                start = nl + 1;
            }
            /* Shift any partial line back to the front of the buffer. */
            size_t leftover = strlen(start);
            memmove(buf, start, leftover);
            fill = leftover;
        }
        close(cli);
    }
    close(srv);
    unlink(ctx->socket_path);
    return NULL;
}

/* Create a non-blocking AF_UNIX SOCK_STREAM listening socket at `path`.
 * Used for two links klippy connects to in tick mode:
 *   - the tick-lockstep socket (--tick-socket): klippy's reactor sends
 *     "advance <T>\n", we run to T, publish sim_time, reply "done <T>\n";
 *   - the host link (the serial transport, see suart_setup): a socket
 *     replaces the pty so klippy's writes are delivered synchronously
 *     (no n_tty line-discipline workqueue), making the byte stream the
 *     bridge reads independent of host scheduling.
 * The caller accept()s the connection from the main loop (non-blocking,
 * so an idle listen fd never stalls the simulator). */
static int
unix_listen_socket(const char *path)
{
    int srv = socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv < 0) {
        fprintf(stderr, "simavr_bridge: tick socket(): %s\n",
                strerror(errno));
        return -1;
    }
    int flags = fcntl(srv, F_GETFL, 0);
    if (flags >= 0)
        fcntl(srv, F_SETFL, flags | O_NONBLOCK);
    struct sockaddr_un sa;
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", path);
    unlink(path);
    if (bind(srv, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        fprintf(stderr, "simavr_bridge: tick bind(%s): %s\n",
                path, strerror(errno));
        close(srv);
        return -1;
    }
    chmod(path, 0666);
    if (listen(srv, 1) < 0) {
        fprintf(stderr, "simavr_bridge: tick listen: %s\n",
                strerror(errno));
        close(srv);
        return -1;
    }
    return srv;
}

/* Read one newline-terminated line from cli into buf (size>=2). The
 * caller's `*fill` accumulates partial reads across calls. Returns
 * length of the line (excluding NUL), 0 on EOF, or -1 on error. */
static ssize_t
tick_read_line(int cli, char *buf, size_t bufsize, size_t *fill)
{
    while (g_running) {
        char *nl = memchr(buf, '\n', *fill);
        if (nl) {
            size_t linelen = (size_t)(nl - buf);
            buf[linelen] = '\0';
            return (ssize_t)linelen;
        }
        if (*fill + 1 >= bufsize) {
            /* Line longer than buffer: treat as protocol error. */
            return -1;
        }
        ssize_t n = read(cli, buf + *fill, bufsize - 1 - *fill);
        if (n == 0) return 0;
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        *fill += (size_t)n;
    }
    return -1;
}

static int
write_slave_link(const char *path, const char *slave_path)
{
    /* Atomic write via tmp+rename so the parent never reads a partial
     * path. */
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) {
        fprintf(stderr, "simavr_bridge: open %s: %s\n", tmp, strerror(errno));
        return -1;
    }
    fprintf(f, "%s\n", slave_path);
    fclose(f);
    if (rename(tmp, path) < 0) {
        fprintf(stderr, "simavr_bridge: rename %s -> %s: %s\n",
                tmp, path, strerror(errno));
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ *
 * Synchronous host-link UART for tick mode.
 *
 * simavr's stock uart_pty runs a background pthread that select()s on
 * the pty and calls avr_raise_irq() to feed the AVR concurrently with
 * the main thread's avr_run(). simavr is not thread-safe, and in tick
 * mode the main thread runs the core in tight bursts while the pty
 * thread feeds a core that is FROZEN between `advance` commands - that
 * both races avr_run() (intermittent crashes) and overruns/loses bytes
 * destined for the RX FIFO. So in tick mode we do not use uart_pty:
 * instead we own the host link and shuttle bytes synchronously inside
 * each advance (feed host->AVR as the core runs, gated by the UART's
 * XON/XOFF flow-control IRQs; drain AVR->host after). Single-threaded,
 * deterministic - the same shape the Renode launcher already uses.
 *
 * The host link is an AF_UNIX SOCK_STREAM socket, NOT a pty. A pty's
 * slave->master delivery is asynchronous: the n_tty line discipline
 * moves bytes via the flush_to_ldisc workqueue, so a non-blocking read
 * of the master at a fixed cycle can catch klippy's flush mid-delivery
 * and split the command byte stream at a host-scheduling-dependent
 * point - jittering the cycle each RX byte reaches the AVR and making
 * the bridge's trace non-deterministic run to run. A stream socket has
 * no line discipline: klippy's write lands in the peer receive buffer
 * within the write() syscall, so our read sees each flush whole, at a
 * deterministic cycle. g_suart_listen is the listening socket;
 * g_suart_master is the accepted connection (set by the main loop once
 * klippy connects, -1 until then). */
static int g_suart_listen = -1;              /* host-link listen socket */
static int g_suart_master = -1;              /* accepted klippy connection */
static avr_irq_t *g_suart_in_irq = NULL;     /* host -> AVR (RX) */
static int g_suart_xoff = 0;                  /* 1 => AVR RX FIFO full */
static uint8_t g_suart_tx[16384];            /* AVR -> host pending */
static size_t g_suart_tx_len = 0;
static uint8_t g_suart_in[4096];             /* host bytes read, unfed */
static size_t g_suart_in_pos = 0, g_suart_in_len = 0;

/* Determinism trace (TICK_PROTOCOL_DESIGN.md 5.1), enabled only when
 * KLIPPY_TICK_TRACE is set. A running FNV-1a over every byte the AVR
 * emits, plus the total count: if the host link delivers klippy's
 * commands deterministically, the firmware executes identically and
 * these are byte-identical across runs. This is the observable that
 * the async-pty C2 jitter perturbed (~30% of runs); proving it stable
 * is the point of the socket transport. Ships disabled (zero cost). */
static uint64_t g_out_total = 0;
static uint64_t g_out_hash = 1469598103934665603ULL;  /* FNV-1a offset */
/* Bytes discarded because g_suart_tx was full. The drop path below is
 * unreachable while the host drains the link every dispatch iteration
 * (TICK_PROTOCOL_DESIGN.md 2.5 D-PRE) - a drop means that precondition
 * broke, which silently corrupts the serial stream, so count it and say
 * so rather than losing the byte without a trace. */
static uint64_t g_out_dropped = 0;

static void
suart_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    if (g_suart_tx_len >= sizeof(g_suart_tx)) {
        /* Count the drop but do NOT fold the byte into the trace: the
         * hash/total are the "what did the host actually receive" observable
         * (5.2), so a dropped byte must change them. Folding it in first -
         * as this hook used to - made the determinism proof blind to the one
         * failure mode the guard exists to catch: two runs that both drop
         * data still produced identical out_total/out_hash. */
        if (!g_out_dropped)
            fprintf(stderr, "simavr_bridge: host-link tx buffer full (%zu B),"
                    " DROPPING output - serial stream is now corrupt\n",
                    sizeof(g_suart_tx));
        g_out_dropped++;
        return;
    }
    g_out_total++;
    g_out_hash = (g_out_hash ^ (uint8_t)value) * 1099511628211ULL;
    g_suart_tx[g_suart_tx_len++] = (uint8_t)value;
}
static void
suart_xon_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)value; (void)param;
    g_suart_xoff = 0;
}
static void
suart_xoff_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)value; (void)param;
    g_suart_xoff = 1;
}

/* Bring up the synchronous host link: bind an AF_UNIX listen socket
 * (klippy connects to it; the main loop accept()s the connection into
 * g_suart_master), wire the AVR UART '0' TX / XON / XOFF IRQs to the
 * hooks above, grab the RX-input IRQ, and return the host-link path
 * (static buffer) for write_slave_link(). The socket path is derived
 * from the slave-link path so it lands in the same per-test tempdir.
 * A socket (not a pty) is what makes the byte stream deterministic -
 * see the block comment above on slave->master async pty delivery. */
static const char *
suart_setup(avr_t *avr, const char *link_path)
{
    static char hostpath[256];
    snprintf(hostpath, sizeof(hostpath), "%s.sock", link_path);
    g_suart_listen = unix_listen_socket(hostpath);
    if (g_suart_listen < 0)
        return NULL;
    /* g_suart_master stays -1 until klippy connects and the main loop
     * accept()s it; suart_refill_input/suart_drain_output no-op until
     * then. No termios: a socket has no line discipline to configure
     * (that is precisely why klipper's binary framing passes untouched
     * and synchronously). */
    /* Disable simavr's built-in stdio echo so UART output reaches our
     * IRQ hook rather than the bridge's stdout. */
    uint32_t f = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &f);
    f &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &f);
    avr_irq_t *out = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'),
                                   UART_IRQ_OUTPUT);
    g_suart_in_irq = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'),
                                   UART_IRQ_INPUT);
    avr_irq_t *xon = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'),
                                   UART_IRQ_OUT_XON);
    avr_irq_t *xoff = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'),
                                    UART_IRQ_OUT_XOFF);
    if (out)
        avr_irq_register_notify(out, suart_out_hook, NULL);
    if (xon)
        avr_irq_register_notify(xon, suart_xon_hook, NULL);
    if (xoff)
        avr_irq_register_notify(xoff, suart_xoff_hook, NULL);
    return hostpath;
}

/* Pull any bytes klippy wrote on the host-link socket into our staging
 * buffer. The socket read is synchronous: a single read() returns
 * klippy's whole pending flush (no n_tty workqueue to split it). */
static void
suart_refill_input(void)
{
    if (g_suart_in_pos < g_suart_in_len || g_suart_master < 0)
        return;
    ssize_t n = read(g_suart_master, g_suart_in, sizeof(g_suart_in));
    g_suart_in_pos = 0;
    g_suart_in_len = (n > 0) ? (size_t)n : 0;
}

/* Feed at most one queued host byte into the AVR if its RX FIFO has
 * room (not XOFF). Called once per avr_run() step so input is paced to
 * the firmware's consumption and never overruns the FIFO. The socket
 * read (refill) is throttled to once every 512 cycles when the staging
 * buffer is empty so an idle link doesn't spin on read()/EAGAIN. */
static void
suart_feed_one(uint64_t cycle)
{
    if (g_suart_xoff || g_suart_in_irq == NULL)
        return;
    if (g_suart_in_pos >= g_suart_in_len) {
        /* NB: this is a phase test, not an elapsed-cycle one - avr_run
         * strides 1-4 cycles and fast-forwards across SLEEP, so a stride can
         * step over the multiple of 512 and wait another full period. A TLA+
         * model of this path (tla/drain, DrainB) shows an unbounded refill
         * delay if a stride stays in lockstep with the mask. Real strides do
         * not sustain that, so this is a latency hazard rather than a live
         * defect, and it is left alone deliberately: changing the gate shifts
         * host->AVR byte timing on every test to close a hole nobody has
         * hit. */
        if (cycle & 0x1FF)
            return;
        suart_refill_input();
    }
    if (g_suart_in_pos < g_suart_in_len)
        avr_raise_irq(g_suart_in_irq, g_suart_in[g_suart_in_pos++]);
}

/* Flush AVR-emitted bytes to the host-link socket after an advance. */
static void
suart_drain_output(void)
{
    if (g_suart_master < 0 || g_suart_tx_len == 0)
        return;
    size_t off = 0;
    while (off < g_suart_tx_len) {
        ssize_t n = write(g_suart_master, g_suart_tx + off,
                          g_suart_tx_len - off);
        if (n > 0) {
            off += (size_t)n;
        } else {
            /* Send buffer full (EAGAIN) or transient error: KEEP the
             * unwritten remainder instead of dropping it. Unsolicited
             * responses (ADC sample bursts under the full streaming
             * quantum) are never retransmitted, so a drop is a lost
             * sample - and a host-scheduling-dependent one, which breaks
             * determinism. The tail drains on the next advance's call,
             * FIFO-preserved; klippy reassembles a split message via its
             * serialqueue input_pos accumulation. The O1 per-advance
             * output cap keeps the kept tail small. See
             * TICK_PROTOCOL_DESIGN.md KEEP-TAIL. */
            break;
        }
    }
    if (off < g_suart_tx_len) {
        memmove(g_suart_tx, g_suart_tx + off, g_suart_tx_len - off);
        g_suart_tx_len -= off;
    } else {
        g_suart_tx_len = 0;
    }
}

int
main(int argc, char *argv[])
{
    /* Never let a write() to a peer that has closed its end (the control
     * socket or the host-link pty) kill us: the default SIGPIPE action
     * terminates the process. klippy closes the fixture control socket
     * once it has pushed the fixture, so the control thread's later "OK"
     * ack writes hit EPIPE - without this the whole bridge dies silently
     * mid-test (no signal handler runs), the pty master closes, and
     * klippy reads EOF. Ignore SIGPIPE so those writes just return -1. */
    signal(SIGPIPE, SIG_IGN);
    /* Optional ldc1612_ramp diag trace (one line per step edge). Off by
     * default; opening here so it's live before the first control
     * command can register the ramp hook. */
    const char *ramp_trace_env = getenv("KLIPPY_LDC1612_RAMP_TRACE");
    if (ramp_trace_env && ramp_trace_env[0]) {
        g_ldc1612_ramp_trace = fopen(ramp_trace_env, "w");
        if (!g_ldc1612_ramp_trace)
            fprintf(stderr, "simavr_bridge: warning: cannot open"
                    " KLIPPY_LDC1612_RAMP_TRACE=%s\n", ramp_trace_env);
    }
    const char *elf_path = NULL;
    const char *slave_link_path = NULL;
    const char *control_socket_path = NULL;
    const char *mcu_name = "atmega2560";
    double duration_s = 0.0;
    int verbose = 0;
    const char *sim_time_path = NULL;
    const char *tick_socket_path = NULL;

    static struct option longopts[] = {
        {"elf",            required_argument, NULL, 'e'},
        {"slave-link",     required_argument, NULL, 'l'},
        {"control-socket", required_argument, NULL, 'c'},
        {"mcu",            required_argument, NULL, 'm'},
        {"duration",       required_argument, NULL, 'd'},
        {"verbose",        no_argument,       NULL, 'v'},
        {"sim-time-file",  required_argument, NULL, 't'},
        {"tick-socket",    required_argument, NULL, 'k'},
        {NULL, 0, NULL, 0},
    };
    int opt;
    const char *optstr = "e:l:c:m:d:vt:k:";
    while ((opt = getopt_long(argc, argv, optstr, longopts, NULL)) != -1) {
        switch (opt) {
        case 'e': elf_path = optarg; break;
        case 'l': slave_link_path = optarg; break;
        case 'c': control_socket_path = optarg; break;
        case 'm': mcu_name = optarg; break;
        case 'd': duration_s = atof(optarg); break;
        case 'v': verbose = 1; break;
        case 't': sim_time_path = optarg; break;
        case 'k': tick_socket_path = optarg; break;
        default:
            fprintf(stderr,
                "Usage: %s --elf <klipper.elf> --slave-link <path>\n"
                "       [--mcu <atmega2560>] [--duration <seconds>] [-v]\n",
                argv[0]);
            return 2;
        }
    }
    if (!elf_path || !slave_link_path) {
        fprintf(stderr, "simavr_bridge: --elf and --slave-link are required\n");
        return 2;
    }

    elf_firmware_t firmware;
    memset(&firmware, 0, sizeof(firmware));
    if (elf_read_firmware(elf_path, &firmware) != 0) {
        fprintf(stderr, "simavr_bridge: failed to load %s\n", elf_path);
        return 1;
    }

    avr_t *avr = avr_make_mcu_by_name(mcu_name);
    if (!avr) {
        fprintf(stderr, "simavr_bridge: unknown MCU %s\n", mcu_name);
        return 1;
    }
    avr_init(avr);
    avr_load_firmware(avr, &firmware);

    /* Force the simulator's MCU clock rate to klipper's actual F_CPU.
     * simavr reads AVR_MMCU_TAG_FREQUENCY out of the ELF if the
     * firmware sets it, but klipper's build doesn't, so simavr falls
     * back to its 1MHz default - which leaves timer interrupts firing
     * 16x too slowly and the host's clock-sync immediately diverges.
     * All AVR boards klipper supports run at 16MHz on the test
     * configs, so override unconditionally here. */
    avr->frequency = 16000000;
    /* Set VCC/AVCC to 5V so the ADC peripheral converts our pushed
     * millivolt values using the same reference klipper assumes
     * (REFS=AVCC by default). simavr's vref defaults to 3.3V, which
     * would map a 3054 mV "room temperature" thermistor to a 13-bit
     * sum of ~7600 instead of ~5000 - klipper's thermistor formulas
     * then read it as too hot and trip max_temp checks. */
    avr->vcc = 5000;
    avr->avcc = 5000;
    avr->aref = 0;

    /* Disable simavr's STDIO mode for UART0 so the firmware's UART
     * traffic doesn't get echoed to our stderr. uart_pty (below) is
     * what actually moves bytes. */
    uint32_t uart_flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &uart_flags);
    uart_flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &uart_flags);

    /* Host link. In tick mode use the single-threaded synchronous UART
     * (suart_*, above) so byte movement is deterministic and avoids the
     * uart_pty pump thread racing avr_run(). In free-run (non-tick) mode
     * keep uart_pty: its pump thread overlapping continuous avr_run() is
     * fine and we get its pty + flow-control plumbing for free. */
    int tick_mode = (tick_socket_path != NULL);
    static uart_pty_t pty;
    const char *host_slavename = NULL;
    if (tick_mode) {
        host_slavename = suart_setup(avr, slave_link_path);
        if (!host_slavename)
            return 1;
    } else {
        /* uart_pty is simavr's prebuilt UART<->pty bridge. It opens a
         * pty pair, runs an internal pump thread that drains the master
         * fd into simavr's UART input IRQ (respecting XON/XOFF), and
         * forwards UART output to the master fd so the slave reads it. */
        uart_pty_init(avr, &pty);
        uart_pty_connect(&pty, '0');
        host_slavename = pty.pty.slavename;
    }

    if (verbose)
        fprintf(stderr,
            "simavr_bridge: host link %s mcu %s freq %u tick=%d\n",
            host_slavename, mcu_name, avr->frequency, tick_mode);

    /* Make the slave node accessible to other processes in the
     * container (klippy generally runs as a different uid in real
     * deployments, but in tests both run as root - even so, openpty
     * leaves the slave at mode 0620 which is restrictive). */
    if (chmod(host_slavename, 0666) < 0) {
        fprintf(stderr, "simavr_bridge: chmod %s 0666: %s\n",
                host_slavename, strerror(errno));
    }

    /* Hook the SPI MOSI byte stream so each firmware write to SPDR
     * gets a matching MISO response from our queue. atmega2560 has
     * a single SPI peripheral named '0' in simavr's IOCTL space. */
    avr_irq_t *spi_out_irq =
        avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT);
    g_spi_in_irq =
        avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT);
    if (spi_out_irq && g_spi_in_irq)
        avr_irq_register_notify(spi_out_irq, spi_out_hook, NULL);

    /* Hook the TWI (I2C) peripheral for any test that talks to an
     * I2C device. simavr's TWI is named '0' on atmega2560 (single
     * TWI bus) and uses msg/addr/data triplets per IRQ event. */
    twi_state.verbose = verbose;
    avr_irq_t *twi_out_irq =
        avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT);
    g_twi_in_irq =
        avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
    if (twi_out_irq && g_twi_in_irq)
        avr_irq_register_notify(twi_out_irq, twi_out_hook, NULL);

    /* Bring up the tick-mode listen socket BEFORE publishing the
     * slave_link so klippy (which only starts once the slave_link
     * exists) is guaranteed a successful connect on its first try. */
    int tick_listen_fd = -1;
    if (tick_socket_path) {
        tick_listen_fd = unix_listen_socket(tick_socket_path);
        if (tick_listen_fd < 0)
            return 1;
        if (verbose)
            fprintf(stderr, "simavr_bridge: tick socket %s\n",
                    tick_socket_path);
    }

    if (write_slave_link(slave_link_path, host_slavename) < 0)
        return 1;

    /* Spawn the control socket listener if asked. The thread runs
     * concurrently with the simulator main loop and applies fixture
     * commands to simavr's peripheral models via avr_raise_irq. */
    pthread_t control_tid;
    int control_started = 0;
    static struct control_ctx ctl_ctx;
    if (control_socket_path) {
        ctl_ctx.avr = avr;
        ctl_ctx.verbose = verbose;
        snprintf(ctl_ctx.socket_path, sizeof(ctl_ctx.socket_path),
                 "%s", control_socket_path);
        if (pthread_create(&control_tid, NULL,
                           control_socket_thread, &ctl_ctx) == 0) {
            control_started = 1;
            if (verbose)
                fprintf(stderr,
                    "simavr_bridge: control socket %s\n",
                    control_socket_path);
        } else {
            fprintf(stderr,
                "simavr_bridge: failed to spawn control thread\n");
        }
    }
    (void)control_started;  /* used implicitly via _exit() at end */

    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);
    g_crash_avr = avr;
    signal(SIGSEGV, on_crash);
    signal(SIGABRT, on_crash);

    /* Duration safety net: kills the simulator after N WALL seconds
     * so a hung firmware test doesn't run forever in CI.
     *
     * The baseline (start_ts) is captured here, before klippy has even
     * started - so it includes the whole startup gap (the runner waits
     * for the slave link, pushes the fixture, runs a setup `barrier`,
     * and for multi-MCU configs does all of that for every bridge before
     * launching klippy). The runner sizes --duration as klippy's work
     * window (EMULATOR_KLIPPY_DEADLINE) plus a small slack, so if the
     * startup gap exceeds that slack the deadline fires WHILE klippy is
     * still running: the bridge _exit()s, its pty master closes, and
     * klippy reads a spurious EOF ("Got EOF when reading from device").
     * That made the pure-simavr multi_mcu_avr* tests (largest startup
     * gap: two bridges) flaky under host load. The renode launcher
     * already avoids this by (re)starting its duration clock only once
     * startup is done; we mirror that here by restarting start_ts when
     * the fixture-setup barrier completes (g_restart_deadline) and, in
     * tick mode, again when klippy connects on the tick socket. Both
     * land the baseline at ~klippy-start so the slack covers exactly the
     * work window, the same as on real hardware's wall-clock-free run. */
    struct timespec start_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);
    uint64_t deadline_wall_ns = duration_s > 0
        ? (uint64_t)(duration_s * 1e9)
        : 0;

    /* Sim-time mode: if --sim-time-file is given, mmap a double and
     * write avr->cycle/frequency (current MCU time in seconds) on
     * each periodic check. klippy reads the same file via
     * KLIPPY_SIM_TIME_FILE so its get_monotonic() returns simulated
     * time instead of clock_gettime. With this active we DROP the
     * wall-clock throttle and let simavr free-run as fast as the host
     * allows; klippy's view of time is consistent with the MCU's
     * regardless of host load, which makes tests deterministic. */
    volatile double *sim_time_ptr = NULL;
    if (sim_time_path) {
        int fd = open(sim_time_path, O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            fprintf(stderr, "simavr_bridge: open %s: %s\n",
                    sim_time_path, strerror(errno));
        } else {
            double zero = 0.;
            if (write(fd, &zero, sizeof zero) != (ssize_t)sizeof zero) {
                fprintf(stderr, "simavr_bridge: write %s: %s\n",
                        sim_time_path, strerror(errno));
            }
            void *p = mmap(NULL, sizeof(double), PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, 0);
            close(fd);
            if (p == MAP_FAILED) {
                fprintf(stderr, "simavr_bridge: mmap %s: %s\n",
                        sim_time_path, strerror(errno));
            } else {
                sim_time_ptr = (volatile double *)p;
                *sim_time_ptr = 0.;
            }
        }
    }

    int state = cpu_Running;
    uint64_t throttle_check_interval = avr->frequency / 1000;
    uint64_t next_throttle_cycle = throttle_check_interval;
    /* Stall diagnostic (BRIDGE_TICK_DIAG): log every advance the bridge reads
     * and every `done` it writes, so when a tick run wedges (mechanism (2),
     * TICK_PROTOCOL_DESIGN.md 4.1) the per-bridge log shows whether this
     * bridge stopped issuing work and at which advance. Ships disabled. */
    int tick_diag = (getenv("BRIDGE_TICK_DIAG") != NULL);
    uint64_t tick_diag_seq = 0;
    int tick_client_fd = -1;
    char tick_buf[256];
    size_t tick_fill = 0;
    /* Optional determinism trace (KLIPPY_TICK_TRACE), tick mode only.
     * One CSV line per advance: seq target_cycle end_cycle out_total
     * out_hash dropped (see g_out_hash / g_out_dropped). The filename
     * matches the tick socket's basename so multi-MCU runs get one trace
     * per bridge. `dropped` is 0 on any healthy run; nonzero means the
     * 2.5 D-PRE drain precondition broke and the stream is corrupt. */
    FILE *g_tick_trace = NULL;
    uint64_t g_tick_seq = 0;
    const char *trace_env = getenv("KLIPPY_TICK_TRACE");
    if (trace_env && trace_env[0] && tick_socket_path) {
        const char *base = strrchr(tick_socket_path, '/');
        base = base ? base + 1 : tick_socket_path;
        char trace_path[PATH_MAX];
        snprintf(trace_path, sizeof(trace_path), "%s.%s", trace_env, base);
        g_tick_trace = fopen(trace_path, "w");
    }
    while (g_running && state != cpu_Done && state != cpu_Crashed) {
        /* Restart the wall-clock --duration baseline once the runner has
         * finished pushing the fixture (its setup `barrier` set this from
         * the control thread). This drops the startup gap out of the
         * duration window so the safety net is measured from ~klippy
         * start. See the start_ts capture above for why. */
        if (g_restart_deadline) {
            g_restart_deadline = 0;
            clock_gettime(CLOCK_MONOTONIC, &start_ts);
        }
        /* Accept klippy's host-link connection (tick mode: the serial
         * transport is a Unix domain socket, not a pty - see suart_setup).
         * Non-blocking; runs every iteration so the link comes up whether
         * we are still free-running or already ticking. Inert in free-run
         * mode where g_suart_listen is -1 (uart_pty owns the pty there). */
        if (g_suart_listen >= 0 && g_suart_master < 0) {
            int hfd = accept(g_suart_listen, NULL, NULL);
            if (hfd >= 0) {
                int fl = fcntl(hfd, F_GETFL, 0);
                if (fl >= 0)
                    fcntl(hfd, F_SETFL, fl | O_NONBLOCK);
                g_suart_master = hfd;
                if (verbose)
                    fprintf(stderr, "simavr_bridge: host link connected "
                            "at cycle %llu\n",
                            (unsigned long long)avr->cycle);
            }
        }
        /* Once klippy connects on the tick socket we leave free-run
         * mode and only advance simavr in response to "advance" lines.
         * Until then (during fixture-setup `barrier`s on the control
         * socket) we run free with the wall-clock throttle below. */
        if (tick_listen_fd >= 0 && tick_client_fd < 0) {
            int fd = accept(tick_listen_fd, NULL, NULL);
            if (fd >= 0) {
                tick_client_fd = fd;
                tick_fill = 0;
                /* klippy is now live and about to drive the test - this
                 * is the tightest possible duration baseline (the renode
                 * launcher's equivalent of "link up, start the clock").
                 * Reset here so a slow/loaded startup can never make the
                 * bridge time out before klippy's own deadline. */
                clock_gettime(CLOCK_MONOTONIC, &start_ts);
                if (verbose)
                    fprintf(stderr,
                        "simavr_bridge: tick client connected at cycle %llu\n",
                        (unsigned long long)avr->cycle);
            }
        }
        if (tick_client_fd >= 0) {
            ssize_t llen = tick_read_line(tick_client_fd, tick_buf,
                                          sizeof(tick_buf), &tick_fill);
            if (llen <= 0) {
                if (verbose)
                    fprintf(stderr,
                        "simavr_bridge: tick client closed at cycle %llu\n",
                        (unsigned long long)avr->cycle);
                break;
            }
            double target = 0.;
            int parsed = sscanf(tick_buf, "advance %lf", &target);
            /* Shift any bytes after the parsed line back to the front. */
            size_t consumed = (size_t)llen + 1;  /* include the '\n' */
            if (consumed <= tick_fill) {
                memmove(tick_buf, tick_buf + consumed, tick_fill - consumed);
                tick_fill -= consumed;
            } else {
                tick_fill = 0;
            }
            if (parsed != 1) {
                /* Unrecognized message - ack with current time and let
                 * klippy keep going rather than wedge the test. */
                char reply[64];
                int rl = snprintf(reply, sizeof(reply), "done %.9f\n",
                                  (double)avr->cycle / (double)avr->frequency);
                if (write(tick_client_fd, reply, rl) != rl) break;
                continue;
            }
            /* Advance to the first cycle STRICTLY past the requested
             * sim-time. klippy's reactor schedules timers (incl. the
             * greenlet wakeups behind reactor.pause()) at arbitrary float
             * times and fires them when eventtime >= waketime. The AVR
             * clock only lands on integer cycles, so truncating
             * target*freq would stop the AVR a fraction of a cycle BELOW
             * a waketime that sits between two cycles -- the timer would
             * never fire, klippy would re-request the identical advance,
             * and tick mode would livelock (identify never completes).
             * The +1 guarantees sim_time = avr->cycle/freq > target, so
             * any timer at <= target is reached. (The Renode path doesn't
             * need this: emulation RunFor takes a time, not cycles.) */
            avr_cycle_count_t target_cycle =
                (avr_cycle_count_t)(target * (double)avr->frequency) + 1;
            /* L1: the cycle->double->cycle round trip is lossy, so even
             * with the +1 above target_cycle can floor back to == avr->cycle
             * when target*freq lands a hair below the current cycle. That is
             * a zero-cycle advance: sim_time is republished unchanged,
             * klippy's pending timer (a sub-cycle fraction above) never
             * fires, it re-requests the identical target, and the handshake
             * wedges with sim_time pinned - the cold-start identify stall and
             * the idle livelock. Force >=1 cycle of progress so sim_time
             * strictly increases on every advance. Mirrors the renode
             * launcher's `if delta_us == 0: delta_us = 1000`. See
             * TICK_PROTOCOL_DESIGN.md L1. */
            if (target_cycle <= avr->cycle)
                target_cycle = avr->cycle + 1;
            if (tick_diag)
                fprintf(stderr, "TICKDIAG advance seq=%llu target=%.6f"
                        " cur_cyc=%llu tgt_cyc=%llu\n",
                        (unsigned long long)tick_diag_seq, target,
                        (unsigned long long)avr->cycle,
                        (unsigned long long)target_cycle);
            avr_cycle_count_t _stall_prev = avr->cycle;
            uint64_t _stall_n = 0;
            /* O1 (TICK_PROTOCOL_DESIGN.md): bound per-advance output so the
             * KEEP-TAIL drain stays within ~one socket buffer and g_suart_tx
             * (16 KiB) never overflows; < the 4096 read size. The remainder
             * runs on the next advance. The round-trip cadence is set by the
             * reactor's bounded quantum (small while klippy waits on a
             * reply/trigger, full while streaming) - the bridge is a dumb
             * "run to T" with NO output-burst early-exit, which under the
             * single-threaded reactor receive would chop streaming into one
             * round trip per sample (load_cell 0.15x real time). */
            const size_t OUTPUT_CAP = 3072;
            while (g_running && avr->cycle < target_cycle
                   && state != cpu_Done && state != cpu_Crashed) {
                /* Synchronous host-link shuttle (tick mode): feed one
                 * pending klippy byte into the AVR per step (flow-control
                 * gated) so RX bytes are delivered as the core runs,
                 * never to a frozen core. No-op when g_suart_in_irq is
                 * NULL (free-run mode uses uart_pty instead). */
                suart_feed_one(avr->cycle);
                state = avr_run(avr);
                /* Stall guard: if the core stops advancing its cycle
                 * counter (e.g. SLEEP with no pending cycle-timer to
                 * fast-forward to) we would spin here forever. Bail to
                 * the target so the advance completes and klippy's timer
                 * still fires (the firmware will simply have no new
                 * output this quantum). */
                if (avr->cycle != _stall_prev) {
                    _stall_prev = avr->cycle;
                    _stall_n = 0;
                } else if (++_stall_n > 200000) {
                    if (verbose)
                        fprintf(stderr, "TICKDIAG STALL at cyc=%llu "
                                "target=%llu state=%d\n",
                                (unsigned long long)avr->cycle,
                                (unsigned long long)target_cycle, state);
                    avr->cycle = target_cycle;
                    break;
                }
                /* O1: cap per-advance output. Checked after avr_run so the
                 * advance always makes >=1 cycle of progress (no zero-cycle
                 * advance -> no livelock); the remainder runs next advance. */
                if (g_suart_tx_len >= OUTPUT_CAP)
                    break;
            }
            /* Flush AVR-emitted bytes to the socket after the advance. */
            suart_drain_output();
            if (sim_time_ptr)
                *sim_time_ptr = (double)avr->cycle / (double)avr->frequency;
            if (g_tick_trace) {
                /* Trailing field is the drop count (always 0 on a healthy
                 * run); a nonzero column makes a broken D-PRE visible in the
                 * trace instead of silently corrupting the byte stream. */
                fprintf(g_tick_trace, "%llu %llu %llu %llu %llu %llu\n",
                        (unsigned long long)g_tick_seq++,
                        (unsigned long long)target_cycle,
                        (unsigned long long)avr->cycle,
                        (unsigned long long)g_out_total,
                        (unsigned long long)g_out_hash,
                        (unsigned long long)g_out_dropped);
                fflush(g_tick_trace);
            }
            /* Honor the wall-clock duration safety net even in tick
             * mode so a runaway test still terminates. */
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            uint64_t wall_ns = (uint64_t)(now.tv_sec - start_ts.tv_sec)
                                * 1000000000ULL
                             + (now.tv_nsec - start_ts.tv_nsec);
            if (deadline_wall_ns && wall_ns >= deadline_wall_ns) {
                if (verbose)
                    fprintf(stderr,
                        "simavr_bridge: wall deadline reached at cycle %llu\n",
                        (unsigned long long)avr->cycle);
                break;
            }
            char reply[64];
            int rl = snprintf(reply, sizeof(reply), "done %.9f\n",
                              (double)avr->cycle / (double)avr->frequency);
            if (write(tick_client_fd, reply, rl) != rl) {
                if (verbose)
                    fprintf(stderr,
                        "simavr_bridge: tick reply write failed: %s\n",
                        strerror(errno));
                break;
            }
            if (tick_diag)
                fprintf(stderr, "TICKDIAG done seq=%llu end_cyc=%llu\n",
                        (unsigned long long)tick_diag_seq++,
                        (unsigned long long)avr->cycle);
            continue;
        }
        if (tick_listen_fd >= 0) {
            /* Tick mode, klippy not yet connected: deterministic setup
             * (TICK_PROTOCOL_DESIGN.md 4). Advance the AVR ONLY toward a
             * pending fixture-setup barrier; pause otherwise. This makes
             * avr->cycle - and hence every fixture cycle-timer's phase
             * (ADS1220 DRDY, sw_uart, ...) - at tick-connect identical
             * across runs, independent of host scheduling, which is what
             * lets the reactor-driven receive give bit-reproducible feature
             * tests. (Free-run / non-tick tests have tick_listen_fd < 0 and
             * fall through to the wall-clock free-run below as before.) */
            if (g_barrier_target && avr->cycle < g_barrier_target) {
                /* Advance to the barrier target in a TIGHT loop - no
                 * per-iteration accept()/suart/sim_time syscalls. The
                 * runner's barrier is multi-second of simulated time but it
                 * only waits ~5 s wall for the "OK"; a one-avr_run-per-main-
                 * loop-iteration path is too slow under that syscall overhead,
                 * so the barrier wouldn't finish in time, the runner would
                 * launch klippy MID-advance, and the AVR cycle (and every
                 * fixture timer phase) at tick-connect would become host-
                 * timing dependent - the residual non-determinism the trace
                 * proof caught. A tight loop reaches the target well under
                 * the timeout, so klippy always connects at the same cycle. */
                while (g_running && avr->cycle < g_barrier_target
                       && state != cpu_Done && state != cpu_Crashed) {
                    state = avr_run(avr);
                }
                if (sim_time_ptr)
                    *sim_time_ptr = (double)avr->cycle / (double)avr->frequency;
            } else {
                struct timespec ts = {0, 200000};  /* 0.2 ms idle wait */
                nanosleep(&ts, NULL);
            }
            continue;
        }
        state = avr_run(avr);
        if (avr->cycle < next_throttle_cycle)
            continue;
        next_throttle_cycle = avr->cycle + throttle_check_interval;
        if (sim_time_ptr) {
            *sim_time_ptr = (double)avr->cycle / (double)avr->frequency;
        }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t dsec = (uint64_t)(now.tv_sec - start_ts.tv_sec);
        uint64_t wall_ns = dsec * 1000000000ULL
                         + (now.tv_nsec - start_ts.tv_nsec);
        if (deadline_wall_ns && wall_ns >= deadline_wall_ns) {
            if (verbose)
                fprintf(stderr,
                    "simavr_bridge: wall deadline reached at cycle %llu\n",
                    (unsigned long long)avr->cycle);
            break;
        }
        /* Cap simavr to wall-clock as an upper bound. In wall-clock
         * mode (no sim-time-file) this is the throttle: klippy's
         * clock_gettime-based view of time must match the MCU's. In
         * sim-time mode it's still useful as a CEILING - it prevents
         * simavr from running ahead of wall-clock, which would let
         * klippy's heater_verify (and other timer thresholds in
         * simulated seconds) fire faster than the test runner's
         * wall-clock deadline. simavr is allowed to fall behind
         * wall-clock under load; klippy's view of time uses our
         * simulated cycle counter and adapts. */
        uint64_t expected_wall_ns =
            (avr->cycle * 1000000000ULL) / avr->frequency;
        if (expected_wall_ns > wall_ns) {
            uint64_t sleep_ns = expected_wall_ns - wall_ns;
            if (sleep_ns > 0) {
                struct timespec ts = {
                    .tv_sec = sleep_ns / 1000000000ULL,
                    .tv_nsec = sleep_ns % 1000000000ULL,
                };
                nanosleep(&ts, NULL);
            }
        }
    }

    g_running = 0;
    if (verbose)
        fprintf(stderr,
            "simavr_bridge: exit state=%d cycles=%llu\n",
            state, (unsigned long long)avr->cycle);
    /* uart_pty_stop occasionally hangs on its internal pthread_join
     * when the pty was closed by the host. Skip the orderly shutdown
     * and let the kernel reclaim everything - the test runner has
     * already terminated the host side, nothing else needs cleanup. */
    fflush(stderr);
    fflush(stdout);
    _exit(0);
}
