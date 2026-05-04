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
 *   spi_tmc_chip <cs_port> <cs_pin> tmc2660
 *       Register a TMC2660 chip on the shared TMC SPI bus. The
 *       bridge hooks the chip's CS pin and routes MOSI bytes through
 *       a 3-byte (20-bit) datagram decoder while CS is asserted.
 *       Other TMC SPI variants on the same bus continue using the
 *       default 5-byte path. Up to TMC_CHIP_MAX chips supported.
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
#include <limits.h>
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
#include <time.h>
#include <unistd.h>

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

static void
on_signal(int sig)
{
    (void)sig;
    g_running = 0;
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
    uint32_t tmc_regs[256];    /* register file shared by all TMC chips
                                * on the bus - klippy writes-then-reads
                                * each chip sequentially, no contention */
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

/* TMC2660 SPI variant. Unlike TMC2130/5160/2240 (5-byte/40-bit
 * datagrams), TMC2660 uses 3-byte/20-bit datagrams: the upper 4 bits
 * of the wire are dummy, then bits 19..17 are a 3-bit register
 * address (0/4/5/6/7) and bits 16..0 are the register value. The
 * MISO response is the chip's READRSP@RDSEL<n> register, packed the
 * same way (response<<4 in 24 bits) so klippy's
 * MCU_TMC2660_SPI.get_register_raw decodes it via
 *   data = (pr[0] << 16) | (pr[1] << 8) | pr[2]
 * which leaves the 20-bit response shifted up by 4 (matching the
 * field offsets in tmc2660.py: stallguard at bit 4, mstep at 14..23).
 *
 * On a shared SPI bus with mixed-protocol chips (e.g. tmc2130 +
 * tmc2660 in the tmc_spi.test config), the bridge needs CS-pin
 * awareness to know which protocol to apply per transaction - the
 * 5-byte byte counter alone would walk off-alignment after each
 * 3-byte tmc2660 transaction. We register a CS-pin output hook for
 * each tmc2660 chip listed in the fixture; on falling edge we mark
 * "active = this chip" and pre-load its 3-byte response, on rising
 * edge we decode the accumulated MOSI bytes and stash any rdsel
 * change for the next response. While a tmc2660 CS is low the
 * spi_out_hook routes through the per-chip 3-byte path and bypasses
 * the global 5-byte accumulator (so its byte counter stays aligned
 * for the other TMC chips on the bus). */
#define TMC_CHIP_MAX 8
struct tmc_chip {
    char cs_port;             /* 'A'..'L' */
    int cs_pin;               /* 0..7 */
    int cs_active;            /* 1 while CS is low */
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
};
static struct ads1220_chip ads1220_chips[ADS1220_CHIP_MAX];
static int ads1220_chips_count = 0;
static int ads1220_active_chip = -1; /* index of chip whose CS is low */

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
static void bltouch_ctrl_hook(struct avr_irq_t *irq, uint32_t value, void *param);

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
static void trigger_pin_hook(struct avr_irq_t *irq, uint32_t value, void *param);

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
    uint64_t rx_last_cycle;      /* cycle of last completed byte (for idle gap) */

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

/* spi_tmc_chip <port> <pin> tmc2660: register a TMC2660 chip on the
 * shared TMC SPI bus. The bridge hooks the CS pin so MOSI bytes get
 * routed through a 3-byte (20-bit) datagram decoder while CS is low
 * for this chip - the default 5-byte path keeps serving the other
 * TMC variants on the same bus. Re-issuing for the same pin is a
 * no-op (we keep the first registration). */
static void
apply_spi_tmc_chip(struct control_ctx *ctx, int port_ord, int pin)
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
    }
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
    if (ctx->verbose)
        fprintf(stderr,
                "simavr_bridge: spi_tmc_chip %c%d tmc2660 (slot %d)\n",
                port, pin, slot);
}

/* CS-pin hook for a registered TMC2660 chip. CS is active-low: when
 * the firmware drops it the chip is selected, when it rises the
 * datagram is complete. The bridge tracks the active chip globally
 * so spi_out_hook can route MOSI bytes through the chip's per-
 * transaction buffer without disturbing the existing 5-byte tmc_mode
 * accumulator (which keeps serving 5-byte chips on the same bus). */
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
        /* CS just went low: start of a 3-byte datagram. Reset the
         * MOSI buffer and pre-load the response (READRSP@RDSEL<rdsel>
         * value).  When rdsel=2 (READRSP@RDSEL2 in tmc2660.py) klippy's
         * periodic _do_periodic_check uses the "se" field as the
         * cs_actual / current-scaler healthiness gate - if it reads
         * zero, klippy decodes it as "0(Reset?)" and shuts down once
         * motion starts.  klippy's tmc2660 init sets RDSEL=2 first
         * (see tmc2660.py "Must set RDSEL value first") so by the time
         * the periodic check runs we're already serving RDSEL2.  Pack
         * a non-zero se: the field sits at data bits 14..18 (after
         * klippy's data = pr[0]<<16 | pr[1]<<8 | pr[2] decode shifts
         * the 20-bit response up by 4), so 5 << 10 in response_value
         * lands as se=5 in the field-decoded value.  Other fields
         * (stallguard / ot / sg_result / etc.) stay zero - good enough
         * for pretty_format and for init not to flag anything. */
        c->in_pos = 0;
        c->out_pos = 0;
        memset(c->in_buf, 0, sizeof(c->in_buf));
        uint32_t response_value = (c->rdsel == 2) ? (5U << 10) : 0;
        uint32_t packed = response_value << 4;
        c->out_buf[0] = (uint8_t)((packed >> 16) & 0xff);
        c->out_buf[1] = (uint8_t)((packed >> 8) & 0xff);
        c->out_buf[2] = (uint8_t)(packed & 0xff);
        tmc_active_chip = idx;
    } else if (!now_active && was_active) {
        /* CS just went high: decode whatever we accumulated. The
         * 20-bit datagram occupies the low 20 bits of the 24-bit
         * stream (upper 4 bits are dummy/zero):
         *   bits 19..17 = 3-bit register address
         *   bit  16     = LSB of (val>>16) -- always 0 except for
         *                 the optional bit-16 fields klippy carries
         *                 in DRVCONF/SGCSCONF/CHOPCONF
         *   bits 15..0  = val[15..0]
         * We only act on DRVCONF (reg-id=7) writes here, latching
         * the rdsel field for the next response. Other writes are
         * accepted but not stored - the test never reads them back. */
        if (c->in_pos == 3) {
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
         * one period from now. */
        ads1220_drive_drdy(&ads1220_chips[slot], 1);
        avr_cycle_timer_register(ctx->avr,
                                 ads1220_chips[slot].period_cycles,
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
    if (spi_state.ads_mode) {
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
                /* RESET: zero the register file so the post-reset
                 * read returns the expected all-zero state. */
                memset(spi_state.ads_regs, 0,
                       sizeof(spi_state.ads_regs));
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
            resp = spi_state.ads_regs[spi_state.ads_reg]
                                     [spi_state.ads_byte_idx];
            spi_state.ads_byte_idx++;
            spi_state.ads_remaining--;
        } else {
            spi_state.ads_regs[spi_state.ads_reg]
                              [spi_state.ads_byte_idx] = mosi;
            spi_state.ads_byte_idx++;
            spi_state.ads_remaining--;
            resp = 0;
        }
    } else if (spi_state.tmc_mode && tmc_active_chip >= 0) {
        /* TMC2660 3-byte protocol: a registered chip's CS is currently
         * low. Route this MOSI byte through the per-chip buffer; the
         * pre-loaded out_buf was set on the CS falling edge. The
         * datagram is finalized in tmc_cs_hook on the rising edge -
         * we don't decode here because byte 3 doesn't always close a
         * transaction (e.g. spi_transfer pads, klippy guarantees
         * exactly 3 bytes per transaction so the assumption holds,
         * but doing it on CS keeps the contract explicit). */
        struct tmc_chip *c = &tmc_chips[tmc_active_chip];
        if (c->out_pos < 3)
            resp = c->out_buf[c->out_pos++];
        if (c->in_pos < 3)
            c->in_buf[c->in_pos++] = mosi;
    } else if (spi_state.tmc_mode) {
        /* TMC SPI: 5-byte datagrams. MISO byte_n is the response
         * we computed at the END of the previous datagram. */
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
            if (is_write)
                spi_state.tmc_regs[reg] = data;
            uint32_t out = spi_state.tmc_regs[reg];
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
        if (matched >= 0 && twi_state.reg_lens[matched] > 0) {
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
        if (got == 3 && strcmp(proto, "tmc2660") == 0) {
            apply_spi_tmc_chip(ctx, (int)(unsigned char)port, pin);
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
                    avr_cycle_count_t target = ctx->avr->cycle
                        + (avr_cycle_count_t)usec
                          * (ctx->avr->frequency / 1000000ULL);
                    while (g_running && ctx->avr->cycle < target) {
                        struct timespec ts = {0, 1000000};  /* 1 ms */
                        nanosleep(&ts, NULL);
                    }
                    const char *ok = "OK\n";
                    ssize_t w = write(cli, ok, 3);
                    (void)w;
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

/* Tick-mode lockstep (--tick-socket): klippy's reactor connects on
 * this AF_UNIX socket and drives simulated time by exchange. Klippy
 * sends "advance <T>\n", we run avr_run() until avr->cycle/frequency
 * reaches T, update the sim_time mmap, and reply "done <T_actual>\n".
 * Once klippy is connected, the wall-clock throttle is bypassed and
 * simavr advances exactly as far as klippy asks - eliminating the
 * host-load timing skew that makes parallel sim_time mode flaky.
 *
 * Pre-connection (before klippy starts) the main loop stays in
 * free-run with the wall-clock throttle so the test runner's
 * fixture-setup `barrier <usec>` over the control socket still
 * advances simavr enough to apply the queued IRQs. */
static int
tick_socket_listen(const char *path)
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

int
main(int argc, char *argv[])
{
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
    while ((opt = getopt_long(argc, argv, "e:l:c:m:d:vt:k:", longopts, NULL)) != -1) {
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

    /* uart_pty is simavr's prebuilt UART<->pty bridge. It opens a pty
     * pair, runs an internal pump thread that drains the master fd
     * into simavr's UART input IRQ (respecting XON/XOFF), and forwards
     * UART output to the master fd so the slave side reads it. We
     * avoid having to reimplement any of that. */
    static uart_pty_t pty;
    uart_pty_init(avr, &pty);
    uart_pty_connect(&pty, '0');

    if (verbose)
        fprintf(stderr,
            "simavr_bridge: pty slave %s mcu %s freq %u\n",
            pty.pty.slavename, mcu_name, avr->frequency);

    /* Make the slave node accessible to other processes in the
     * container (klippy generally runs as a different uid in real
     * deployments, but in tests both run as root - even so, openpty
     * leaves the slave at mode 0620 which is restrictive). */
    if (chmod(pty.pty.slavename, 0666) < 0) {
        fprintf(stderr, "simavr_bridge: chmod %s 0666: %s\n",
                pty.pty.slavename, strerror(errno));
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
        tick_listen_fd = tick_socket_listen(tick_socket_path);
        if (tick_listen_fd < 0)
            return 1;
        if (verbose)
            fprintf(stderr, "simavr_bridge: tick socket %s\n",
                    tick_socket_path);
    }

    if (write_slave_link(slave_link_path, pty.pty.slavename) < 0)
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

    /* Duration safety net: kills the simulator after N WALL seconds
     * so a hung firmware test doesn't run forever in CI. */
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
    int tick_client_fd = -1;
    char tick_buf[256];
    size_t tick_fill = 0;
    while (g_running && state != cpu_Done && state != cpu_Crashed) {
        /* Once klippy connects on the tick socket we leave free-run
         * mode and only advance simavr in response to "advance" lines.
         * Until then (during fixture-setup `barrier`s on the control
         * socket) we run free with the wall-clock throttle below. */
        if (tick_listen_fd >= 0 && tick_client_fd < 0) {
            int fd = accept(tick_listen_fd, NULL, NULL);
            if (fd >= 0) {
                tick_client_fd = fd;
                tick_fill = 0;
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
            avr_cycle_count_t target_cycle =
                (avr_cycle_count_t)(target * (double)avr->frequency);
            while (g_running && avr->cycle < target_cycle
                   && state != cpu_Done && state != cpu_Crashed) {
                state = avr_run(avr);
            }
            if (sim_time_ptr)
                *sim_time_ptr = (double)avr->cycle / (double)avr->frequency;
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
        uint64_t wall_ns = (uint64_t)(now.tv_sec - start_ts.tv_sec) * 1000000000ULL
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
