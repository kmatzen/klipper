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
 *
 * Lines that don't parse are silently ignored to avoid breaking the
 * simulator on a fixture typo.
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
};

static struct spi_response_state spi_state = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .len = 0,
    .pos = 0,
};

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

/* Forward decl - definition lives further down with the SPI hook. */
static void step_pin_hook(struct avr_irq_t *irq, uint32_t value, void *param);

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
         * Used by tests that need to simulate a "physical touch" -
         * e.g. dropping a bltouch sensor pin to trigger a probe. */
        char port = (char)a;
        if (port < 'A' || port > 'L')
            return;
        if (b < 0 || b > 7)
            return;
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

    if (ctx->verbose)
        fprintf(stderr,
            "simavr_bridge: step_trigger step=%c%d count=%u "
            "trig=%c%d val=%d\n",
            (char)step_port_ord, step_pin,
            (unsigned)count_threshold,
            (char)trig_port_ord, trig_pin, trig_val);
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

/* Hook on the firmware's SPI MOSI byte. Each time the firmware
 * writes SPDR, simavr fires this with the byte going OUT to the
 * (virtual) slave; we synchronously raise SPI_IRQ_INPUT with the
 * next byte from our response queue so the firmware reads back a
 * matching MISO byte on the same transfer. */
static void
spi_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)value;
    (void)param;
    if (!g_spi_in_irq)
        return;
    /* Default to 0x00 rather than 0xff for unscripted bytes. Klipper's
     * thermocouple drivers compute value=0 fault=0 from all-zero
     * responses, which passes the firmware's min/max range check
     * when min_temp=0 (the typical test config). 0xff would set
     * fault bits in MAX31855 (`value & 0x07`) and produce a huge
     * value out of range for the others, immediately tripping the
     * "Thermocouple reader fault" shutdown. */
    uint8_t resp = 0x00;
    pthread_mutex_lock(&spi_state.lock);
    if (spi_state.len > 0) {
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
    /* Address byte: firmware is starting a transaction. ACK to
     * indicate the slave is present at this address. */
    if (msg & TWI_COND_ADDR) {
        twi_state.current_addr = addr >> 1;  /* drop R/W bit */
        if (twi_state.verbose)
            fprintf(stderr, "simavr_bridge: twi addr=%02x %s\n",
                    twi_state.current_addr,
                    (msg & TWI_COND_READ) ? "read" : "write");
        avr_raise_irq(g_twi_in_irq,
                      avr_twi_irq_msg(TWI_COND_ACK, addr, 1));
        return;
    }
    /* Firmware writing a data byte: ACK it (we don't need the
     * content for any current test). */
    if (msg & TWI_COND_WRITE) {
        avr_raise_irq(g_twi_in_irq,
                      avr_twi_irq_msg(TWI_COND_ACK, addr, 1));
        return;
    }
    /* Firmware reading a byte: hand it the next queued response. */
    if (msg & TWI_COND_READ) {
        uint8_t resp = 0x00;
        pthread_mutex_lock(&twi_state.lock);
        if (twi_state.len > 0) {
            resp = twi_state.bytes[twi_state.pos];
            twi_state.pos = (twi_state.pos + 1) % twi_state.len;
        }
        pthread_mutex_unlock(&twi_state.lock);
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
                apply_control_line(ctx, start);
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

    static struct option longopts[] = {
        {"elf",            required_argument, NULL, 'e'},
        {"slave-link",     required_argument, NULL, 'l'},
        {"control-socket", required_argument, NULL, 'c'},
        {"mcu",            required_argument, NULL, 'm'},
        {"duration",       required_argument, NULL, 'd'},
        {"verbose",        no_argument,       NULL, 'v'},
        {NULL, 0, NULL, 0},
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "e:l:c:m:d:v", longopts, NULL)) != -1) {
        switch (opt) {
        case 'e': elf_path = optarg; break;
        case 'l': slave_link_path = optarg; break;
        case 'c': control_socket_path = optarg; break;
        case 'm': mcu_name = optarg; break;
        case 'd': duration_s = atof(optarg); break;
        case 'v': verbose = 1; break;
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
     * so a hung firmware test doesn't run forever in CI. Klippy
     * requires the MCU clock to advance at real-time rate (its
     * clock-sync regression assumes it), so we throttle the simulator
     * to wall-time below regardless of duration. Setting --duration=0
     * disables only the deadline, not the throttling. */
    struct timespec start_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);
    uint64_t deadline_wall_ns = duration_s > 0
        ? (uint64_t)(duration_s * 1e9)
        : 0;

    int state = cpu_Running;
    /* Throttling check is expensive (clock_gettime + arithmetic), so
     * only do it periodically. Every ~16k MCU cycles is roughly 1ms
     * of MCU time at 16MHz - tight enough to keep wall vs MCU clocks
     * within a millisecond, loose enough not to dominate the work. */
    uint64_t throttle_check_interval = avr->frequency / 1000;
    uint64_t next_throttle_cycle = throttle_check_interval;
    while (g_running && state != cpu_Done && state != cpu_Crashed) {
        state = avr_run(avr);
        if (avr->cycle < next_throttle_cycle)
            continue;
        next_throttle_cycle = avr->cycle + throttle_check_interval;
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
        /* Wall-clock the simulator: if MCU cycles are running ahead of
         * wall time, sleep until they line up. clock_freq cycles
         * should take exactly 1 wall second. */
        uint64_t expected_wall_ns = (avr->cycle * 1000000000ULL) / avr->frequency;
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
