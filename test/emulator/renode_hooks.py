# -*- coding: utf-8 -*-
# IronPython module loaded into Renode by renode_launcher.py via
# `include @<path>` in the rendered .resc. Defines the peripheral hook
# functions that fixture commands resolve to (step_trigger, bltouch,
# gpio_set, probe_step, ...). Each function is invoked by the launcher
# via `python "<func>(<args>)"` over the Renode TCP Monitor; persistent
# state (counters, hook handles) lives in module-level dicts so re-arm
# patterns (multi-sample probing, multi-axis homing) don't lose state
# across fixture pushes.
#
# Names mirror simavr_bridge.c control commands one-for-one. Same
# semantics: rising-edge step counts trigger endstop pin drives; the
# fixture pusher in scripts/test_klippy.py emits the same vocabulary
# regardless of backend.
#
# Hook registration uses C# Action[bool] callables built from Python
# closures, NOT the GPIOHookExtensions.AddStateChangedHook(string)
# wrapper. The string form spawns a fresh GPIOPythonEngine ScriptScope
# per hook (no module imports, no shared globals), which makes
# multi-step hook logic with shared state painful and forces
# multi-level escape quoting through the Monitor TCP. The base
# IGPIOWithHooks.AddStateChangedHook(Action<bool>) lets us pass any
# Python callable directly; closures naturally capture per-hook
# state.

from System import Action


# Module-level state. The Monitor's PythonEngine retains its script
# scope across `python` Monitor calls, so dicts assigned at module
# load time persist for the lifetime of the Renode process (which is
# one test invocation).
_step_hooks = {}    # (step_port, step_pin) -> Action[bool] handle
_bltouch_hooks = {}  # (ctrl_port, ctrl_pin) -> Action[bool] handle

# MonitorPythonEngine injects `monitor` ONLY into its own Python
# scope, not into modules imported FROM that scope. So when the
# launcher does `import renode_hooks` and then
# `renode_hooks.step_trigger(...)`, the function body runs in
# renode_hooks's namespace where `monitor` is undefined. The launcher
# fixes this by calling `renode_hooks.set_monitor(monitor)` once
# right after the import; every hook function then resolves the
# current machine via the saved reference.
_M = None


def set_monitor(m):
    global _M
    _M = m


# GPIO peripheral naming varies by chip family in Renode platforms:
#   STM32 family    -> sysbus.gpioPortA / gpioPortB / ...
#   Atmel SAM3/4/E7 -> sysbus.pioA / pioB / pioC / ...
#   SAMD21 / SAMD51 -> sysbus.gpio_a / gpio_b / ...
# Probe in that order, cache the form that works for each port letter
# so subsequent calls are O(1). On unresolved letters we fall through
# to a clear error rather than silently no-op'ing.
_GPIO_PORT_CACHE = {}
_GPIO_PORT_NAME_FORMS = (
    'gpioPort%s',     # STM32: gpioPortA
    'pio%s',          # SAM3/SAM4S/SAM4E/SAME70: pioA
    'gpio_%s',        # SAMD21/SAMD51: gpio_a (lowercase)
)


def _gpio_port(letter):
    upper = letter.upper()
    cached = _GPIO_PORT_CACHE.get(upper)
    if cached is not None:
        return cached
    sysbus = _M.Machine
    tried = []
    for fmt in _GPIO_PORT_NAME_FORMS:
        if '_%s' in fmt:
            name = fmt % upper.lower()
        else:
            name = fmt % upper
        path = 'sysbus.' + name
        tried.append(path)
        try:
            p = sysbus[path]
        except Exception:
            continue
        _GPIO_PORT_CACHE[upper] = p
        return p
    raise RuntimeError(
        "no gpio port found for letter %s (tried %s)"
        % (upper, ', '.join(tried)))


def _pin(port, idx):
    return _gpio_port(port).Connections[int(idx)]


# CPU peripheral name varies by platform .repl: most name it `cpu`
# (STM32 / SAM / LPC / HC32 / RP2040), but the SAMD repls name it `cpu0`
# (samd21g18.repl / samd51p20.repl, mirroring the upstream atsamd
# platforms). The TMC sw_uart and SPI responders attach CPU PC hooks via
# this object, so resolve either name; without this the hook install on
# SAMD silently raises at the lookup and no TMC datagram is ever served.
_CPU_CACHE = [None]


def _cpu():
    if _CPU_CACHE[0] is not None:
        return _CPU_CACHE[0]
    for name in ('sysbus.cpu', 'sysbus.cpu0'):
        try:
            c = _M.Machine[name]
        except Exception:
            continue
        _CPU_CACHE[0] = c
        return c
    raise RuntimeError("no cpu peripheral found (tried sysbus.cpu, "
                       "sysbus.cpu0)")


def gpio_set(port, pin, value):
    # Drive an input pin into the firmware's view. Renode's GPIOPort
    # is an IGPIOReceiver; OnGPIO(pin, bool) is the canonical input
    # injection path.
    _gpio_port(port).OnGPIO(int(pin), bool(int(value)))


def step_trigger(step_port, step_pin, count, trig_port, trig_pin,
                 trig_value):
    # On the `count`-th rising edge of <step_port:step_pin>, drive
    # <trig_port:trig_pin> to <trig_value>. Auto-rearms after firing
    # so multi-axis homing / multi-sample probing keeps working
    # without re-issuing the fixture command per attempt.
    sp = step_port.upper()
    spi = int(step_pin)
    tp = trig_port.upper()
    tpi = int(trig_pin)
    threshold = int(count)
    val = bool(int(trig_value))
    counter_box = [0]
    trig_port_obj = _gpio_port(tp)

    def handler(state):
        if not state:
            return  # rising edges only
        counter_box[0] += 1
        if counter_box[0] >= threshold:
            counter_box[0] = 0
            trig_port_obj.OnGPIO(tpi, val)

    pin_obj = _pin(sp, spi)
    key = (sp, spi)
    old = _step_hooks.get(key)
    if old is not None:
        try:
            pin_obj.RemoveStateChangedHook(old)
        except Exception:
            pass
    cb = Action[bool](handler)
    pin_obj.AddStateChangedHook(cb)
    _step_hooks[key] = cb


def probe_step(step_port, step_pin, reset_us, force_per_step,
               trig_port, trig_pin, trig_value):
    # probe_step is step_trigger with a per-step force decay (used by
    # load-cell-probe simulation). The renode bridge does not yet
    # model analog load-cell sensing, so we degrade to step_trigger
    # with the per-step force as the count threshold - close enough
    # for the multi-sample / multi-point probing test cases without
    # the analog physics.
    step_trigger(step_port, step_pin, force_per_step,
                 trig_port, trig_pin, trig_value)


def bltouch(ctrl_port, ctrl_pin, sensor_port, sensor_pin, invert):
    # BLTouch state machine: rising edge on ctrl pin DEPLOYS the
    # probe (drives sensor pin to "untriggered" = !invert), falling
    # edge ARMS it (sensor pin "triggered" = invert). Mirrors
    # simavr_bridge.c:312-396 minus the 50ms self-pulse - klippy's
    # poll loop tolerates the absence of the pulse on first cut.
    cp = ctrl_port.upper()
    cpi = int(ctrl_pin)
    sp = sensor_port.upper()
    spi = int(sensor_pin)
    untriggered = not bool(int(invert))
    triggered = bool(int(invert))
    sensor_port_obj = _gpio_port(sp)

    def handler(state):
        sensor_port_obj.OnGPIO(spi, untriggered if state else triggered)

    pin_obj = _pin(cp, cpi)
    key = (cp, cpi)
    old = _bltouch_hooks.get(key)
    if old is not None:
        try:
            pin_obj.RemoveStateChangedHook(old)
        except Exception:
            pass
    cb = Action[bool](handler)
    pin_obj.AddStateChangedHook(cb)
    _bltouch_hooks[key] = cb


# --------------------------------------------------------------------
# ADC defaults.
#
# empty_fixture.json's analog_in_default value is in the AVR oversampled
# raw-ADC domain (max 8184). Two STM32 ADC C# classes need different
# unit conversions:
#   - Analog.STM32_ADC (used by F1/F4/F7/L0 platforms): FeedSample
#     takes RAW 12-bit counts (max 4095). Conversion: raw_avr * 4095 // 8184.
#   - Analog.STM32_ADC_Common (base for F0/G0/H7-via-F0_ADC):
#     FeedVoltageSampleToChannel + SetDefaultValue take MILLIVOLTS
#     against a 3300 mV reference. Conversion: raw_avr * 3300 // 8184.
#
# adc_set probes for whichever API the loaded ADC peripheral supports
# and falls back if the first attempt errors.

_ADC_PERIPHS = ('sysbus.adc1', 'sysbus.adc2', 'sysbus.adc3',
                'sysbus.adc')

# AFEC peripheral bases for SAME70 (matches _AFEC_BASES_FOR_CHIP in
# renode_launcher.py). The afec_stub.py PythonPeripheral exposes
# magic offsets at 0x100 (default value) and 0x104..0x130 (per-channel
# overrides for 12 channels) - see afec_stub.py for the layout. Both
# AFEC0 and AFEC1 are listed; sysbus.WriteDoubleWord on a base that
# isn't mapped (e.g. SAM4S where there's no AFEC) raises, so the
# pokes below are wrapped in try/except.
_AFEC_BASES = (
    0x4003C000,  # SAME70 AFEC0
    0x40064000,  # SAME70 AFEC1
    0x400B0000,  # SAM4E AFEC0
    0x400B4000,  # SAM4E AFEC1
    0x40038000,  # SAM4S ADC (single peripheral, 16 channels)
    0x40012400,  # STM32F1/G0 ADC1 (stm32_adc_stub.py / stm32g0_adc_stub.py)
    0x40012000,  # STM32F4 ADC1 (stm32_adc_stub.py; magic offsets at 0x100+)
    0x40022000,  # STM32H7 ADC1 (stm32h7_adc_stub.py; magic offsets at 0x100+)
    0x4004C000,  # RP2040 ADC (rp2040_adc_stub.py; magic offsets at 0x100+)
    # NB: the LPC176x ADC (lpc176x_adc.cs at 0x40034000) is deliberately
    # NOT listed here. _afec_poke writes every base on every chip, and
    # 0x40034000 is the RP2040's host-link UART0 - poking it during
    # skr_pico would scribble on a live peripheral. The LPC ADC is a real
    # C# peripheral, so it takes fixture values through the idiomatic
    # FeedSample / SetDefaultValue methods via _iter_adcs() instead (it is
    # named `adc`, resolved as sysbus.adc only on the LPC platform).
)
_AFEC_MAGIC_DEFAULT = 0x100
_AFEC_MAGIC_CH_BASE = 0x104

# When the launcher knows the chip's actual ADC base(s) it calls
# set_adc_bases() so _afec_poke writes ONLY those, not every candidate in
# _AFEC_BASES. This matters where a base that is an ADC on one chip
# aliases a DIFFERENT live peripheral on another: e.g. 0x40022000 is the
# STM32H7 ADC1 but the STM32G0 flash controller, and 0x40034000 is the
# LPC ADC but the RP2040 UART0. Poking a foreign live peripheral (the
# write succeeds, so try/except does not catch it) can corrupt boot, so
# restrict to the current chip. Falls back to the full candidate list if
# the launcher never set it.
_adc_bases_override = [None]


def set_adc_bases(bases):
    try:
        _adc_bases_override[0] = [int(b) for b in bases]
        _log("set_adc_bases: %s",
             ' '.join('%x' % b for b in _adc_bases_override[0]))
    except Exception as e:
        _log("set_adc_bases: bad bases %r: %s", bases, e)


def _iter_adcs():
    for name in _ADC_PERIPHS:
        try:
            yield _M.Machine[name]
        except Exception:
            continue


def _afec_poke(offset, value):
    # Write through sysbus to whichever AFEC bases are mapped on the
    # current platform. SAM4S/STM32 don't have AFEC and the writes
    # silently fail; SAME70 has both AFEC0 and AFEC1 wired by the
    # launcher's afec_block.
    try:
        sysbus = _M.Machine.SystemBus
    except Exception:
        return
    bases = _adc_bases_override[0]
    if bases is None:
        bases = _AFEC_BASES
    for base in bases:
        try:
            sysbus.WriteDoubleWord(base + offset, int(value) & 0xFFF)
        except Exception:
            continue


def _feed_one(adc, channel, raw_avr):
    raw_stm32 = (int(raw_avr) * 4095) // 8184
    mv_stm32 = (int(raw_avr) * 3300) // 8184
    try:
        adc.FeedSample(raw_stm32, int(channel), 1)
        return True
    except Exception:
        pass
    try:
        adc.FeedVoltageSampleToChannel(int(channel), mv_stm32, 1)
        return True
    except Exception:
        pass
    try:
        adc.SetDefaultValue(mv_stm32, int(channel))
        return True
    except Exception:
        return False


def adc_set(channel, raw_value):
    for adc in _iter_adcs():
        _feed_one(adc, channel, raw_value)
    raw_12bit = (int(raw_value) * 4095) // 8184
    _afec_poke(_AFEC_MAGIC_CH_BASE + int(channel) * 4, raw_12bit)


def adc_default(raw_value):
    for adc in _iter_adcs():
        bulk_ok = False
        try:
            adc.SetDefaultValue((int(raw_value) * 3300) // 8184, None)
            bulk_ok = True
        except Exception:
            bulk_ok = False
        if bulk_ok:
            continue
        for ch in range(16):
            _feed_one(adc, ch, raw_value)
    raw_12bit = (int(raw_value) * 4095) // 8184
    _afec_poke(_AFEC_MAGIC_DEFAULT, raw_12bit)


# --------------------------------------------------------------------
# I2C register-response mocking.
#
# Mocks.DummyI2CSlave is the canonical I2C mock - exposes
# EnqueueResponseBytes, DataReceived (Action<byte[]>), ReadRequested
# (Action<int>). Klippy's I2C read pattern is write-register-then-
# read; we capture the register byte from DataReceived, enqueue the
# matching response from ReadRequested.
#
# IronPython on .NET 6 portable Renode auto-discovers loaded
# assemblies, so the import works without an explicit clr
# AddReference (which would fail because the assembly name varies
# between Mono and .NET 6 builds).

try:
    from Antmicro.Renode.Peripherals.Mocks import DummyI2CSlave
    _HAVE_DUMMY_I2C = True
except ImportError:
    _HAVE_DUMMY_I2C = False

from System import Array, Byte


def _byte_array(vals):
    # EnqueueResponseBytes takes IEnumerable<byte>. Handing it a plain
    # IronPython list of ints type-checks at call time but explodes
    # inside DummyI2CSlave.Read when the enumerator's Current is cast
    # ("Unable to cast object of type 'System.Int32' to type
    # 'System.Byte'") - which kills the whole Renode process from the
    # CPU thread. Build a typed byte[] so the enumeration is sound.
    return Array[Byte]([v & 0xff for v in vals])

_i2c_slaves = {}  # (bus_name, addr) -> (slave, last_reg_box, responses)


def _bus(bus_index_or_name):
    if isinstance(bus_index_or_name, str):
        return _M.Machine[bus_index_or_name]
    return _M.Machine['sysbus.i2c%d' % int(bus_index_or_name)]


def i2c_register_response(bus, addr, register_responses):
    if not _HAVE_DUMMY_I2C:
        return
    bus_obj = _bus(bus)
    addr_int = int(addr)
    bus_name = ('sysbus.i2c%d' % int(bus)
                if not isinstance(bus, str) else bus)
    key = (bus_name, addr_int)
    norm_resp = {}
    for k, v in register_responses.items():
        ki = int(k, 0) if isinstance(k, str) else int(k)
        norm_resp[ki] = list(v)
    if key in _i2c_slaves:
        _i2c_slaves[key][2].clear()
        _i2c_slaves[key][2].update(norm_resp)
        return
    slave = DummyI2CSlave()
    last_reg_box = [None]
    responses = dict(norm_resp)

    def on_data_received(data):
        if data and len(data) >= 1:
            last_reg_box[0] = int(data[0]) & 0xff

    def on_read_requested(count):
        reg = last_reg_box[0]
        payload = responses.get(reg)
        if payload is None:
            return  # leave queue empty -> DummyI2CSlave returns zeros
        slave.EnqueueResponseBytes(_byte_array(payload[:int(count)]))

    slave.DataReceived += on_data_received
    slave.ReadRequested += on_read_requested
    bus_obj.Register(slave, addr_int)
    _i2c_slaves[key] = (slave, last_reg_box, responses)


# --------------------------------------------------------------------
# LDC1612 eddy-probe ramp (probe_eddy_current) - the renode counterpart
# of simavr_bridge.c's ldc1612_ramp, invoked through the launcher's
# _xlat_passthrough with the same positional command vocabulary:
#
#   ldc1612_ramp <step_port> <step_pin> <dir_port> <dir_pin>
#                <descend_level> <baseline_raw> <free_per_step>
#                <sample_rate> [<contact_descent> <depress_per_step>]
#
# A register-aware I2C slave (LDC1612 default address 0x2a on i2c1)
# serves the driver's startup probe (manufacturer/device IDs), absorbs
# its register writes into a register file, and answers DATA0 with a
# 28-bit count that is a pure function of the Z stepper's absolute
# position: raw(N) = baseline_raw + free_per_step * min(N,
# contact_descent) + depress_per_step * max(0, N - contact_descent),
# floored at _LDC1612_FLOOR_RAW. N (net_descent) counts rising step
# edges signed by the DIR pin level (== descend_level -> +1), NEVER
# reset, mirroring the bridge's absolute-position model.
#
# Sampling is paced by STATUS gating on the virtual clock, matching the
# bridge's uniform-period model: UNREADCONV0 reports ready only once
# per 1/sample_rate of virtual time, and the sample latched then is
# interpolated to the exact period boundary using the last two step
# timestamps (fraction (t_boundary - t_last_step) / step interval,
# reset on direction reversal). Without the gating the firmware's
# rest_ticks poll grid (0.5 / data_rate) would double the sample rate
# and trip klippy's sample-timing validator; without the interpolation
# the +/-1-step quantization of the poll-grid/step-grid beat adds
# derivative noise on the scale of free_per_step per sample, which the
# tap detector's diff_peak filter would see as signal. The cfg omits
# intb_pin, so the firmware timer polls unconditionally
# (sensor_ldc1612.c ldc1612_event) and STATUS is the only pacing.

_LDC1612_FLOOR_RAW = 0x1CEA000   # keep the settled/idle count above the
                                 # driver's amplitude-error region but
                                 # below every trigger threshold the
                                 # eddy tests arm (see simavr_bridge.c)
_LDC1612_ADDR = 0x2a
_ldc1612_ramp_state = {}         # singleton keyed 'ramp'


def _now_us():
    # Probed against Renode 1.16.1 (same API the launcher's tick
    # driver uses): LocalTimeSource.ElapsedVirtualTime advances only
    # inside RunFor windows, so under tick mode this is deterministic.
    return _M.Machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds \
        * 1000000.0


def ldc1612_ramp(step_port, step_pin, dir_port, dir_pin, descend_level,
                 baseline_raw, free_per_step, sample_rate,
                 contact_descent='0', depress_per_step='0'):
    if not _HAVE_DUMMY_I2C:
        _log("ldc1612_ramp: DummyI2CSlave unavailable, ramp disabled")
        return
    sp, spi = step_port.upper(), int(step_pin)
    dp, dpi = dir_port.upper(), int(dir_pin)
    st = {
        'descend_level': int(descend_level),
        'baseline': int(baseline_raw),
        'free': int(free_per_step),
        'period_us': 1000000.0 / max(1, int(sample_rate)),
        'contact': int(contact_descent),
        'depress': int(depress_per_step),
        'net': 0,               # signed absolute step position
        'dir': 0,               # last observed DIR level
        'sign': 1,              # +1 descending, -1 retracting
        'last_step_us': None,   # newest step edge timestamp
        'prev_step_us': None,   # the one before (same direction)
        'next_period_us': None,
        'pending': False,
        'latched': 0,
        'reg_ptr': 0,
        'regfile': {0x7e: 0x5449, 0x7f: 0x3055},
        'steps': 0,             # lifetime rising edges (diagnostics)
    }
    _ldc1612_ramp_state['ramp'] = st

    def on_dir(state):
        st['dir'] = 1 if state else 0

    def on_step(state):
        if not state:
            return  # rising edges only
        sign = 1 if st['dir'] == st['descend_level'] else -1
        if sign != st['sign']:
            # Direction reversal: the previous interval no longer
            # predicts the next step, so restart interpolation (both
            # timestamps - the old direction's last edge must not seed
            # the new direction's interval).
            st['prev_step_us'] = None
            st['last_step_us'] = None
            st['sign'] = sign
        st['net'] += sign
        st['steps'] += 1
        st['prev_step_us'], st['last_step_us'] = \
            st['last_step_us'], _now_us()

    def pos_at(t_us):
        # Continuous stepper position at t_us: integer count plus a
        # sub-step fraction linearly inter/extrapolated from the last
        # step interval (t may fall slightly before the newest step
        # when the period boundary predates it - the negative fraction
        # walks the position back, matching the bridge's model).
        last, prev = st['last_step_us'], st['prev_step_us']
        if last is None or prev is None or last <= prev:
            return float(st['net'])
        frac = (t_us - last) / (last - prev)
        return st['net'] + st['sign'] * frac

    def raw_at(pos):
        c, free, dep = st['contact'], st['free'], st['depress']
        if c > 0 and pos > c:
            v = st['baseline'] + free * c + dep * (pos - c)
        else:
            v = st['baseline'] + free * pos
        v = int(v + 0.5)
        if v < _LDC1612_FLOOR_RAW:
            v = _LDC1612_FLOOR_RAW
        if v > 0x0FFFFFFF:
            v = 0x0FFFFFFF
        return v

    def on_data_received(data):
        if data is None or len(data) == 0:
            return
        reg = int(data[0]) & 0xff
        st['reg_ptr'] = reg
        if len(data) >= 3:
            st['regfile'][reg] = ((int(data[1]) & 0xff) << 8) \
                | (int(data[2]) & 0xff)

    def on_read_requested(count):
        reg = st['reg_ptr']
        if reg == 0x18:            # STATUS
            now = _now_us()
            if st['next_period_us'] is None:
                st['next_period_us'] = now + st['period_us']
            if not st['pending'] and now >= st['next_period_us']:
                st['latched'] = raw_at(pos_at(st['next_period_us']))
                st['pending'] = True
                last = st.get('last_latch_us')
                if last is not None:
                    gap = st['next_period_us'] - last
                    if gap < 0.8 * st['period_us'] \
                            or gap > 1.2 * st['period_us']:
                        _log("ldc1612_ramp: latch gap %.1fus (period "
                             "%.1fus) at now=%.1f next=%.1f"
                             % (gap, st['period_us'], now,
                                st['next_period_us']))
                st['last_latch_us'] = st['next_period_us']
                st['latch_count'] = st.get('latch_count', 0) + 1
                if st['latch_count'] % 4000 == 0:
                    _log("ldc1612_ramp: %d latches, net=%d now=%.1f"
                         % (st['latch_count'], st['net'], now))
                while st['next_period_us'] <= now:
                    st['next_period_us'] += st['period_us']
            val = 0x0008 if st['pending'] else 0x0000
        elif reg == 0x00:          # DATA0_MSB (consumes the sample)
            st['pending'] = False
            val = (st['latched'] >> 16) & 0x0FFF
        elif reg == 0x01:          # DATA0_LSB
            val = st['latched'] & 0xFFFF
        else:
            val = st['regfile'].get(reg, 0) & 0xFFFF
        slave.EnqueueResponseBytes(_byte_array(
            [(val >> 8) & 0xff, val & 0xff][:int(count)]))

    slave = DummyI2CSlave()
    slave.DataReceived += on_data_received
    slave.ReadRequested += on_read_requested
    _bus(1).Register(slave, _LDC1612_ADDR)
    _pin(sp, spi).AddStateChangedHook(Action[bool](on_step))
    _pin(dp, dpi).AddStateChangedHook(Action[bool](on_dir))
    _log("ldc1612_ramp: armed on %s%d/%s%d descend_level=%d baseline=%d "
         "free=%d period_us=%.1f contact=%d depress=%d"
         % (sp, spi, dp, dpi, st['descend_level'], st['baseline'],
            st['free'], st['period_us'], st['contact'], st['depress']))


def ldc1612_ramp_status():
    # Diagnostic: dump the ramp state (invoked manually or from a
    # fixture debug command while chasing geometry mismatches).
    st = _ldc1612_ramp_state.get('ramp')
    if st is None:
        _log("ldc1612_ramp_status: not armed")
        return
    _log("ldc1612_ramp_status: net=%d steps=%d dir=%d sign=%d "
         "latched=%d pending=%s"
         % (st['net'], st['steps'], st['dir'], st['sign'],
            st['latched'], st['pending']))


# --------------------------------------------------------------------
# TMC2208/2209 software UART support.
#
# Goal: when klippy sends a TMC UART read request, drive a synthetic
# response onto the firmware's RX pin so klippy reads back what looks
# like a real chip response. Writes update an in-memory register file
# so subsequent reads return the just-written value, IFCNT advances,
# GSTAT clears, etc - matching the simavr bridge's sw_uart behaviour.
#
# Mechanism: 4 CPU PC hooks on the firmware's tmcuart_* event functions.
# Each hook fires BEFORE the function body runs, so a gpio drive done
# in the hook is the value the function's gpio_in_read sees.
#
#   command_tmcuart_send       - decode the request, build the response
#                                bit stream, mark the uart "active"
#   tmcuart_send_finish_event  - drive RX HIGH (idle for sync)
#   tmcuart_read_sync_event    - 1st fire: leave HIGH (firmware will
#                                set TU_READ_SYNC); 2nd fire: drive LOW
#                                start bit, transition to bit-driving
#   tmcuart_read_event         - drive next response bit on each fire
#
# Symbol addresses come from the launcher via apply_sw_uart_symbols(),
# called once after ELF load. Configs that don't link tmcuart.o report
# no addresses; the hook installer no-ops.
#
# This file is the ONLY trace channel: every hook fire, state transition,
# register access, and error logs to stderr via _log(). The launcher
# wires Renode's stdout/stderr to its own stderr, which the test runner
# captures, so [sw_uart] lines surface in the test log.

try:
    from Antmicro.Renode.Peripherals.CPU import CpuAddressHook
    _HAVE_CPU_HOOK = True
except ImportError:
    _HAVE_CPU_HOOK = False

try:
    # Used by the TMC SPI chain responder to set PC=LR (skip the real
    # hardware transfer). SetRegisterUnsafe wants a RegisterValue.
    from Antmicro.Renode.Peripherals.CPU import RegisterValue
except ImportError:
    RegisterValue = None


_sw_uart_states = {}      # (rx_port, rx_pin, tx_port, tx_pin) -> _SwUart
_sw_uart_active = [None]  # uart currently driving a response (or None)
_sw_uart_symbols = {}     # symbol name -> firmware addr
_sw_uart_cpu_hooks_installed = [False]


# Dual-route logger. Hooks fire from C# event-dispatch context where
# sys.stderr does not always flow back to the renode subprocess stderr
# (Monitor-driven `python "..."` calls do flow because Renode wraps
# stdout/stderr around the python call). To be sure traces survive
# regardless of context, also append to a flat file the test harness
# can read after the run via a tempdir mount.
_LOG_PATH = '/tmp/sw_uart_trace.log'


def _log(msg, *args):
    import sys
    if args:
        msg = msg % args
    line = "[sw_uart] " + msg
    try:
        f = open(_LOG_PATH, 'a')
        try:
            f.write(line + "\n")
        finally:
            f.close()
    except Exception:
        pass
    try:
        sys.stderr.write(line + "\n")
        sys.stderr.flush()
    except Exception:
        pass


def _crc8_atm(data):
    crc = 0
    for b in data:
        b = int(b) & 0xff
        for _ in range(8):
            if ((crc >> 7) ^ (b & 0x01)) & 0x01:
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
            b >>= 1
    return crc


def _strip_serial_bits(serial_bytes):
    # Reverse klippy's _add_serial_bits packing: each data byte was
    # framed as 10 bits (start=0, 8 data LSB-first, stop=1) and the
    # bitstream was repacked into bytes LSB-first. Recover the
    # underlying data bytes; return None on framing error.
    if not serial_bytes:
        return None
    bits = []
    for b in serial_bytes:
        for i in range(8):
            bits.append((int(b) >> i) & 1)
    out = []
    pos = 0
    while pos + 10 <= len(bits):
        if bits[pos] != 0 or bits[pos + 9] != 1:
            return None
        v = 0
        for i in range(8):
            v |= bits[pos + 1 + i] << i
        out.append(v)
        pos += 10
    return out


class _SwUart(object):
    def __init__(self, rx_port, rx_pin, tx_port, tx_pin, bit_time, addr):
        self.rx_port = rx_port.upper()
        self.rx_pin = int(rx_pin)
        self.tx_port = tx_port.upper()
        self.tx_pin = int(tx_pin)
        self.bit_time = int(bit_time)
        self.addr = int(addr) & 0xff
        # TMC2208 register file with reset defaults klippy probes
        # during connect / DUMP_TMC. Same defaults as the simavr
        # bridge's apply_sw_uart in simavr_bridge.c.
        self.regs = [0] * 128
        self.regs[0x00] = 0x00000040    # GCONF: pdn_disable=1
        self.regs[0x01] = 0x00000001    # GSTAT: reset=1
        self.regs[0x06] = 0x21000040    # IOIN: version=0x21
        self.regs[0x6f] = 0xc0000000    # DRV_STATUS: stst=1
        # Response state.
        self.response_bits = []
        self.response_idx = 0
        # IDLE -> SYNC_HIGH -> SYNC_LOW -> DRIVING -> IDLE
        self.phase = 'IDLE'


# RP2040 SIO GPIO_IN register. The local rp2040.repl models the
# single-cycle-IO region as plain storeback memory (not an
# IGPIOReceiver), so klipper's gpio_in_read (src/rp2040/gpio.c) just
# reads back sio_hw->gpio_in at SIO base 0xD0000000 + 0x004. To drive a
# firmware-side input bit we read-modify-write that word rather than
# calling OnGPIO on a GPIO peripheral that doesn't exist.
_RP2040_SIO_GPIO_IN = 0xD0000004

# LPC176x fast GPIO (LPC_GPIO_BASE). Five 0x20-spaced port blocks; the
# input register FIOPIN sits at +0x14. renode_launcher maps a plain
# storeback region here (_EXTRA_PERIPHERAL_STUBS_FOR_CHIP['lpc176x']),
# so - exactly like the RP2040 SIO region - we drive a firmware-side
# input bit by read-modify-writing FIOPIN, which klipper's gpio_in_read
# (src/lpc176x/gpio.c: FIOPIN & bit) reads back. The 'LPCn' rx_port
# sentinel (emitted by test_klippy._sw_uart_port_pin for "Pn.m" pin
# names) selects this path and carries the port number.
_LPC_GPIO_BASE = 0x2009C000
_LPC_GPIO_PORT_STRIDE = 0x20
_LPC_GPIO_FIOPIN = 0x14


def _drive_rx_one(s, value):
    # Drive a single firmware-side RX pin for sw_uart state `s`.
    if s.rx_port == 'RP':
        sb = _M.Machine.SystemBus
        cur = int(sb.ReadDoubleWord(_RP2040_SIO_GPIO_IN))
        if value:
            cur |= (1 << s.rx_pin)
        else:
            cur &= ~(1 << s.rx_pin)
        sb.WriteDoubleWord(_RP2040_SIO_GPIO_IN, cur & 0xFFFFFFFF)
        return
    if s.rx_port.startswith('LPC'):
        port = int(s.rx_port[3:])
        addr = (_LPC_GPIO_BASE + port * _LPC_GPIO_PORT_STRIDE
                + _LPC_GPIO_FIOPIN)
        sb = _M.Machine.SystemBus
        cur = int(sb.ReadDoubleWord(addr))
        if value:
            cur |= (1 << s.rx_pin)
        else:
            cur &= ~(1 << s.rx_pin)
        sb.WriteDoubleWord(addr, cur & 0xFFFFFFFF)
        return
    _gpio_port(s.rx_port).OnGPIO(s.rx_pin, bool(value))


def _drive_rx(s, value):
    # Drive the firmware-side RX bit for the active TMC UART response.
    #
    # Multi-drop shared-bus configs (skr_pico / skr_mini_e3_v2: four
    # TMC2209s on one rx/tx pair, distinguished by datagram address)
    # register a SINGLE sw_uart state, so this drives just that pin.
    #
    # Single-wire-per-driver configs (BTT SKR v1.4: four TMC2208s, each
    # with its own dedicated uart_pin, all datagram-address 0) register
    # one state PER pin. The responder can't tell from the bit-banged
    # request which oid/pin it targets (TMC2208 has no address field), so
    # broadcast the same framed reply onto every registered RX pin: klippy
    # initialises and reads the drivers strictly sequentially, the shared
    # register file (see _do_tmcuart_send target selection) makes the
    # reply content correct for whichever driver is live, and the other
    # pins simply aren't being sampled at that instant.
    if _sw_uart_states:
        for t in _sw_uart_states.values():
            _drive_rx_one(t, value)
    else:
        _drive_rx_one(s, value)


def _build_response_bits(s, reg):
    # 8-byte TMC response: [sync=0x05, master=0xff, reg, d3..d0, crc].
    # 80 bits total: per byte = 1 start(0) + 8 data LSB-first + 1 stop(1).
    val = s.regs[reg & 0x7f] & 0xffffffff
    resp = [0x05, 0xff, reg & 0x7f,
            (val >> 24) & 0xff, (val >> 16) & 0xff,
            (val >> 8) & 0xff, val & 0xff]
    resp.append(_crc8_atm(resp))
    bits = []
    for b in resp:
        bits.append(0)
        for i in range(8):
            bits.append((b >> i) & 1)
        bits.append(1)
    s.response_bits = bits
    s.response_idx = 0
    return resp


def _on_tmcuart_send_hook(cpu, pc):
    try:
        _do_tmcuart_send(cpu, pc)
    except Exception as e:
        _log("send_hook EXCEPTION: %s", e)


def _do_tmcuart_send(cpu, pc):
    # Firmware just entered command_tmcuart_send(args). R0 = uint32_t*
    # args. The argument layout for `tmcuart_send oid=%c write=%*s
    # read=%c` is:
    #   args[0] = oid
    #   args[1] = write_len (the %*s prefix length)
    #   args[2] = write_ptr (firmware-RAM addr of message bytes)
    #   args[3] = read_count
    # New request -> drop any stale active state. The previous
    # response's event_hook clears active when it drives the last bit,
    # but if a request gets cancelled mid-flight (e.g. firmware shutdown
    # during a response) active may still be set; clearing here makes
    # the per-request state machine idempotent.
    _sw_uart_active[0] = None
    try:
        args_ptr = int(cpu.GetRegisterUnsafe(0).RawValue)
    except Exception as e:
        _log("send: cannot read R0: %s", e)
        return
    sb = _M.Machine.SystemBus
    try:
        write_len = sb.ReadDoubleWord(args_ptr + 4)
        write_ptr = sb.ReadDoubleWord(args_ptr + 8)
        read_count = sb.ReadDoubleWord(args_ptr + 12)
    except Exception as e:
        _log("send: cannot read args at %x: %s", args_ptr, e)
        return
    if write_len < 1 or write_len > 32:
        _log("send: bad write_len=%d, ignoring", write_len)
        return
    msg = []
    for i in range(int(write_len)):
        try:
            msg.append(int(sb.ReadByte(int(write_ptr) + i)) & 0xff)
        except Exception as e:
            _log("send: msg read err at offset %d: %s", i, e)
            return
    decoded = _strip_serial_bits(msg)
    if decoded is None:
        _log("send: serial-bit strip returned None (framing error)")
        return
    if len(decoded) < 4 or decoded[0] != 0xf5:
        _log("send: bad header decoded=%s",
             ' '.join('%02x' % b for b in decoded))
        return
    addr = decoded[1] & 0xff
    reg_field = decoded[2] & 0xff
    is_write = (reg_field & 0x80) != 0
    expected = 8 if is_write else 4
    if len(decoded) < expected:
        _log("send: truncated decoded len=%d < %d", len(decoded), expected)
        return
    crc = _crc8_atm(decoded[:expected - 1])
    if crc != (decoded[expected - 1] & 0xff):
        _log("send: crc mismatch got=%02x want=%02x",
             decoded[expected - 1], crc)
        return
    reg = reg_field & 0x7f
    # Pick the uart state matching this request's address. Multi-chip
    # configs (duet2 maestro: 4 TMC2208s on shared PA9/PA10 with
    # select-pin mux) use addr=0 for all chips, so we fall back to
    # the single registered state when there's only one.
    target = None
    for s in _sw_uart_states.values():
        if s.addr == addr or len(_sw_uart_states) == 1:
            target = s
            break
    if target is None:
        _log("send: no uart state matches addr=%d", addr)
        return
    if is_write:
        v = ((decoded[3] & 0xff) << 24) | ((decoded[4] & 0xff) << 16) \
            | ((decoded[5] & 0xff) << 8) | (decoded[6] & 0xff)
        if reg == 0x01:
            # GSTAT: write-1-to-clear
            target.regs[reg] = target.regs[reg] & ~v
        else:
            target.regs[reg] = v
        if reg != 0x02:
            # IFCNT auto-increments on every register write
            target.regs[0x02] = (target.regs[0x02] + 1) & 0xff
        return
    # Read: build response, mark active, wait for finish_event hook.
    _build_response_bits(target, reg)
    target.phase = 'IDLE'
    _sw_uart_active[0] = target


def _on_send_finish_hook(cpu, pc):
    s = _sw_uart_active[0]
    if s is None:
        return
    try:
        _drive_rx(s, 1)
        s.phase = 'SYNC_HIGH'
    except Exception as e:
        _log("finish_hook EXCEPTION: %s", e)


def _on_read_sync_hook(cpu, pc):
    s = _sw_uart_active[0]
    if s is None:
        return
    try:
        if s.phase == 'SYNC_HIGH':
            s.phase = 'SYNC_LOW'
        elif s.phase == 'SYNC_LOW':
            _drive_rx(s, 0)
            s.phase = 'DRIVING'
            s.response_idx = 0
    except Exception as e:
        _log("sync_hook EXCEPTION: %s", e)


def _on_read_event_hook(cpu, pc):
    s = _sw_uart_active[0]
    if s is None or s.phase != 'DRIVING':
        return
    try:
        if s.response_idx >= len(s.response_bits):
            # Sentinel; we should normally clean up on the last bit
            # drive below.
            _drive_rx(s, 1)
            s.phase = 'IDLE'
            _sw_uart_active[0] = None
            return
        bit = s.response_bits[s.response_idx]
        _drive_rx(s, bit)
        s.response_idx += 1
        if s.response_idx >= len(s.response_bits):
            # Last bit driven. Firmware reads it and finalizes - no
            # more event_hook fires for this request, so clean up
            # active state inline. Otherwise it leaks to the next
            # request's finish_hook and triggers a spurious sync drive.
            s.phase = 'IDLE'
            _sw_uart_active[0] = None
    except Exception as e:
        _log("event_hook EXCEPTION: %s", e)


def _ensure_cpu_hooks_installed():
    if _sw_uart_cpu_hooks_installed[0]:
        return
    if not _HAVE_CPU_HOOK:
        _log("CpuAddressHook unavailable; cannot install hooks")
        return
    needed = ('command_tmcuart_send', 'tmcuart_send_finish_event',
              'tmcuart_read_sync_event', 'tmcuart_read_event')
    missing = [n for n in needed if n not in _sw_uart_symbols]
    if missing:
        _log("missing symbols: %s", ', '.join(missing))
        return
    cpu = _cpu()
    install = (
        ('command_tmcuart_send', _on_tmcuart_send_hook),
        ('tmcuart_send_finish_event', _on_send_finish_hook),
        ('tmcuart_read_sync_event', _on_read_sync_hook),
        ('tmcuart_read_event', _on_read_event_hook),
    )
    for name, cb in install:
        addr = _sw_uart_symbols[name]
        try:
            cpu.AddHook(int(addr), CpuAddressHook(cb))
            _log("hook installed: %s @ %x", name, int(addr))
        except Exception as e:
            _log("hook install FAILED for %s @ %x: %s", name, int(addr), e)
    _sw_uart_cpu_hooks_installed[0] = True


def apply_sw_uart_symbols(symbols):
    if not isinstance(symbols, dict):
        _log("apply_sw_uart_symbols: not a dict (got %r)",
             type(symbols).__name__)
        return
    for k, v in symbols.items():
        try:
            _sw_uart_symbols[str(k)] = int(v)
            _log("symbol %s -> %x", k, int(v))
        except (TypeError, ValueError) as e:
            _log("symbol %s rejected (%s): %r", k, e, v)


# --------------------------------------------------------------------
# Firmware-shutdown diagnostics. SchedStatus is a 12-byte struct (on
# 32-bit Cortex-M): {timer_list[4], last_insert[4], tasks_status[1],
# tasks_busy[1], shutdown_status[1], shutdown_reason[1]}. The byte at
# offset 11 is the static_string_id corresponding to the shutdown
# reason; klippy's dict maps these IDs to human-readable strings
# (enumerations.static_string_id). When the firmware shuts down before
# klippy has loaded the dict, the shutdown response message lands as
# "Unknown message -17 ... while identifying" - the reason is opaque
# until we peek the byte directly.

_sched_status_addr = [None]
_last_shutdown_reason = [0]


def apply_sched_status_addr(addr):
    try:
        _sched_status_addr[0] = int(addr)
        _log("SchedStatus @ %x", int(addr))
    except (TypeError, ValueError) as e:
        _log("apply_sched_status_addr: bad addr %r: %s", addr, e)


def peek_shutdown_reason():
    # Returns the firmware's shutdown_reason byte, or None if not
    # configured. Called from the launcher's tick loop after each
    # RunFor; if non-zero AND changed since last call, log it.
    addr = _sched_status_addr[0]
    if addr is None:
        return None
    try:
        sb = _M.Machine.SystemBus
        status = int(sb.ReadByte(addr + 10)) & 0xff
        reason = int(sb.ReadByte(addr + 11)) & 0xff
    except Exception as e:
        _log("peek_shutdown_reason: ReadByte err %s", e)
        return None
    if status:
        # shutdown_status non-zero == firmware genuinely shut down;
        # surface even if shutdown_reason byte read came back 0.
        return reason if reason else 0xff
    if reason != _last_shutdown_reason[0]:
        _last_shutdown_reason[0] = reason
        if reason:
            _log("FIRMWARE SHUTDOWN reason=%d (static_string_id; see dict"
                 " enumerations.static_string_id for human-readable)",
                 reason)
    return reason


def sw_uart(rx_port, rx_pin, tx_port, tx_pin, bit_time, addr):
    _log("register rx=%s%d tx=%s%d bt=%d addr=%d",
         rx_port, int(rx_pin), tx_port, int(tx_pin),
         int(bit_time), int(addr))
    key = (rx_port.upper(), int(rx_pin), tx_port.upper(), int(tx_pin))
    if key not in _sw_uart_states:
        _sw_uart_states[key] = _SwUart(rx_port, rx_pin, tx_port, tx_pin,
                                       bit_time, addr)
    _ensure_cpu_hooks_installed()


# --------------------------------------------------------------------
# TMC SPI daisy-chain responder (TMC2130 / TMC5160 / TMC2240).
#
# Renode models no USART-in-SPI-mode peripheral and no SPI device for
# the firmware to exchange bytes with, so a real spidev_transfer reads
# back garbage. Instead of modelling the peripheral we intercept the
# firmware's `spidev_transfer(spi, receive_data, len, data)` with a CPU
# PC hook (same mechanism as sw_uart): read the MOSI bytes out of the
# `data` buffer, synthesise the chain's MISO response in place, then
# return early (set PC=LR) so the real hardware exchange - which would
# overwrite our bytes with garbage - never runs. command_spi_transfer
# then `sendf`s our response back to klippy.
#
# Chain semantics (klippy MCU_TMC_SPI_chain, extras/tmc2130.py): a
# transaction is chain_len*5 bytes = n_slots 5-byte datagrams. klippy
# places a driver's command at slot (chain_len - chain_pos) and reads
# its response from the same slot, but the TMC reply carries the data
# addressed by the PREVIOUS datagram for that position (reg_read sends
# the read command twice; reg_write sends write_cmd then a dummy_read).
# So the correct model is a ONE-TRANSACTION delay per slot: response[s]
# = regfile[reg addressed at slot s in the previous transaction]. Both
# spi_send (command_spi_send) and spi_transfer (command_spi_transfer)
# route through spidev_transfer, so the single hook sees every datagram
# in order. The register file is shared across the chain - klippy
# initialises drivers sequentially, so one file suffices (matches the
# simavr bridge's tmc_regs).
_spi_tmc_symbol = [None]            # firmware addr of spidev_transfer
_spi_tmc_enabled = [False]
_spi_tmc_hook_installed = [False]
_spi_tmc_regs = {}                  # reg (0..0x7f) -> 32-bit value
_spi_tmc_prev_addr = {}            # slot index -> reg addressed last txn
_spi_tmc_dbg = [0]                  # limit per-call trace volume

# TMC2660 sub-mode. Unlike tmc2130/5160/2240 (5-byte/40-bit datagrams),
# the TMC2660 uses 3-byte/20-bit datagrams with NO read/write bit and NO
# chain-position addressing - every transaction returns the chip's
# READRSP register selected by the RDSEL field of the most recent DRVCONF
# write. The renode spidev_transfer hook sees the whole 3-byte datagram
# in one call, so unlike the simavr bridge it needs no CS-pin tracking: a
# single shared rdsel suffices because klippy initialises drivers
# sequentially. Enabled by the fixture's `spi_tmc_chip <port> <pin>
# tmc2660` (auto-emitted for every [tmc2660 ...] section); the 5-byte
# path keeps serving any 40-bit chips on the same bus.
_spi_tmc2660_enabled = [False]
_spi_tmc2660_rdsel = [0]


def _spi_tmc_reset():
    _spi_tmc_regs.clear()
    _spi_tmc_prev_addr.clear()
    # DRV_STATUS default for tmc2130 / tmc5160 / tmc2240: stst=1 +
    # cs_actual=5 (any non-zero scaler klippy decodes as healthy), so
    # the periodic DRV_STATUS check doesn't read cs_actual=0 and shut
    # down. Matches simavr_bridge.c's spi_tmc default.
    _spi_tmc_regs[0x6f] = 0xc0050000
    # IOIN VERSION (bits 24..31) = 0x30 (TMC5160 / TMC5160A); DUMP_TMC's
    # IOIN read then reports version=0x30, a positive proof the chain
    # read path round-tripped. Other registers default to 0.
    _spi_tmc_regs[0x04] = 0x30000000


def spi_tmc():
    _log("spi_tmc: enable TMC SPI chain responder")
    _spi_tmc_enabled[0] = True
    _spi_tmc_reset()
    _ensure_spi_tmc_hook_installed()


def spi_tmc_chip(cs_port, cs_pin, kind):
    # Fixture: `spi_tmc_chip <cs_port> <cs_pin> tmc2660`. Switches the
    # spidev_transfer responder into the TMC2660 3-byte sub-mode for the
    # shared bus. cs_port/cs_pin are accepted for protocol parity with
    # the simavr bridge but unused here - the renode hook sees the full
    # datagram in one call, so a single shared register file/RDSEL works
    # (klippy initialises drivers sequentially). Re-issuing is harmless.
    if str(kind) == 'tmc2660':
        _spi_tmc2660_enabled[0] = True
        _spi_tmc2660_rdsel[0] = 0
        _log("spi_tmc_chip: tmc2660 mode (cs=%s%s)", cs_port, cs_pin)
        _ensure_spi_tmc_hook_installed()


def apply_spi_tmc_symbol(addr):
    try:
        _spi_tmc_symbol[0] = int(addr)
        _log("spi_tmc symbol spidev_transfer -> %x", int(addr))
    except (TypeError, ValueError) as e:
        _log("apply_spi_tmc_symbol: bad addr %r: %s", addr, e)
        return
    _ensure_spi_tmc_hook_installed()


def _ensure_spi_tmc_hook_installed():
    if _spi_tmc_hook_installed[0]:
        return
    if not (_spi_tmc_enabled[0] or _spi_tmc2660_enabled[0]):
        return
    if not _HAVE_CPU_HOOK:
        _log("spi_tmc: CpuAddressHook unavailable")
        return
    addr = _spi_tmc_symbol[0]
    if addr is None:
        _log("spi_tmc: spidev_transfer symbol not resolved yet")
        return
    cpu = _cpu()
    try:
        cpu.AddHook(int(addr), CpuAddressHook(_on_spidev_transfer_hook))
        _spi_tmc_hook_installed[0] = True
        _log("spi_tmc hook installed: spidev_transfer @ %x", int(addr))
    except Exception as e:
        _log("spi_tmc hook install FAILED @ %x: %s", int(addr), e)


def _on_spidev_transfer_hook(cpu, pc):
    try:
        _do_spidev_transfer(cpu, pc)
    except Exception as e:
        _log("spi_tmc hook EXCEPTION: %s", e)


def _spi_pc_skip(cpu):
    # Skip the real (garbage-returning) hardware transfer: return to the
    # caller immediately by pointing PC at the link register. Clear the
    # thumb bit (LR bit0=1 on Cortex-M) - a PC with bit0 set faults.
    # SetRegisterUnsafe / the PC setter want a RegisterValue, not a raw
    # int, so build one (fall back to LR's own RegisterValue object).
    lr_rv = cpu.GetRegisterUnsafe(14)
    lr = int(lr_rv.RawValue) & 0xfffffffe
    try:
        cpu.SetRegisterUnsafe(15, RegisterValue.Create(lr, 32))
    except Exception as e1:
        try:
            cpu.SetRegisterUnsafe(15, lr_rv)
            _log("spi_tmc PC-skip via raw LR RegisterValue (Create err: %s)",
                 e1)
        except Exception as e2:
            _log("spi_tmc PC-skip FAILED: %s / %s", e1, e2)


def _do_tmc2660_transfer(cpu, data_ptr):
    # TMC2660: a 3-byte (20-bit) datagram. The 24-bit SPI stream carries
    # the 20-bit datagram in its low 20 bits (top 4 bits zero):
    #   byte0 = [0,0,0,0, d19,d18,d17,d16]  -> reg-id = (byte0>>1)&0x7,
    #                                          val bit16 = byte0&1
    #   byte1 = d15..d8,  byte2 = d7..d0
    # The MISO response is the chip's READRSP@RDSEL register, packed left
    # by 4 to occupy bits 23..4. klippy sets RDSEL via DRVCONF (reg-id 7,
    # field at val[5..4]) "first", then its periodic check reads the "se"
    # (current-scaler) field at RDSEL2 - a zero there decodes as
    # "0(Reset?)" and shuts down once motion starts, so serve se=5 (the
    # field lands at response_value bit 10 after klippy's 20-bit decode).
    sb = _M.Machine.SystemBus
    mosi = [int(sb.ReadByte(data_ptr + i)) & 0xff for i in range(3)]
    # Response reflects the RDSEL set by a PRIOR datagram (real silicon
    # latches DRVCONF.RDSEL and serves it on the next transaction).
    rdsel = _spi_tmc2660_rdsel[0]
    response_value = (5 << 10) if rdsel == 2 else 0
    packed = (response_value << 4) & 0xFFFFF
    resp = [(packed >> 16) & 0xff, (packed >> 8) & 0xff, packed & 0xff]
    for i in range(3):
        sb.WriteByte(data_ptr + i, resp[i])
    # Now latch this datagram's RDSEL if it is a DRVCONF write.
    b0 = mosi[0]
    reg_id = (b0 >> 1) & 0x7
    val = ((b0 & 1) << 16) | (mosi[1] << 8) | mosi[2]
    if reg_id == 7:
        _spi_tmc2660_rdsel[0] = (val >> 4) & 0x3
    if _spi_tmc_dbg[0] < 6:
        _spi_tmc_dbg[0] += 1
        _log("tmc2660 xfer reg=%d val=%05x rdsel=%d -> resp=%02x%02x%02x",
             reg_id, val, _spi_tmc2660_rdsel[0], resp[0], resp[1], resp[2])
    _spi_pc_skip(cpu)


def _do_spidev_transfer(cpu, pc):
    # spidev_transfer(struct spidev_s *spi, uint8_t receive_data,
    #                 uint8_t data_len, uint8_t *data)
    #   R0 = spi, R1 = receive_data, R2 = data_len, R3 = data ptr
    data_len = int(cpu.GetRegisterUnsafe(2).RawValue) & 0xff
    data_ptr = int(cpu.GetRegisterUnsafe(3).RawValue)
    if _spi_tmc2660_enabled[0] and data_len == 3:
        # TMC2660 3-byte datagram path (separate from the 5-byte chain).
        _do_tmc2660_transfer(cpu, data_ptr)
        return
    if data_len == 0 or (data_len % 5) != 0:
        # Not a 5-byte-datagram TMC transaction; let the real transfer
        # run (don't skip). No TMC config in scope hits this, but stay
        # safe for any non-TMC SPI device on the bus.
        return
    sb = _M.Machine.SystemBus
    mosi = []
    for i in range(data_len):
        mosi.append(int(sb.ReadByte(data_ptr + i)) & 0xff)
    n_slots = data_len // 5
    resp = [0] * data_len
    for s in range(n_slots):
        off = s * 5
        prev = _spi_tmc_prev_addr.get(s, 0)
        val = _spi_tmc_regs.get(prev, 0) & 0xffffffff
        resp[off] = 0                       # SPI status = OK
        resp[off + 1] = (val >> 24) & 0xff
        resp[off + 2] = (val >> 16) & 0xff
        resp[off + 3] = (val >> 8) & 0xff
        resp[off + 4] = val & 0xff
        addr = mosi[off]
        reg = addr & 0x7f
        if addr & 0x80:                     # write
            data = ((mosi[off + 1] << 24) | (mosi[off + 2] << 16)
                    | (mosi[off + 3] << 8) | mosi[off + 4])
            if reg == 0x01:                 # GSTAT: write-1-to-clear
                _spi_tmc_regs[reg] = _spi_tmc_regs.get(reg, 0) & ~data
            else:
                _spi_tmc_regs[reg] = data
        _spi_tmc_prev_addr[s] = reg
    for i in range(data_len):
        sb.WriteByte(data_ptr + i, resp[i])
    if _spi_tmc_dbg[0] < 6:
        _spi_tmc_dbg[0] += 1
        _log("spi_tmc xfer len=%d ptr=%x reg0=%02x",
             data_len, data_ptr, mosi[0])
    _spi_pc_skip(cpu)


# --------------------------------------------------------------------
# Synchronous serial bridge (host-link UART for tick mode).
#
# In real-time mode Renode's `emulation CreateUartPtyTerminal` wires
# the firmware's host-link USART to a host pty that klippy connects
# to via pyserial. That works because Renode runs continuously and
# Renode's pty terminal thread drains the firmware-side bytes onto
# the pty as they're emitted.
#
# In tick mode the pty path breaks down: klippy's C serialqueue
# background thread polls the pty fd on wall-clock time, while
# klippy's reactor advances sim_time only via tick-socket round
# trips. The two clocks decouple and the request/response sequence
# misses klippy's identify timeout window.
#
# This bridge replaces the pty terminal with a launcher-managed
# pty pair plus a synchronous byte shuttle: the launcher reads any
# bytes klippy wrote, hands them to the firmware UART here via
# WriteChar, runs the emulation forward, then reads back any bytes
# the firmware emitted (captured here via the CharReceived event).
# Byte delivery is exactly synchronous with RunFor return, which is
# what klippy's serialqueue thread needs to see in tick mode.

_serial_uart = [None]
_serial_rx_buf = bytearray()
try:
    import threading as _threading
    _serial_rx_lock = _threading.Lock()
except ImportError:
    _serial_rx_lock = None


def serial_init(uart_path):
    # Subscribe to CharReceived on the named UART and start
    # accumulating firmware-emitted bytes for serial_drain_hex.
    try:
        uart = _M.Machine[uart_path]
    except Exception as e:
        _log("serial_init: cannot find %s: %s", uart_path, e)
        return
    _serial_uart[0] = uart

    def _on_char(c):
        b = int(c) & 0xff
        if _serial_rx_lock is not None:
            _serial_rx_lock.acquire()
            try:
                _serial_rx_buf.append(b)
            finally:
                _serial_rx_lock.release()
        else:
            _serial_rx_buf.append(b)

    try:
        uart.CharReceived += _on_char
    except Exception as e:
        _log("serial_init: CharReceived subscribe err: %s", e)
        return
    _log("serial_init: bridged %s", uart_path)


def serial_write_hex(hex_str):
    # Inject a hex-encoded byte stream into the firmware's UART RX.
    # Each WriteChar call delivers one byte to the model; the firmware
    # picks them up via its serial RX interrupt during the next
    # RunFor.
    uart = _serial_uart[0]
    if uart is None or not hex_str:
        return
    try:
        for i in range(0, len(hex_str), 2):
            uart.WriteChar(int(hex_str[i:i + 2], 16))
    except Exception as e:
        _log("serial_write_hex: err at offset %d: %s", i, e)


def serial_drain_hex():
    # Return accumulated firmware-output bytes as a hex string, then
    # clear the buffer. Stdout output is what the launcher's Monitor
    # response parser will pick up.
    import sys
    # Peek SchedStatus.shutdown_reason on every drain (free - the
    # Monitor call to serial_drain_hex is already running); _log()
    # writes to /tmp/sw_uart_trace.log whenever the value changes.
    peek_shutdown_reason()
    if _serial_rx_lock is not None:
        _serial_rx_lock.acquire()
    try:
        if not _serial_rx_buf:
            return
        s = ''.join('%02x' % b for b in _serial_rx_buf)
        del _serial_rx_buf[:]
    finally:
        if _serial_rx_lock is not None:
            _serial_rx_lock.release()
    sys.stdout.write(s)


def serial_tick(tx_hex, delta_us):
    # Combined "host serial shuttle one quantum" entry point used by
    # the launcher to avoid round-tripping 3 separate Monitor commands
    # per klippy tick (write_hex, RunFor, drain_hex). The launcher
    # invokes this from a single `python "..."` so the wall-clock
    # overhead for one tick advance is ~10 ms instead of ~30 ms.
    #
    #   tx_hex   - klippy -> firmware bytes (hex)
    #   delta_us - microseconds of virtual time to advance
    #
    # Prints the firmware-emitted bytes (hex) to stdout so the launcher
    # response parser picks them up; no return value.
    import sys
    from Antmicro.Renode.Time import TimeInterval
    if tx_hex:
        serial_write_hex(tx_hex)
    if int(delta_us) > 0:
        # Use the master emulation time source via the Monitor's
        # python helper so we don't need to thread the launcher's
        # `monitor` reference here. The shortcut name `emulation`
        # injected by Renode at script init is not available in this
        # module scope; reach through EmulationManager directly.
        from Antmicro.Renode.Core import EmulationManager
        emu = EmulationManager.Instance.CurrentEmulation
        emu.RunFor(TimeInterval.FromMicroseconds(int(delta_us)))
    serial_drain_hex()
