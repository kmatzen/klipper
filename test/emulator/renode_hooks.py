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


def _gpio_port(letter):
    return _M.Machine['sysbus.gpioPort' + letter.upper()]


def _pin(port, idx):
    return _gpio_port(port).Connections[int(idx)]


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
_AFEC_BASES = (0x4003C000, 0x40064000)
_AFEC_MAGIC_DEFAULT = 0x100
_AFEC_MAGIC_CH_BASE = 0x104


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
    for base in _AFEC_BASES:
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
        slave.EnqueueResponseBytes(list(payload[:int(count)]))

    slave.DataReceived += on_data_received
    slave.ReadRequested += on_read_requested
    bus_obj.Register(slave, addr_int)
    _i2c_slaves[key] = (slave, last_reg_box, responses)
