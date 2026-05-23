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
)
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


def _drive_rx(s, value):
    _gpio_port(s.rx_port).OnGPIO(s.rx_pin, bool(value))


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
    cpu = _M.Machine['sysbus.cpu']
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
