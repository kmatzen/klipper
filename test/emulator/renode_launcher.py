#!/usr/bin/env python3
# Renode-backed bridge for klipper firmware testing.
#
# Spawns `renode` with a per-chip platform script that loads the
# klipper firmware ELF over USART1, exposes a host pty for klippy to
# connect to, and accepts the same line-oriented fixture control
# protocol as test/emulator/simavr_bridge.c. Fixture commands sent to
# --control-socket are translated into Renode Monitor commands sent
# over Renode's TCP Monitor port (-P), which in turn dispatch to the
# Python hook functions defined in test/emulator/renode_hooks.py
# (loaded into Renode at startup).
#
# Argument shape mirrors simavr_bridge so scripts/test_klippy.py can
# spawn either backend with the same kwargs - --sim-time-file and
# --tick-socket are accepted-and-ignored on first cut (Renode is
# real-time only until lockstep stepping is wired up; klippy in pure
# real-time mode falls back to live USART-based clock estimation,
# which is the production code path).

import argparse
import json
import os
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

# Per-chip Renode platform .repl path (relative to the Renode install).
# Most chips use upstream platforms verbatim; chips Renode does not
# ship a .repl for (currently just stm32h723) live under
# test/emulator/repl/ in this repo and are referenced by absolute path
# at .resc render time.
_LOCAL_REPL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'repl')
def _local(name):
    return '@' + os.path.join(_LOCAL_REPL_DIR, name)


# Platform map. Local overrides (test/emulator/repl/) extend the
# upstream Renode platforms with peripherals upstream doesn't model
# but klipper firmware initialises - notably ADC (missing on
# stm32f1.repl + stm32f4.repl + stm32f429.repl in Renode upstream)
# and SPI (missing on stm32f103.repl). For chips whose upstream
# platform already covers everything klipper needs (F0 / G0 / H7
# variants in scope today) we point at upstream verbatim. SAM4S8C
# memory layout (512KB flash + 128KB sram) matches sam4s8b, so we
# point at upstream sam4s8b.repl rather than carrying a duplicate
# locally.
_PLATFORM_FOR_CHIP = {
    'stm32f070': '@platforms/cpus/stm32f072.repl',
    'stm32f103': _local('stm32f103.repl'),
    'stm32f401': _local('stm32f4.repl'),
    'stm32f405': _local('stm32f4.repl'),
    'stm32f407': _local('stm32f4.repl'),
    'stm32f429': _local('stm32f429.repl'),
    'stm32f446': _local('stm32f4.repl'),
    'stm32g0b1': '@platforms/cpus/stm32g0.repl',
    'stm32h723': _local('stm32h723.repl'),
    'stm32h743': '@platforms/cpus/stm32h743.repl',
    'sam3x8c': _local('sam3x8e.repl'),
    'sam3x8e': _local('sam3x8e.repl'),
    'sam4s8c': '@platforms/cpus/sam4s8b.repl',
    'sam4e8e': _local('sam4e8e.repl'),
    'same70q20b': _local('same70q20b.repl'),
    'samd51p20': _local('samd51p20.repl'),
    'lpc176x': _local('lpc176x.repl'),
}

# Renode peripheral name for the UART/USART that klipper uses as the
# host link in -serial.config mode. STM32 -serial.config selects
# USART1 family-wide. Atmel chips don't share that convention -
# klipper's src/atsam/serial.c picks a chip-specific Atmel UART
# peripheral (UART1 on SAM4S, UART2 on SAME70) which the platform
# .repl exposes under different names; klipper's src/atsamd/serial.c
# always picks SERCOM0 on the SAMx5 family. Per-chip overrides keyed
# by the same chip basename as _PLATFORM_FOR_CHIP; chips not in the
# override map fall back to the STM32 default.
_DEFAULT_HOST_LINK_PERIPHERAL = 'usart1'
_HOST_LINK_FOR_CHIP = {
    'sam3x8c': 'uart',
    'sam3x8e': 'uart',
    'sam4s8c': 'uart1',
    'sam4e8e': 'uart0',
    'same70q20b': 'uart2',
    'samd51p20': 'sercom0',
    'lpc176x': 'uart0',
}


def _host_link_peripheral(chip):
    return _HOST_LINK_FOR_CHIP.get(chip, _DEFAULT_HOST_LINK_PERIPHERAL)

# Where renode_hooks.py lives, to be loaded into Renode at startup via
# `include @<path>` so all the hook functions (step_trigger, bltouch,
# adc_default, i2c_register_response, ...) are in scope before any
# fixture command arrives.
_HOOKS_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'renode_hooks.py')

_RCC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'rcc_stub.py')

_AFEC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'afec_stub.py')

_SAMD_OSCCTRL_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'samd_oscctrl_stub.py')
_SAMD_GCLK_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'samd_gclk_stub.py')
_SAMD_OSC32KCTRL_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'samd_osc32kctrl_stub.py')
_SAMD_STOREBACK_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'samd_storeback.py')

_LPC_SC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'lpc_sc_stub.py')

# Per-chip RCC peripheral base address (RM cross-reference per
# family). Most upstream Renode STM32 platforms ship some kind of
# RCC model: F4 has Miscellaneous.STM32F4_RCC, H7 has
# Miscellaneous.STM32H7_RCC, F0/G0 have their own
# Python.PythonPeripheral stubs at 0x40021000. STM32F1 is the
# exception - upstream stm32f103.repl has no rcc entry at all and
# the firmware spins on HSERDY/PLLRDY. We register our local
# rcc_stub.py only for F1.
#
# Adding a stub at an address that already has an RCC peripheral
# defined causes the platform load to fail silently (Renode
# discards the .resc remainder), so it's important the entries below
# stay restricted to chips whose upstream platform truly lacks RCC.
_RCC_BASE_FOR_CHIP = {
    'stm32f103': 0x40021000,
}

# Per-chip AFEC peripheral base addresses. SAME70 has AFEC0 / AFEC1
# (12 channels each); klipper's src/atsam/sam4e_afec.c initialises
# both. Renode does not ship a SAM_AFEC peripheral model, so we
# register Python.PythonPeripheral stubs (test/emulator/afec_stub.py)
# at each base. The stub serves AFE_LCDR / AFE_CDR reads with values
# poked through magic offsets by the renode_hooks adc_default /
# adc_set path.
_AFEC_BASES_FOR_CHIP = {
    'sam4e8e': (0x400B0000, 0x400B4000),
    'same70q20b': (0x4003C000, 0x40064000),
}

# Per-chip extra Python.PythonPeripheral stubs to inject after the
# platform load. Each entry is a list of (name, base, size, stub_path)
# tuples. The samd51p20 entries cover the chip's clock controllers
# (OSCCTRL / GCLK / OSC32KCTRL) and the simple store/return blocks
# (MCLK / CMCC) that klipper firmware touches during SystemInit() and
# enable_pclock() - none of which Renode upstream models. Without
# these the firmware spins forever in samd51_clock.c.
_EXTRA_PERIPHERAL_STUBS_FOR_CHIP = {
    'samd51p20': [
        ('oscctrl', 0x40001000, 0x80, _SAMD_OSCCTRL_STUB_PY),
        ('osc32kctrl', 0x40001400, 0x40, _SAMD_OSC32KCTRL_STUB_PY),
        ('gclk', 0x40001C00, 0x200, _SAMD_GCLK_STUB_PY),
        ('mclk', 0x40000800, 0x40, _SAMD_STOREBACK_PY),
        ('cmcc', 0x41006000, 0x40, _SAMD_STOREBACK_PY),
    ],
    'lpc176x': [
        ('lpc_sc', 0x400FC000, 0x200, _LPC_SC_STUB_PY),
    ],
}


def _chip_for_elf(elf_path):
    # Klipper builds yield ci_build/elf/<chip>.elf; the chip basename is
    # the lookup key for _PLATFORM_FOR_CHIP. For oddball test paths
    # (e.g. a one-off ELF passed by hand) we accept any name that
    # matches a known chip prefix.
    base = os.path.basename(elf_path)
    if base.endswith('.elf'):
        base = base[:-4]
    if base in _PLATFORM_FOR_CHIP:
        return base
    for chip in _PLATFORM_FOR_CHIP:
        if base.startswith(chip):
            return chip
    raise RuntimeError(
        "renode_launcher: no Renode platform mapping for ELF %r "
        "(expected basename matching one of %s)"
        % (elf_path, sorted(_PLATFORM_FOR_CHIP)))


def _allocate_tcp_port():
    # Bind ephemeral, read back the port, close. The kernel keeps the
    # port in TIME_WAIT briefly but Renode's listen will succeed
    # because we set SO_REUSEADDR (and Mono's TcpListener uses it by
    # default). For multi-instance parallel test runs each launcher
    # gets a distinct port; SO_REUSEADDR isn't sufficient if two
    # launchers race for the same port, so we wrap with a retry loop
    # in the caller.
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _render_resc(chip, elf_path, pty_path, monitor_port, log_path):
    # The .resc Renode runs on startup. mach create + LoadPlatform +
    # the RCC PythonPeripheral stub (so clock setup completes) +
    # LoadELF + UartPtyTerminal connected to USART1 + include of
    # renode_hooks.py + set_monitor(monitor) so the hook funcs can
    # resolve `monitor.Machine` (which their module namespace
    # otherwise can't see). We do NOT call `start` here - the
    # launcher sends `start` over the TCP Monitor after it has
    # finished pushing all fixture-driven hook registrations, so
    # peripheral state is configured before the CPU begins
    # executing.
    platform = _PLATFORM_FOR_CHIP[chip]
    rcc_base = _RCC_BASE_FOR_CHIP.get(chip)
    rcc_block = ''
    if rcc_base is not None:
        rcc_block = (
            'machine LoadPlatformDescriptionFromString '
            '"rcc: Python.PythonPeripheral @ sysbus 0x{rcc:08X} '
            '{{ size: 0x400; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(rcc=rcc_base, stub=_RCC_STUB_PY)
    afec_bases = _AFEC_BASES_FOR_CHIP.get(chip, ())
    afec_block = ''
    for idx, base in enumerate(afec_bases):
        afec_block += (
            'machine LoadPlatformDescriptionFromString '
            '"afec{idx}: Python.PythonPeripheral @ sysbus 0x{base:08X} '
            '{{ size: 0x200; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(idx=idx, base=base, stub=_AFEC_STUB_PY)
    extra_stubs = _EXTRA_PERIPHERAL_STUBS_FOR_CHIP.get(chip, ())
    extra_block = ''
    for name, base, size, stub in extra_stubs:
        extra_block += (
            'machine LoadPlatformDescriptionFromString '
            '"{name}: Python.PythonPeripheral @ sysbus 0x{base:08X} '
            '{{ size: 0x{size:X}; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(name=name, base=base, size=size, stub=stub)
    return (
        'using sysbus\n'
        'mach create "klipper-{chip}"\n'
        'machine LoadPlatformDescription {platform}\n'
        '{rcc_block}'
        '{afec_block}'
        '{extra_block}'
        'sysbus LoadELF @{elf}\n'
        'logFile @{log}\n'
        'logLevel 1\n'
        'emulation CreateUartPtyTerminal "uartTerm" "{pty}"\n'
        'connector Connect sysbus.{usart} uartTerm\n'
        'i @{hooks}\n'
        'python "import renode_hooks; renode_hooks.set_monitor(monitor)"\n'
    ).format(chip=chip, platform=platform, elf=elf_path,
             log=log_path, pty=pty_path, rcc_block=rcc_block,
             afec_block=afec_block, extra_block=extra_block,
             usart=_host_link_peripheral(chip), hooks=_HOOKS_PY)


def _wait_for_path(path, deadline, poll=0.05):
    while time.monotonic() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(poll)
    return False


def _connect_monitor(host, port, deadline):
    # Renode's TCP monitor takes a moment to come up after process
    # spawn. Retry until either it accepts a connection or we miss the
    # deadline.
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection((host, port), timeout=1.0)
            s.settimeout(None)
            return s
        except (ConnectionRefusedError, OSError):
            time.sleep(0.05)
    raise RuntimeError(
        "renode_launcher: Renode TCP Monitor on %s:%d did not "
        "accept connection within deadline" % (host, port))


_PROMPT_RE = re.compile(rb'\([\w-]+\)\s*$')


def _drain_monitor_until_prompt(sock, timeout=10.0):
    # Renode's monitor echoes commands and prints `(<context>) ` as a
    # prompt. We treat the prompt as the response delimiter. Returns
    # the bytes received between the last command and the prompt.
    deadline = time.monotonic() + timeout
    buf = b''
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(
                "renode_launcher: monitor read timed out after %fs; "
                "buffer=%r" % (timeout, buf[-200:]))
        r, _, _ = select.select([sock], [], [], min(remaining, 0.5))
        if not r:
            continue
        try:
            chunk = sock.recv(4096)
        except OSError:
            return buf
        if not chunk:
            return buf
        buf += chunk
        if _PROMPT_RE.search(buf):
            return buf


def _send_monitor(sock, line):
    # Trailing \n is required; carriage return optional but harmless.
    sock.sendall((line + '\n').encode('utf-8'))
    return _drain_monitor_until_prompt(sock)


# ---------------------------------------------------------------------
# Fixture control protocol -> Renode hook function call translation.
#
# Mirrors the command vocabulary that
# scripts/test_klippy.py:_push_fixture_to_control_socket emits and
# that test/emulator/simavr_bridge.c parses. Each function returns the
# Python expression to evaluate inside Renode's MonitorPythonEngine
# (which has the renode_hooks.py module's symbols in scope).

def _xlat_step_trigger(parts):
    # step_trigger <step_port> <step_pin> <count> <trig_port> <trig_pin> <val>
    # eg "step_trigger A 5 100 B 7 1" -> "step_trigger('A', 5, 100, 'B', 7, 1)"
    if len(parts) != 7:
        return None
    return ("step_trigger('%s', %d, %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3]),
               parts[4], int(parts[5]), int(parts[6])))


def _xlat_probe_step(parts):
    # probe_step <step_port> <step_pin> <reset_us> <force_per_step>
    #            <trig_port> <trig_pin> <val>
    if len(parts) != 8:
        return None
    return ("probe_step('%s', %d, %d, %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3]), int(parts[4]),
               parts[5], int(parts[6]), int(parts[7])))


def _xlat_bltouch(parts):
    # bltouch <ctrl_port> <ctrl_pin> <sensor_port> <sensor_pin> <invert>
    if len(parts) != 6:
        return None
    return ("bltouch('%s', %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), parts[3], int(parts[4]),
               int(parts[5])))


def _xlat_gpio(parts):
    # gpio <port> <pin> <val>
    if len(parts) != 4:
        return None
    return ("gpio_set('%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3])))


def _xlat_passthrough(parts):
    # Catch-all: forward as a function call with the same name. Lets
    # us add new fixture commands without re-touching the launcher as
    # long as renode_hooks.py grows the matching function.
    name = parts[0]
    args = ', '.join(repr(p) for p in parts[1:])
    return "%s(%s)" % (name, args)


_XLAT = {
    'step_trigger': _xlat_step_trigger,
    'probe_step': _xlat_probe_step,
    'bltouch': _xlat_bltouch,
    'gpio': _xlat_gpio,
}


# Default I2C addresses to register the empty fixture's
# i2c_default.register_responses against. Covers the LDC1612 default
# (0x2a) and its alternate (0x29) - the only I2C device klippy probes
# at startup with a fixed ID (and so the only one that hard-fails an
# entire printer config when the response is wrong / missing). Other
# I2C devices in printer configs get probed with their own register
# vocabularies; expand this list as concrete tests surface failures.
_DEFAULT_I2C_ADDRS = (0x2a, 0x29)


def _apply_fixture_to_renode(fixture_path, monitor_sock):
    # Read the fixture file and emit the renode-hook calls that
    # simavr_bridge.c handles internally. step_trigger / bltouch /
    # probe_step come over the control socket from
    # _push_fixture_to_control_socket - we don't duplicate those
    # here. analog_in, analog_in_default, i2c_default are NOT pushed
    # over the control socket by the test runner today, so the
    # launcher is the right place to consume them for the renode
    # backend.
    if not fixture_path or not os.path.isfile(fixture_path):
        return
    try:
        with open(fixture_path) as f:
            fx = json.load(f)
    except (OSError, ValueError) as e:
        sys.stderr.write(
            "renode_launcher: ignoring unreadable fixture %r: %s\n"
            % (fixture_path, e))
        return

    def _send(py_call):
        _send_monitor(monitor_sock,
                      'python "%s"' % py_call.replace('"', r'\"'))

    # All-channels ADC default.
    default_block = fx.get('analog_in_default')
    if isinstance(default_block, dict):
        dv = default_block.get('default_value')
        if dv is not None:
            _send('adc_default(%d)' % int(dv))

    # Per-channel overrides. The keys in `analog_in` are labels
    # (typically `_hot_extruder_N`) that the AVR fixture pusher maps
    # to the FIRST few configured analog_in OIDs. We don't have the
    # OID assignment here, so we use the order of dict iteration as
    # the channel index - close enough for the empty-fixture case
    # where the goal is just to get extruder thermistors to decode
    # to plausible temperatures rather than min_temp shutdowns.
    analog_in = fx.get('analog_in')
    if isinstance(analog_in, dict):
        for ch_idx, (_label, spec) in enumerate(analog_in.items()):
            if not isinstance(spec, dict):
                continue
            v = spec.get('default_value')
            if v is None:
                continue
            _send('adc_set(%d, %d)' % (ch_idx, int(v)))

    # I2C ID-probe responses.
    i2c_default = fx.get('i2c_default')
    if isinstance(i2c_default, dict):
        reg_resp = i2c_default.get('register_responses')
        if isinstance(reg_resp, dict) and reg_resp:
            # Normalise the dict to a JSON-safe form for embedding in
            # a python string (bytes lists -> int lists, hex keys
            # passed through as strings - the hook re-parses with
            # int(k, 0)).
            normalised = {str(k): list(v) for k, v in reg_resp.items()}
            payload_json = json.dumps(normalised)
            for addr in _DEFAULT_I2C_ADDRS:
                _send('i2c_register_response(1, %d, %s)'
                      % (addr, payload_json))


def _translate_fixture_command(line):
    parts = line.strip().split()
    if not parts:
        return None
    fn = _XLAT.get(parts[0], _xlat_passthrough)
    return fn(parts)


# ---------------------------------------------------------------------

def _control_socket_loop(sock_path, monitor_sock, stop_evt):
    # Accept connections from _push_fixture_to_control_socket. Each
    # newline-terminated command is translated to a Renode python
    # call and forwarded over the monitor socket. After all commands
    # the test runner sends `start` (mirroring simavr_bridge.c's
    # control protocol); on receipt we issue Renode's `start` to
    # begin CPU execution.
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.settimeout(0.5)
    started = False
    while not stop_evt[0]:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        try:
            buf = b''
            while not stop_evt[0]:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, _, buf = buf.partition(b'\n')
                    text = line.decode('utf-8', 'replace').strip()
                    if not text:
                        continue
                    if text == 'start' and not started:
                        _send_monitor(monitor_sock, 'start')
                        started = True
                        try:
                            conn.sendall(b'OK\n')
                        except OSError:
                            pass
                        continue
                    py_call = _translate_fixture_command(text)
                    if py_call is None:
                        sys.stderr.write(
                            "renode_launcher: unrecognized fixture "
                            "command %r\n" % text)
                        continue
                    _send_monitor(
                        monitor_sock,
                        'python "%s"' % py_call.replace('"', r'\"'))
                    try:
                        conn.sendall(b'OK\n')
                    except OSError:
                        pass
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
    if not started:
        # No `start` was issued (test runner gave up early or never
        # connected the control socket - happens in single-shot
        # diagnostic runs). Start the CPU so the firmware at least
        # boots and we can observe the failure mode.
        try:
            _send_monitor(monitor_sock, 'start')
        except (OSError, RuntimeError):
            pass
    try:
        srv.close()
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', required=True)
    ap.add_argument('--slave-link', required=True,
                    help='symlink published to the host pty Renode opens '
                         'for USART1; klippy connects to this path.')
    ap.add_argument('--control-socket', required=True,
                    help='unix socket that receives newline-terminated '
                         'fixture commands (same protocol as '
                         'simavr_bridge.c).')
    ap.add_argument('--duration', type=float, default=120.0,
                    help='hard wall-clock cap on this run (seconds).')
    # Accepted for arg-shape parity with simavr_bridge; ignored on
    # first cut (Renode runs real-time, klippy uses live clock
    # estimation over USART when KLIPPY_SIM_TIME_FILE is unset).
    ap.add_argument('--sim-time-file', default=None)
    ap.add_argument('--tick-socket', default=None)
    # The fixture file the test runner is about to push commands
    # from. simavr_bridge.c handles analog_in / i2c_default /
    # spi_response by reading the fixture itself (out-of-band of the
    # control socket); we mirror that here so the renode hooks for
    # ADC defaults and LDC1612 ID probes get applied before klippy
    # starts running. Optional - if omitted, only the control-socket
    # commands take effect (sufficient for tests with empty or
    # GPIO-only fixtures).
    ap.add_argument('--fixture-file', default=None)
    args = ap.parse_args()

    chip = _chip_for_elf(args.elf)

    workdir = tempfile.mkdtemp(prefix='renode_launcher_')
    pty_path = os.path.join(workdir, 'uart.pty')
    resc_path = os.path.join(workdir, 'launch.resc')
    log_path = os.path.join(workdir, 'renode.log')

    monitor_port = _allocate_tcp_port()
    resc = _render_resc(chip, os.path.abspath(args.elf), pty_path,
                        monitor_port, log_path)
    with open(resc_path, 'w') as f:
        f.write(resc)

    renode = shutil.which('renode')
    if renode is None:
        sys.stderr.write("renode_launcher: `renode` not on PATH\n")
        return 2

    # --plain disables ANSI colors (cleaner log capture); --disable-xwt
    # avoids any GUI initialization (works headless on Linux Docker
    # without X). -P binds the Monitor to a TCP port so we can drive
    # it from this process; -e includes our generated .resc.
    cmd = [
        renode, '--plain', '--disable-xwt',
        '-P', str(monitor_port),
        '-e', 'include @' + resc_path,
    ]

    deadline = time.monotonic() + args.duration
    proc = subprocess.Popen(cmd, stdout=sys.stderr, stderr=sys.stderr)

    stop_evt = [False]

    def _shutdown(*_a):
        stop_evt[0] = True

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        # Wait for the pty Renode creates, then publish the symlink so
        # klippy's _wait_for_slave_link sees the slave path.
        if not _wait_for_path(pty_path,
                              min(deadline, time.monotonic() + 30.0)):
            sys.stderr.write(
                "renode_launcher: Renode did not create UART pty at "
                "%s within 30s; aborting\n" % pty_path)
            return 3
        # The pty file Renode creates IS the slave end (Renode opens
        # /dev/ptmx, then symlinks the published name to the slave
        # pts/N). Mirror linuxprocess: publish slave_link as a
        # symlink to it.
        try:
            os.unlink(args.slave_link)
        except OSError:
            pass
        os.symlink(pty_path, args.slave_link)

        monitor_sock = _connect_monitor(
            '127.0.0.1', monitor_port,
            min(deadline, time.monotonic() + 30.0))
        _drain_monitor_until_prompt(monitor_sock)

        # Apply the fixture-resident hooks (ADC defaults, I2C ID
        # responses) BEFORE the control loop accepts the runner's
        # `start` command - those hook calls have to be in place
        # before the CPU starts executing or klippy may probe a
        # peripheral and shutdown before the response arrives.
        _apply_fixture_to_renode(args.fixture_file, monitor_sock)

        import threading
        ctl_thread = threading.Thread(
            target=_control_socket_loop,
            args=(args.control_socket, monitor_sock, stop_evt),
            daemon=True)
        ctl_thread.start()

        # Wait for Renode subprocess or duration cap.
        while not stop_evt[0]:
            if proc.poll() is not None:
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)
    finally:
        stop_evt[0] = True
        try:
            proc.terminate()
        except OSError:
            pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                proc.kill()
            except OSError:
                pass
        for p in (args.slave_link, args.control_socket):
            try:
                os.unlink(p)
            except OSError:
                pass
        try:
            shutil.rmtree(workdir, ignore_errors=True)
        except OSError:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
