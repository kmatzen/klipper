#!/usr/bin/env python3
# Standalone Renode probe runner. Exists to validate the API
# assumptions baked into renode_launcher.py and renode_hooks.py
# without rebuilding the full klipper test runtime each iteration.
#
# Run inside the emulator-test Docker image:
#   docker run --rm klipper-emu python3 test/emulator/validate_renode.py
# or interactively if iterating on failures:
#   docker run --rm -it --entrypoint bash klipper-emu
#   /klipper# python3 test/emulator/validate_renode.py
#
# Each probe prints `[PASS]` / `[FAIL]` with the raw Monitor response
# so a failed assumption pinpoints exactly which Renode command form
# needs adjusting in the launcher/hooks. Probes run sequentially - a
# failure short-circuits the rest because later probes assume the
# earlier setup succeeded (e.g. pin-level hook registration assumes
# the platform loaded).

import os
import re
import select
import shutil
import socket
import subprocess
import sys
import time

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         os.pardir, os.pardir))
ELF_PATH = os.path.join(REPO_ROOT, 'ci_build', 'elf', 'stm32f103.elf')
PTY_PATH = '/tmp/renode_validate.pty'

PROMPT_RE = re.compile(rb'\([\w-]+\)\s*$')


def _allocate_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _connect_monitor(port, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection(('127.0.0.1', port), timeout=1.0)
            s.settimeout(None)
            return s
        except (ConnectionRefusedError, OSError):
            time.sleep(0.1)
    raise RuntimeError("Renode TCP Monitor did not accept connection")


def _drain(sock, timeout=10.0):
    deadline = time.monotonic() + timeout
    buf = b''
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return buf
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
        if PROMPT_RE.search(buf):
            return buf


def _send(sock, line, timeout=10.0):
    sock.sendall((line + '\n').encode('utf-8'))
    return _drain(sock, timeout=timeout)


_total = [0]
_failed = [0]


def _probe(name, ok_predicate, response_bytes):
    _total[0] += 1
    txt = response_bytes.decode('utf-8', 'replace')
    ok = ok_predicate(txt)
    label = '[PASS]' if ok else '[FAIL]'
    if not ok:
        _failed[0] += 1
    sys.stdout.write('%s %s\n' % (label, name))
    # Limit response dump to last ~12 lines to keep the log scannable
    # while still preserving error context.
    tail = '\n'.join(txt.splitlines()[-12:])
    sys.stdout.write('   --- response (tail) ---\n')
    for line in tail.splitlines():
        sys.stdout.write('   | %s\n' % line)
    sys.stdout.write('   ------------------------\n')
    sys.stdout.flush()
    return ok


def _is_clean(txt):
    # Renode reports errors as lines containing "error" / "exception"
    # / "could not" - any of those is a fail signal. A clean
    # successful command echoes back its own text and prompt only.
    low = txt.lower()
    bad = ('error', 'exception', 'could not', 'no such',
           'unhandled', 'failed', 'unknown command')
    return not any(b in low for b in bad)


def _contains(needle):
    needle = needle.lower()
    return lambda t: needle in t.lower() and _is_clean(t)


def main():
    if not os.path.isfile(ELF_PATH):
        sys.stderr.write(
            "[setup] missing %s - rebuild the Docker image so the "
            "stm32f103 firmware is present\n" % ELF_PATH)
        return 2
    renode = shutil.which('renode')
    if renode is None:
        sys.stderr.write("[setup] `renode` not on PATH\n")
        return 2
    if os.path.exists(PTY_PATH):
        os.unlink(PTY_PATH)

    port = _allocate_port()
    # The whole point of this script is to discover what works, so we
    # do NOT pre-load any .resc - just bring Renode up empty with the
    # TCP Monitor, then drive it command by command from this side.
    proc = subprocess.Popen(
        [renode, '--plain', '--disable-xwt', '--hide-monitor',
         '-P', str(port)],
        stdout=sys.stderr, stderr=sys.stderr,
        stdin=subprocess.DEVNULL)
    try:
        sock = _connect_monitor(port)
        # Banner / initial prompt drain.
        _drain(sock)

        # Probe 1 - basic command roundtrip (`version` returns the
        # build identifier; failure here means our prompt parser is
        # wrong).
        r = _send(sock, 'version')
        _probe('version command roundtrip', _contains('renode'), r)

        # Probe 2 - mach create.
        r = _send(sock, 'mach create "validate"')
        _probe('mach create', _is_clean, r)

        # Probe 3 - platform load (needs to find platforms/ in
        # Renode's resources path; the .deb installs it under
        # /opt/renode/platforms).
        r = _send(sock,
                  'machine LoadPlatformDescription '
                  '@platforms/cpus/stm32f103.repl')
        _probe('LoadPlatformDescription stm32f103', _is_clean, r)

        # Probe 3b - register the RCC PythonPeripheral stub before
        # LoadELF, so the firmware's clock_setup() can complete when
        # the CPU starts. Without this stub klipper spins forever
        # waiting for HSERDY / PLLRDY bits the SVD-derived RCC
        # placeholders never raise.
        rcc_stub = os.path.join(REPO_ROOT, 'test', 'emulator',
                                'rcc_stub.py')
        r = _send(sock,
                  'machine LoadPlatformDescriptionFromString '
                  '"rcc: Python.PythonPeripheral @ sysbus 0x40021000 '
                  '{ size: 0x400; initable: true; '
                  'filename: \\"%s\\" }"' % rcc_stub)
        _probe('Register RCC PythonPeripheral stub', _is_clean, r)

        # Probe 4 - ELF load.
        r = _send(sock, 'sysbus LoadELF @' + ELF_PATH)
        _probe('sysbus LoadELF (klipper firmware)', _is_clean, r)

        # Probe 5 - UART pty terminal + connector.
        r = _send(sock,
                  'emulation CreateUartPtyTerminal "uartTerm" "%s"'
                  % PTY_PATH)
        _probe('CreateUartPtyTerminal', _is_clean, r)
        r = _send(sock, 'connector Connect sysbus.usart1 uartTerm')
        _probe('connector Connect usart1 -> uartTerm', _is_clean, r)
        # Verify the pty file appeared.
        appeared = False
        for _ in range(40):
            if os.path.exists(PTY_PATH):
                appeared = True
                break
            time.sleep(0.05)
        _probe('pty file appeared at %s' % PTY_PATH,
               lambda _t: appeared, b'pty path: %s exists=%s'
               % (PTY_PATH.encode(), str(appeared).encode()))

        # Probe 6 - python state persistence across Monitor calls.
        # Critical for renode_hooks.py - functions defined in one
        # `python "..."` call must be callable from a later call.
        # IronPython 2.7 prints `print(a, b)` as the tuple repr
        # `('a', b)`, so we accept either Py2 tuple form or Py3
        # space-separated form.
        _send(sock, 'python "x = 12345"')
        r = _send(sock, 'python "print(\\"X=\\", x)"')
        rtxt = r.decode('utf-8', 'replace').lower()
        _probe('python state persists across calls',
               lambda _t: '12345' in rtxt and 'name' not in rtxt
               and 'error' not in rtxt, r)

        # Probe 7 - peripheral lookup via monitor.Machine[''].
        # MonitorPythonEngine injects only `monitor` and `self` (the
        # Monitor itself) into the Python scope; current machine
        # comes from monitor.Machine, NOT a top-level `machine` var.
        r = _send(sock,
                  'python "p = monitor.Machine[\\"sysbus.gpioPortA\\"]; '
                  'print(\\"PORT_TYPE=\\", type(p).__name__)"')
        _probe('monitor.Machine[\'sysbus.gpioPortA\'] lookup',
               _contains('PORT_TYPE='), r)

        # Probe 8 - pin-level GPIO object access. Two forms tested -
        # the launcher currently uses Connections[N], but if that
        # fails we want to know whether ConnectionsByNumber or some
        # other accessor works.
        r = _send(sock,
                  'python "pin = p.Connections[5]; '
                  'print(\\"PIN_TYPE=\\", type(pin).__name__)"')
        _probe('p.Connections[5] indexing',
               _contains('PIN_TYPE='), r)
        # Fallback exploration even if probe 8 passes - tells us
        # whether ConnectionsByNumber exists as an alternative.
        r = _send(sock,
                  'python "import sys; '
                  'print(\\"HAS_CBNUM=\\", '
                  'hasattr(p, \\"ConnectionsByNumber\\"))"')
        _probe('hasattr ConnectionsByNumber (info only)',
               _contains('has_cbnum='), r)

        # Probe 9 - System.Action import (needed for the
        # closure-based hook registration in renode_hooks.py).
        r = _send(sock,
                  'python "from System import Action; '
                  'print(\\"ACTION_OK=\\", Action is not None)"')
        rtxt = r.decode('utf-8', 'replace').lower()
        _probe('System.Action import',
               lambda _t: 'true' in rtxt and 'error' not in rtxt, r)

        # Probe 10 - import renode_hooks (after sys.path
        # manipulation). renode_hooks.py also imports
        # Antmicro.Renode.Peripherals.Mocks.DummyI2CSlave - if that
        # fails, the import-or-stub fallback inside renode_hooks
        # disables I2C support but the rest of the module still
        # loads.
        hooks_dir = os.path.join(REPO_ROOT, 'test', 'emulator')
        _send(sock,
              'python "import sys; sys.path.append(\\"%s\\")"'
              % hooks_dir)
        r = _send(sock,
                  'python "import renode_hooks; '
                  'print(\\"HOOKS_OK=\\", '
                  'hasattr(renode_hooks, \\"step_trigger\\"), '
                  'hasattr(renode_hooks, \\"adc_default\\"), '
                  'hasattr(renode_hooks, \\"i2c_register_response\\"))"')
        rtxt = r.decode('utf-8', 'replace').lower()
        _probe('import renode_hooks (all hook funcs)',
               lambda _t: 'true' in rtxt and 'error' not in rtxt
               and 'traceback' not in rtxt, r)

        # Probe 10b - report whether DummyI2CSlave loaded inside
        # renode_hooks (best effort; if the IronPython auto-discovery
        # of Antmicro.Renode.Peripherals.Mocks failed at import
        # time, _HAVE_DUMMY_I2C is False and i2c hooks are no-ops).
        r = _send(sock,
                  'python "print(\\"HAVE_I2C=\\", '
                  'renode_hooks._HAVE_DUMMY_I2C)"')
        _probe('DummyI2CSlave import (info)', _is_clean, r)

        # Probe 10c - end-to-end: register a hook via renode_hooks's
        # closure-based path on the same pin. If this works, the
        # production launcher path from `python "step_trigger(...)"`
        # over the Monitor TCP is unblocked. Caller must
        # set_monitor() first because the imported module's namespace
        # doesn't see the Monitor's `monitor` global.
        _send(sock,
              'python "renode_hooks.set_monitor(monitor)"')
        r = _send(sock,
                  'python "renode_hooks.step_trigger('
                  '\\"A\\", 5, 100, \\"B\\", 7, 1)"')
        _probe('renode_hooks.step_trigger(...) registers',
               _is_clean, r)

        # Probe 11 - start CPU + observe firmware boot on pty.
        r = _send(sock, 'start')
        _probe('start CPU', _is_clean, r)

        if os.path.exists(PTY_PATH):
            try:
                # Open non-blocking and read whatever the firmware
                # sends in 2 seconds. klipper's identify exchange
                # starts with a 0x07-prefixed framed message; we
                # don't decode it, just confirm bytes flow.
                fd = os.open(PTY_PATH, os.O_RDWR | os.O_NOCTTY
                             | os.O_NONBLOCK)
                deadline = time.monotonic() + 2.0
                buf = b''
                while time.monotonic() < deadline and len(buf) < 32:
                    r, _, _ = select.select([fd], [], [], 0.2)
                    if r:
                        try:
                            buf += os.read(fd, 256)
                        except OSError:
                            break
                os.close(fd)
                _probe('firmware bytes flow over pty (any data)',
                       lambda _t: len(buf) > 0,
                       ('bytes=%d head=%r' % (len(buf), buf[:32]))
                       .encode())
            except OSError as e:
                _probe('open pty for read', lambda _t: False,
                       str(e).encode())
    finally:
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
        try:
            os.unlink(PTY_PATH)
        except OSError:
            pass

    sys.stdout.write('\nSUMMARY: %d/%d probes passed (%d failed)\n'
                     % (_total[0] - _failed[0], _total[0],
                        _failed[0]))
    return 0 if _failed[0] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
