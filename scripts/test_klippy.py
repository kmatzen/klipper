# Regression test helper script
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json, sys, os, optparse, logging, re, socket, subprocess, time

# Python 2.7 compatibility - test_klippy.py is run under both python2 and
# python3 in CI.
_monotonic = getattr(time, "monotonic", time.time)

TEMP_GCODE_FILE = "_test_.gcode"
TEMP_LOG_FILE = "_test_.log"
TEMP_OUTPUT_FILE = "_test_output"
TEMP_EMU_CFG = "_test_emu.cfg"
TEMP_EMU_LINK = "_test_emu.tty"
TEMP_EMU_LOG = "_test_emu.log"
TEMP_EMU_CTL = "_test_emu.ctl"

# Maximum wall-clock seconds klippy is allowed to run in emulator mode
# before we conclude the test is hung. Klippy in real-mcu mode does not
# exit on stdin EOF; we kill it after a timeout and rely on log checks.
EMULATOR_KLIPPY_DEADLINE = 20.0
SERIAL_PLACEHOLDER = "__EMULATOR_PTY__"


######################################################################
# Test cases
######################################################################

class error(Exception):
    pass


def _parse_quoted(s):
    """Strip surrounding double-quotes from a directive argument."""
    s = s.strip()
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        return s[1:-1]
    return s

class TestCase:
    def __init__(self, fname, dictdir, tempdir, verbose, keepfiles,
                 force_emulator=False, default_fixture=None):
        self.fname = fname
        self.dictdir = dictdir
        self.tempdir = tempdir
        self.verbose = verbose
        self.keepfiles = keepfiles
        self.force_emulator = force_emulator
        self.default_fixture = default_fixture
    def relpath(self, fname, rel='test'):
        if rel == 'dict':
            reldir = self.dictdir
        elif rel == 'temp':
            reldir = self.tempdir
        else:
            reldir = os.path.dirname(self.fname)
        return os.path.join(reldir, fname)
    def parse_test(self):
        # Parse file into test cases
        config_fname = gcode_fname = dict_fnames = None
        emulator_fixture = None
        log_required = []
        log_forbidden = []
        should_fail = multi_tests = allow_shutdown = False
        gcode = []
        f = open(self.fname, 'r')
        for raw_line in f:
            cpos = raw_line.find('#')
            if cpos >= 0:
                line = raw_line[:cpos]
            else:
                line = raw_line
            parts = line.strip().split(None, 1)
            if not parts:
                continue
            if parts[0] == "CONFIG":
                if config_fname is not None:
                    # Multiple tests in same file
                    if not multi_tests:
                        multi_tests = True
                        self.launch_test(config_fname, dict_fnames,
                                         gcode_fname, gcode, should_fail,
                                         emulator_fixture, log_required,
                                         log_forbidden)
                config_fname = self.relpath(parts[1].strip())
                if multi_tests:
                    self.launch_test(config_fname, dict_fnames,
                                     gcode_fname, gcode, should_fail,
                                     emulator_fixture, log_required,
                                     log_forbidden)
            elif parts[0] == "DICTIONARY":
                dict_args = parts[1].split() if len(parts) > 1 else []
                dict_fnames = [self.relpath(dict_args[0], 'dict')]
                for mcu_dict in dict_args[1:]:
                    mcu, fname = mcu_dict.split('=', 1)
                    dict_fnames.append('%s=%s' % (
                        mcu.strip(), self.relpath(fname.strip(), 'dict')))
            elif parts[0] == "GCODE":
                gcode_fname = self.relpath(parts[1].strip())
            elif parts[0] == "EMULATOR":
                emulator_fixture = self.relpath(parts[1].strip())
            elif parts[0] == "EXPECT_LOG_CONTAINS":
                log_required.append(_parse_quoted(parts[1]))
            elif parts[0] == "EXPECT_LOG_NOT_CONTAINS":
                log_forbidden.append(_parse_quoted(parts[1]))
            elif parts[0] == "SHOULD_FAIL":
                should_fail = True
            elif parts[0] == "ALLOW_SHUTDOWN":
                allow_shutdown = True
            else:
                gcode.append(raw_line.strip())
        f.close()
        if (self.force_emulator and emulator_fixture is None
                and self.default_fixture is not None):
            emulator_fixture = self.default_fixture
        if self.force_emulator and should_fail:
            # SHOULD_FAIL tests expect klippy to error out for
            # config-loading reasons. Their failure path is
            # orthogonal to whether the MCU is real or emulated, and
            # in emulator mode the fileoutput-specific timing can
            # change which error fires first. Skip them.
            sys.stderr.write(
                "    Skipping %s (SHOULD_FAIL incompatible with "
                "--force-emulator)\n" % (self.fname,))
            return
        if not multi_tests:
            self.launch_test(config_fname, dict_fnames, gcode_fname, gcode,
                             should_fail, emulator_fixture, log_required,
                             log_forbidden, allow_shutdown)
    def launch_test(self, config_fname, dict_fnames, gcode_fname, gcode,
                    should_fail, emulator_fixture=None, log_required=None,
                    log_forbidden=None, allow_shutdown=False):
        # Under --force-emulator, skip subtests whose dict file isn't
        # present in dictdir. printers.test iterates ~30 MCU configs
        # and the emulator-test Docker image only builds the AVR
        # variants - without this skip the first missing dict aborts
        # the whole test case. Regular CI runs build every dict, so
        # this only affects the emulator-only sweep.
        if self.force_emulator and dict_fnames:
            for df in dict_fnames:
                path = df.split('=', 1)[1] if '=' in df else df
                if not os.path.exists(path):
                    sys.stderr.write(
                        "    Skipping %s (%s) - dict %r not built\n"
                        % (self.fname, os.path.basename(config_fname),
                           os.path.basename(path)))
                    return
        gcode_is_temp = False
        if gcode_fname is None:
            gcode_fname = self.relpath(TEMP_GCODE_FILE, 'temp')
            gcode_is_temp = True
            f = open(gcode_fname, 'w')
            f.write('\n'.join(gcode + ['']))
            f.close()
        elif gcode:
            raise error("Can't specify both a gcode file and gcode commands")
        if config_fname is None:
            raise error("config file not specified")
        if dict_fnames is None:
            raise error("data dictionary file not specified")
        sys.stderr.write("    Starting %s (%s)\n" % (
            self.fname, os.path.basename(config_fname)))
        if emulator_fixture is not None:
            res, log_path = self._launch_emulator_test(
                config_fname, dict_fnames, gcode_fname, emulator_fixture)
        else:
            res, log_path = self._launch_fileoutput_test(
                config_fname, dict_fnames, gcode_fname)
        is_fail = (should_fail and not res) or (not should_fail and res)
        # Emulator-mode tests by default surface silent klippy shutdowns
        # as failures - the runner's exit-code check alone misses them
        # because klippy logs "Transition to shutdown state" then exits
        # cleanly. Tests that legitimately exercise a shutdown path
        # opt out via ALLOW_SHUTDOWN.
        effective_forbidden = list(log_forbidden or ())
        if (emulator_fixture is not None and not should_fail
                and not allow_shutdown):
            effective_forbidden.extend([
                r'Transition to shutdown state',
                r'Klippy is shutdown',
                r'Internal error',
            ])
        if not is_fail and (log_required or effective_forbidden):
            mismatch = self._check_log(log_path, log_required,
                                       effective_forbidden)
            if mismatch is not None:
                is_fail = True
                sys.stderr.write("    Log assertion failure: %s\n"
                                 % (mismatch,))
        if is_fail:
            if not self.verbose:
                self.show_log()
            if should_fail:
                raise error("Test failed to raise an error")
            raise error("Error during test")
        # Do cleanup
        if self.keepfiles:
            return
        for fname in os.listdir(self.tempdir):
            if (fname.startswith(TEMP_OUTPUT_FILE)
                    or fname.startswith(TEMP_EMU_LINK)
                    or fname == TEMP_EMU_CFG
                    or fname == TEMP_EMU_LOG):
                try:
                    os.unlink(os.path.join(self.tempdir, fname))
                except OSError:
                    pass
        if not self.verbose:
            try:
                os.unlink(TEMP_LOG_FILE)
            except OSError:
                pass
        else:
            sys.stderr.write('\n')
        if gcode_is_temp:
            os.unlink(gcode_fname)

    # ------------------------------------------------------------------
    # File-output (legacy) test launcher
    # ------------------------------------------------------------------

    def _launch_fileoutput_test(self, config_fname, dict_fnames, gcode_fname):
        args = [sys.executable, './klippy/klippy.py', config_fname,
                '-i', gcode_fname, '-o', TEMP_OUTPUT_FILE, '-v']
        for df in dict_fnames:
            args += ['-d', df]
        if not self.verbose:
            args += ['-l', TEMP_LOG_FILE]
        res = subprocess.call(args)
        return res, TEMP_LOG_FILE

    # ------------------------------------------------------------------
    # Emulator-driven test launcher
    # ------------------------------------------------------------------

    def _launch_emulator_test(self, config_fname, dict_fnames, gcode_fname,
                              fixture_path):
        if len(dict_fnames) != 1:
            raise error("EMULATOR mode currently supports a single MCU dict")
        dict_path = dict_fnames[0]
        if '=' in dict_path:
            dict_path = dict_path.split('=', 1)[1]
        repo_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), os.pardir))
        slave_link = os.path.join(self.tempdir, TEMP_EMU_LINK)
        emu_log = os.path.join(self.tempdir, TEMP_EMU_LOG)
        cfg_path = os.path.join(self.tempdir, TEMP_EMU_CFG)
        if os.path.exists(slave_link):
            os.unlink(slave_link)
        # Prefer the simavr-based bridge when an .elf for this MCU
        # The simavr bridge runs the actual klipper firmware ELF
        # under cycle-accurate AVR simulation - every endstop sample
        # loop, trsync timer, and software-PWM pulse runs the same
        # C code that ships on real hardware. The fixture is
        # translated into bridge control-socket commands that drive
        # ADC values, GPIO pins, SPI/I2C MISO queues, step-edge
        # triggers, and BLTouch state.
        elf_path = self._find_elf_for_dict(dict_path)
        bridge_path = os.path.join(repo_root, 'ci_build', 'simavr_bridge')
        if elf_path is None or not os.path.isfile(bridge_path) \
                or not os.access(bridge_path, os.X_OK):
            raise error(
                "simavr bridge or .elf missing - this build of "
                "test_klippy.py requires both. Rebuild with "
                "scripts/Dockerfile.emulator-test or build the bridge "
                "manually with gcc against libsimavr.")
        ctl_socket = os.path.join(self.tempdir, TEMP_EMU_CTL)
        try:
            os.unlink(ctl_socket)
        except OSError:
            pass
        emu_args = [
            bridge_path,
            '--elf', elf_path,
            '--slave-link', slave_link,
            '--control-socket', ctl_socket,
            '--duration', str(EMULATOR_KLIPPY_DEADLINE + 5),
        ]
        emu_log_fd = open(emu_log, 'w')
        emu_proc = subprocess.Popen(emu_args, cwd=repo_root,
                                    stdout=emu_log_fd,
                                    stderr=subprocess.STDOUT)
        try:
            slave_path = self._wait_for_slave_link(slave_link, emu_proc)
            if ctl_socket is not None:
                self._push_fixture_to_control_socket(
                    ctl_socket, fixture_path, config_fname)
            self._materialize_emulator_config(config_fname, cfg_path,
                                              slave_path)
            klippy_args = [sys.executable, './klippy/klippy.py', cfg_path,
                           '-i', gcode_fname, '-l', TEMP_LOG_FILE, '-v']
            for df in dict_fnames:
                klippy_args += ['-d', df]
            res = self._run_klippy_with_deadline(klippy_args)
        finally:
            self._terminate(emu_proc)
            emu_log_fd.close()
        return res, TEMP_LOG_FILE

    _STEPPER_RE = re.compile(r'^\[(stepper_[a-z0-9_]+)\]\s*$')
    _PIN_RE = re.compile(r'^\s*([a-z_]+_pin)\s*:\s*([!^~]*)(P[A-L]\d+)\s*'
                         r'(?:#.*)?$')

    @classmethod
    def _parse_stepper_endstops_any(cls, config_fname):
        # Like _parse_stepper_endstops but yields every stepper
        # section (including those with virtual endstops like
        # probe:z_virtual_endstop). Used to find the Z stepper's
        # step_pin for bltouch's step_trigger setup, since Z's
        # endstop_pin in the cfg is `probe:z_virtual_endstop`
        # rather than a plain GPIO.
        steppers = []
        current = None
        try:
            f = open(config_fname)
        except OSError:
            return steppers
        try:
            for line in f:
                m = cls._STEPPER_RE.match(line)
                if m:
                    if current:
                        steppers.append(current)
                    current = {'name': m.group(1)}
                    continue
                if line.strip().startswith('[') and current:
                    steppers.append(current)
                    current = None
                    continue
                if current is None:
                    continue
                m = cls._PIN_RE.match(line)
                if not m:
                    continue
                key, _flags, bare = m.groups()
                if key in ('step_pin', 'endstop_pin'):
                    current[key] = bare
        finally:
            f.close()
        if current:
            steppers.append(current)
        return steppers

    @classmethod
    def _parse_stepper_endstops(cls, config_fname):
        # Yield {stepper_name, step_pin, endstop_pin} for every
        # [stepper_<axis>] section that has both a step_pin and a
        # plain GPIO endstop_pin (skip virtual endstops like
        # `probe:z_virtual_endstop` - those go through the bltouch
        # / probe path which has its own fixture wiring). Pins are
        # returned bare ("PE5") with the `^!~` modifier flags
        # stripped, since the bridge speaks raw simavr GPIO.
        steppers = []
        current = None
        try:
            f = open(config_fname)
        except OSError:
            return steppers
        try:
            for line in f:
                m = cls._STEPPER_RE.match(line)
                if m:
                    if current and 'step_pin' in current \
                            and 'endstop_pin' in current:
                        steppers.append(current)
                    current = {'name': m.group(1)}
                    continue
                if line.strip().startswith('[') and current:
                    if 'step_pin' in current \
                            and 'endstop_pin' in current:
                        steppers.append(current)
                    current = None
                    continue
                if current is None:
                    continue
                m = cls._PIN_RE.match(line)
                if not m:
                    continue
                key, _flags, bare = m.groups()
                if key in ('step_pin', 'endstop_pin'):
                    current[key] = bare
        finally:
            f.close()
        if current and 'step_pin' in current \
                and 'endstop_pin' in current:
            steppers.append(current)
        return steppers

    _EXTRUDER_SECTION_RE = re.compile(r'^\[(extruder\d*)\]\s*$')

    @classmethod
    def _parse_extruder_sensor_pins(cls, config_fname):
        # Walk [extruder] / [extruder1] / [extruder2] / ... sections
        # and yield (section_name, sensor_pin) pairs in declaration
        # order. Used to apply a hot ADC default to extruder pins so
        # tests with extrusion gcode pass min_extrude_temp without
        # per-test fixture scripting.
        out = []
        current = None
        try:
            f = open(config_fname)
        except OSError:
            return out
        try:
            for line in f:
                m = cls._EXTRUDER_SECTION_RE.match(line)
                if m:
                    current = m.group(1)
                    continue
                if line.strip().startswith('['):
                    current = None
                    continue
                if current is None:
                    continue
                m = cls._PIN_RE.match(line)
                if not m:
                    continue
                key, _flags, bare = m.groups()
                if key == 'sensor_pin':
                    out.append((current, bare))
                    current = None
        finally:
            f.close()
        return out

    @staticmethod
    def _adc_channel_for_pin(pin_name):
        # atmega ADC channel layout:
        #   ADC0..ADC7  = PF0..PF7
        #   ADC8..ADC15 = PK0..PK7  (atmega2560 only)
        if not pin_name or len(pin_name) < 3:
            return None
        port = pin_name[1]
        try:
            pin_num = int(pin_name[2:])
        except ValueError:
            return None
        if port == 'F' and 0 <= pin_num <= 7:
            return pin_num
        if port == 'K' and 0 <= pin_num <= 7:
            return 8 + pin_num
        return None

    def _push_fixture_to_control_socket(self, socket_path, fixture_path,
                                        config_fname=None):
        # Translate the JSON fixture into newline-terminated commands
        # the simavr bridge understands and write them through the
        # control socket. Connection retries briefly because the
        # bridge thread spawns the listener concurrently with the
        # main thread's UART setup; the test runner reaches this
        # point shortly after slave_link is published.
        raw = {}
        if fixture_path is not None:
            try:
                with open(fixture_path) as f:
                    raw = json.load(f)
            except (OSError, ValueError):
                pass
        lines = []
        # Translate the fixture's auto_trsync_trigger_ticks into
        # bridge step_trigger commands, one per stepper that has a
        # real (non-virtual) endstop pin. Each home_start by klippy
        # advances its stepper's step pin; once the configured count
        # is reached the bridge drives the endstop pin to triggered
        # so klippy's home loop sees an endstop hit on a wire-level
        # GPIO transition the firmware actually samples.
        auto_ticks = raw.get('auto_trsync_trigger_ticks')
        if auto_ticks is not None and config_fname is not None:
            for stepper in self._parse_stepper_endstops(config_fname):
                step_p = stepper['step_pin']
                end_p = stepper['endstop_pin']
                # 100 steps is well below any homing move's full
                # range; the home succeeds long before klippy's
                # home_wait timeout. Triggered = pin_value=1 (klippy
                # XORs with the `!` invert flag from the cfg).
                lines.append("step_trigger %s %d 100 %s %d 1" % (
                    step_p[1], int(step_p[2:]),
                    end_p[1], int(end_p[2:])))
        # bltouch + auto_trigger_after_steps: configure the bridge's
        # BLTouch state machine on the [bltouch] control/sensor pins,
        # AND wire a step_trigger from the Z stepper (the one driving
        # the probe-move stepper) to the sensor pin so the firmware's
        # endstop sample loop sees a "touch" after the configured
        # number of stepper edges. Together these let multi-sample
        # probe tests run against real klipper firmware via simavr.
        bltouch = raw.get('bltouch')
        if bltouch and config_fname is not None:
            ctrl = bltouch.get('control_pin', '')
            sens = bltouch.get('sensor_pin', '')
            inv = 1 if bltouch.get('invert', False) else 0
            if (len(ctrl) >= 3 and ctrl[0] == 'P'
                    and len(sens) >= 3 and sens[0] == 'P'):
                lines.append("bltouch %s %d %s %d %d" % (
                    ctrl[1], int(ctrl[2:]),
                    sens[1], int(sens[2:]), inv))
                ats = raw.get('auto_trigger_after_steps')
                if ats is not None:
                    # Find the Z stepper's step_pin (the one whose
                    # endstop is the bltouch sensor, which we know
                    # comes via probe:z_virtual_endstop). For the
                    # current test fixtures we just use the first
                    # stepper_z section.
                    z_step_pin = None
                    for s in self._parse_stepper_endstops_any(
                            config_fname):
                        if s['name'] == 'stepper_z' and 'step_pin' in s:
                            z_step_pin = s['step_pin']
                            break
                    if z_step_pin is not None:
                        lines.append("step_trigger %s %d %d %s %d 1" % (
                            z_step_pin[1], int(z_step_pin[2:]),
                            int(ats),
                            sens[1], int(sens[2:])))
        adc_default = raw.get('analog_in_default', {})

        def _raw_to_mv(raw_value):
            # Fixture ADC values are raw oversampled 13-bit readings
            # (klipper sums 8 samples of 10-bit ADC by default, max
            # ~8184). simavr's ADC model speaks millivolts at VCC=5V
            # so round-trip: raw / 8184 ~= mv / 5000.
            return int(raw_value * 5000 / 8184)

        if 'default_value' in adc_default:
            mv = _raw_to_mv(adc_default['default_value'])
            for ch in range(16):
                lines.append("adc %d %d" % (ch, mv))
        # by_pin overrides for specific physical ADC pins. atmega
        # ADC channel mapping: ADC0..ADC7 = PF0..PF7, ADC8..ADC15 =
        # PK0..PK7 (atmega2560 only - the smaller AVRs cap at ADC7).
        for pin_name, raw_value in adc_default.get('by_pin', {}).items():
            ch = self._adc_channel_for_pin(pin_name)
            if ch is None:
                continue
            lines.append("adc %d %d" % (ch, _raw_to_mv(raw_value)))
        # _hot_extruder_N entries in the fixture's analog_in block
        # apply a hot ADC value (default ~190 C with EPCOS 100K) to
        # the Nth [extruder*] section's sensor_pin in the cfg, so
        # tests with extrusion gcode get past klippy's min_extrude_temp
        # without per-test fixture scripting. N is the section index
        # in declaration order: extruder=0, extruder1=1, etc.
        analog_in = raw.get('analog_in', {})
        if analog_in and config_fname is not None:
            extruder_pins = self._parse_extruder_sensor_pins(config_fname)
            for label, spec in analog_in.items():
                if not label.startswith('_hot_extruder_'):
                    continue
                try:
                    idx = int(label[len('_hot_extruder_'):])
                except ValueError:
                    continue
                if idx < 0 or idx >= len(extruder_pins):
                    continue
                _name, pin = extruder_pins[idx]
                ch = self._adc_channel_for_pin(pin)
                if ch is None:
                    continue
                rv = spec.get('default_value')
                if rv is None:
                    continue
                lines.append("adc %d %d" % (ch, _raw_to_mv(rv)))
        # spi_response: a hex byte stream the bridge round-robins
        # back to klipper as MISO data. Tests with thermocouples or
        # similar SPI-resident sensors use this to keep the firmware
        # from tripping its range checks.
        spi_hex = raw.get('spi_response')
        if spi_hex:
            lines.append("spi %s" % spi_hex.strip())
        # i2c reads: concatenate per-slave read sequences in fixture
        # order, push as a single bridge i2c queue. The bridge
        # auto-ACKs addressing/writes and serves reads round-robin
        # from the queue; klippy's I2C drivers see the bytes their
        # peripherals would return on the wire.
        i2c_specs = raw.get('i2c', {})
        i2c_bytes = []
        for label, spec in i2c_specs.items():
            for read in spec.get('reads', []):
                i2c_bytes.extend(int(b) & 0xff for b in read)
        # i2c_default's register_responses are typically chip-id
        # reads klippy probes once at startup. Inject those next so
        # they answer the first i2c reads if a label-specific list
        # is empty.
        for reg_str, payload in (raw.get('i2c_default', {})
                                 .get('register_responses', {}).items()):
            i2c_bytes.extend(int(b) & 0xff for b in payload)
        if i2c_bytes:
            lines.append("i2c " + ''.join("%02x" % b for b in i2c_bytes))
        # Connect with a brief retry.
        deadline = _monotonic() + 2.0
        sock = None
        last_err = None
        while _monotonic() < deadline:
            try:
                import socket as _s
                sock = _s.socket(_s.AF_UNIX, _s.SOCK_STREAM)
                sock.connect(socket_path)
                break
            except OSError as e:
                last_err = e
                if sock is not None:
                    sock.close()
                    sock = None
                time.sleep(0.05)
        if sock is None:
            sys.stderr.write("    WARN: control socket %s unreachable: %s\n"
                             % (socket_path, last_err))
            return
        try:
            for line in lines:
                sock.sendall((line + '\n').encode('ascii'))
            # Send a barrier and wait for the bridge's "OK" so the
            # IRQ events we just queued have been dispatched in
            # simulated time before we let klippy connect. Bound the
            # wall-clock wait to keep a stuck bridge from hanging
            # CI - 5 s is far longer than 2 ms of simulated time
            # ever takes on a working host.
            try:
                sock.sendall(b'barrier 200000\n')
                sock.settimeout(5.0)
                ack = b''
                while b'\n' not in ack and len(ack) < 16:
                    chunk = sock.recv(16 - len(ack))
                    if not chunk:
                        break
                    ack += chunk
            except (OSError, socket.timeout):
                pass
        finally:
            sock.close()

    def _find_elf_for_dict(self, dict_path):
        # The Dockerfile builds a parallel ci_build/elf/<mcu>.elf
        # alongside ci_build/dict/<mcu>.dict; we use the dict's
        # basename to locate the matching firmware ELF.
        dict_dir = os.path.dirname(dict_path)
        base = os.path.basename(dict_path)
        if not base.endswith('.dict'):
            return None
        mcu_name = base[:-len('.dict')]
        candidates = [
            os.path.join(dict_dir, '..', 'elf', mcu_name + '.elf'),
            os.path.join(os.path.dirname(dict_dir), 'elf',
                         mcu_name + '.elf'),
        ]
        for c in candidates:
            c = os.path.normpath(c)
            if os.path.isfile(c):
                return c
        return None

    def _wait_for_slave_link(self, link_path, emu_proc, timeout=10.0):
        deadline = _monotonic() + timeout
        while _monotonic() < deadline:
            if emu_proc.poll() is not None:
                raise error("emulator exited before publishing slave link "
                            "(returncode=%d)" % (emu_proc.returncode,))
            if os.path.exists(link_path):
                with open(link_path) as f:
                    return f.read().strip()
            time.sleep(0.05)
        raise error("emulator did not publish slave link within %.1fs"
                    % (timeout,))

    def _materialize_emulator_config(self, src_path, dest_path, slave_path):
        with open(src_path) as f:
            cfg = f.read()
        if SERIAL_PLACEHOLDER in cfg:
            cfg = cfg.replace(SERIAL_PLACEHOLDER, slave_path)
        else:
            # No explicit placeholder: rewrite every "serial: ..." line
            # to point at the emulator. This lets existing .cfg files
            # be reused with `EMULATOR` without modification.
            cfg, n = re.subn(r'(?m)^(\s*serial\s*:\s*).*$',
                             r'\1' + slave_path, cfg)
            if n == 0:
                raise error(
                    "EMULATOR config %r has no serial: line and no %r "
                    "placeholder; nothing to substitute"
                    % (src_path, SERIAL_PLACEHOLDER))
        with open(dest_path, 'w') as f:
            f.write(cfg)

    def _run_klippy_with_deadline(self, args):
        proc = subprocess.Popen(args)
        try:
            return proc.wait(timeout=EMULATOR_KLIPPY_DEADLINE)
        except subprocess.TimeoutExpired:
            # Klippy in real-mcu mode does not exit on gcode EOF. We
            # treat reaching the deadline without a crash as success;
            # the caller will run log assertions to verify behaviour.
            self._terminate(proc)
            return 0

    @staticmethod
    def _terminate(proc):
        if proc.poll() is not None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)

    # ------------------------------------------------------------------
    # Log assertions
    # ------------------------------------------------------------------

    def _check_log(self, log_path, log_required, log_forbidden):
        try:
            with open(log_path) as f:
                content = f.read()
        except (OSError, IOError) as e:
            return "could not read log %r: %s" % (log_path, e)
        for pat in log_required or ():
            if not re.search(pat, content):
                return "EXPECT_LOG_CONTAINS pattern not found: %r" % (pat,)
        for pat in log_forbidden or ():
            if re.search(pat, content):
                return "EXPECT_LOG_NOT_CONTAINS pattern found: %r" % (pat,)
        return None
    def run(self):
        try:
            self.parse_test()
        except error as e:
            return str(e)
        except Exception:
            logging.exception("Unhandled exception during test run")
            return "internal error"
        return "success"
    def show_log(self):
        f = open(TEMP_LOG_FILE, 'r')
        data = f.read()
        f.close()
        sys.stdout.write(data)


######################################################################
# Startup
######################################################################

def main():
    # Parse args
    usage = "%prog [options] <test cases>"
    opts = optparse.OptionParser(usage)
    opts.add_option("-d", "--dictdir", dest="dictdir", default=".",
                    help="directory for dictionary files")
    opts.add_option("-t", "--tempdir", dest="tempdir", default=".",
                    help="directory for temporary files")
    opts.add_option("-k", action="store_true", dest="keepfiles",
                    help="do not remove temporary files")
    opts.add_option("-v", action="store_true", dest="verbose",
                    help="show all output from tests")
    opts.add_option("--force-emulator", action="store_true",
                    dest="force_emulator",
                    help="run every test in emulator mode (auto-injects "
                         "EMULATOR with an empty fixture if not already "
                         "set). Used to find which tests still need real "
                         "emulator support during the fileoutput "
                         "deprecation effort.")
    opts.add_option("--emulator-fixture-default",
                    dest="default_fixture",
                    help="path to an empty/default fixture JSON used "
                         "when --force-emulator injects EMULATOR")
    options, args = opts.parse_args()
    if len(args) < 1:
        opts.error("Incorrect number of arguments")
    logging.basicConfig(level=logging.DEBUG)

    # Run each test
    failures = []
    for fname in args:
        tc = TestCase(fname, options.dictdir, options.tempdir, options.verbose,
                      options.keepfiles,
                      force_emulator=options.force_emulator,
                      default_fixture=options.default_fixture)
        res = tc.run()
        if res != 'success':
            sys.stderr.write("\n\nTest case %s FAILED (%s)!\n\n"
                             % (fname, res))
            if options.force_emulator:
                failures.append(fname)
            else:
                sys.exit(-1)

    if options.force_emulator:
        sys.stderr.write("\n    %d/%d emulator-mode runs passed\n"
                         % (len(args) - len(failures), len(args)))
        if failures:
            sys.stderr.write("    Failures:\n")
            for f in failures:
                sys.stderr.write("      - %s\n" % (f,))
            sys.exit(1)
    else:
        sys.stderr.write("\n    All %d test cases passed\n" % (len(args),))

if __name__ == '__main__':
    main()
