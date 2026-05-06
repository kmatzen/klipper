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
        prepend = []
        if emulator_fixture is not None:
            try:
                with open(emulator_fixture) as ff:
                    raw = json.load(ff)
                prepend = list(raw.get('prepend_gcode') or ())
            except (OSError, ValueError):
                prepend = []
        if gcode_fname is None:
            gcode_fname = self.relpath(TEMP_GCODE_FILE, 'temp')
            gcode_is_temp = True
            f = open(gcode_fname, 'w')
            f.write('\n'.join(prepend + gcode + ['']))
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
        # `DICTIONARY a.dict mcu1=b.dict ...` parses into a list where the
        # first entry is the default `[mcu]` and the rest are
        # `name=path` for `[mcu name]` sections. We map each entry to its
        # own simavr bridge (separate ELF, pty, sockets) so multi-MCU
        # configs spin up one bridge per MCU and klippy connects to each
        # MCU section's pty independently.
        mcu_dicts = []  # [(mcu_name, dict_path), ...]
        for i, df in enumerate(dict_fnames):
            if '=' in df:
                mcu_name, dict_path = df.split('=', 1)
                mcu_dicts.append((mcu_name.strip(), dict_path))
            else:
                mcu_dicts.append(('mcu', df))
        repo_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), os.pardir))
        cfg_path = os.path.join(self.tempdir, TEMP_EMU_CFG)
        emu_log = os.path.join(self.tempdir, TEMP_EMU_LOG)
        bridge_path = os.path.join(repo_root, 'ci_build', 'simavr_bridge')
        renode_launcher = os.path.join(repo_root, 'test', 'emulator',
                                       'renode_launcher.py')
        # Per-MCU paths suffix the mcu name (default `mcu` keeps the
        # legacy `_test_emu.tty` filename so single-MCU tests are
        # byte-identical to the pre-fan-out path).
        def mcu_suffix(name):
            return '' if name == 'mcu' else '_' + name
        bridges = []  # list of dicts, one per MCU
        any_simavr = False
        any_renode = False
        for mcu_name, dict_path in mcu_dicts:
            elf_path = self._find_elf_for_dict(dict_path)
            if elf_path is None:
                raise error(
                    "EMULATOR mode: no .elf alongside dict %r for [mcu %s]"
                    % (dict_path, mcu_name))
            backend = self._backend_for_dict(dict_path)
            if backend == 'simavr':
                any_simavr = True
            elif backend == 'renode':
                any_renode = True
            sfx = mcu_suffix(mcu_name)
            slave_link = os.path.join(self.tempdir,
                                      TEMP_EMU_LINK[:-len('.tty')]
                                      + sfx + '.tty')
            ctl_socket = os.path.join(self.tempdir,
                                      '_test_emu' + sfx + '.ctl')
            for p in (slave_link, ctl_socket):
                try:
                    os.unlink(p)
                except OSError:
                    pass
            bridges.append({
                'mcu': mcu_name,
                'backend': backend,
                'elf': elf_path,
                'slave_link': slave_link,
                'ctl_socket': ctl_socket,
                'sfx': sfx,
            })
        if any_simavr and (not os.path.isfile(bridge_path)
                           or not os.access(bridge_path, os.X_OK)):
            raise error(
                "simavr bridge or .elf missing - this build of "
                "test_klippy.py requires both. Rebuild with "
                "scripts/Dockerfile.emulator-test or build the bridge "
                "manually with gcc against libsimavr.")
        if any_renode and not os.path.isfile(renode_launcher):
            raise error(
                "renode launcher missing - this build of test_klippy.py "
                "requires test/emulator/renode_launcher.py for stm32 "
                "dicts. Rebuild with scripts/Dockerfile.emulator-test "
                "(which installs renode and the launcher).")
        # Opt-in sim-time mode: tests with `sim_time: true` in their
        # fixture get a deterministic-time runtime where klippy reads
        # MCU clock via a memory-mapped double instead of
        # clock_gettime, and simavr free-runs without wall-clock
        # throttling. This eliminates the "rescheduled timer in past"
        # class of host-load flakiness, but disables the wall-clock
        # safety net that previously masked heater_verify timeouts in
        # tests that set heater targets, so it must be opt-in until
        # the bridge models heater PWM -> ADC heat-up.
        sim_time_enabled = False
        tick_mode_enabled = False
        try:
            if fixture_path is not None:
                with open(fixture_path) as ff:
                    fx = json.load(ff)
                sim_time_enabled = bool(fx.get('sim_time'))
                tick_mode_enabled = bool(fx.get('tick_mode'))
        except (OSError, ValueError):
            sim_time_enabled = False
            tick_mode_enabled = False
        # tick_mode implies sim_time (klippy reads the MCU's cycle as
        # its monotonic clock); the bridge then drives klippy in
        # lockstep over the tick socket so the two clocks can't drift.
        if tick_mode_enabled:
            sim_time_enabled = True
        # Per-bridge sim_time and tick_socket files. klippy can only
        # read one canonical sim_time mmap (set via KLIPPY_SIM_TIME_FILE),
        # so the first bridge's file is authoritative -- under tick mode
        # all bridges advance to the same target each round-trip, so
        # they stay sync'd to within a single tick anyway.
        canonical_sim_time_file = None
        tick_socket_paths = []
        for b in bridges:
            sfx = b['sfx']
            if b['backend'] == 'linuxprocess':
                # linuxprocess: the linux klipper.elf is itself the MCU.
                # It opens a pty via openpty() and symlinks the slave end
                # to the path passed via -I, so klippy connects directly
                # without any simavr bridge in between. Sim-time / tick
                # mode don't apply (real-time host clock is the MCU
                # clock); per-test fixtures plumb peripheral state via
                # filesystem mocks (e.g. KLIPPER_W1_DEVICES_PATH for
                # DS18B20) rather than a control socket.
                b['args'] = [b['elf'], '-I', b['slave_link']]
                continue
            if b['backend'] == 'renode':
                # renode_launcher.py wraps `renode` with the per-chip
                # platform script and exposes the same interface as
                # simavr_bridge: --slave-link is a pty symlink klippy
                # connects to, --control-socket accepts the same
                # newline-terminated fixture commands that
                # _push_fixture_to_control_socket emits. Sim-time and
                # tick-mode are not yet wired through Renode (real-time
                # only on first cut); the flags are accepted-and-ignored
                # by the launcher so the dispatch above can stay
                # uniform. --fixture-file lets the launcher self-apply
                # the fixture's analog_in / i2c_default keys (which
                # simavr_bridge.c handles internally from the file
                # rather than over the control socket).
                renode_args = [
                    sys.executable, renode_launcher,
                    '--elf', b['elf'],
                    '--slave-link', b['slave_link'],
                    '--control-socket', b['ctl_socket'],
                    '--duration',
                    str(EMULATOR_KLIPPY_DEADLINE + 5),
                ]
                if fixture_path is not None:
                    renode_args += ['--fixture-file', fixture_path]
                b['args'] = renode_args
                continue
            args = [
                bridge_path,
                '--elf', b['elf'],
                '--slave-link', b['slave_link'],
                '--control-socket', b['ctl_socket'],
                '--duration', str(EMULATOR_KLIPPY_DEADLINE + 5),
            ]
            if sim_time_enabled:
                stf = os.path.join(self.tempdir, 'sim_time' + sfx)
                try:
                    os.unlink(stf)
                except OSError:
                    pass
                args += ['--sim-time-file', stf]
                if canonical_sim_time_file is None:
                    canonical_sim_time_file = stf
            if tick_mode_enabled:
                tsp = os.path.join(self.tempdir, 'tick_sock' + sfx)
                try:
                    os.unlink(tsp)
                except OSError:
                    pass
                args += ['--tick-socket', tsp]
                tick_socket_paths.append(tsp)
            b['args'] = args
        # One log file per bridge so concurrent stderr from multiple
        # simavr instances doesn't interleave (each fd has its own
        # write position).
        emu_log_fds = []
        emu_procs = []
        try:
            for b in bridges:
                if len(bridges) == 1:
                    log_path = emu_log
                else:
                    log_path = os.path.join(self.tempdir,
                                            '_test_emu' + b['sfx'] + '.log')
                fd = open(log_path, 'w')
                emu_log_fds.append(fd)
                proc_env = None
                if b['backend'] == 'linuxprocess':
                    # Pre-create w1_slave mock files for any DS18B20
                    # sensors in the cfg, then point the binary at
                    # that tempdir so its sysfs reads succeed without
                    # /sys being writable.
                    w1_dir = self._setup_w1_mocks(config_fname,
                                                  fixture_path, b['sfx'])
                    if w1_dir is not None:
                        proc_env = dict(os.environ)
                        proc_env['KLIPPER_W1_DEVICES_PATH'] = w1_dir
                emu_procs.append(subprocess.Popen(b['args'], cwd=repo_root,
                                                  stdout=fd,
                                                  stderr=subprocess.STDOUT,
                                                  env=proc_env))
            for p, b in zip(emu_procs, bridges):
                is_lp = (b['backend'] == 'linuxprocess')
                # renode publishes the slave end via
                # `emulation CreateUartPtyTerminal` followed by a
                # symlink, so it uses the symlink-readiness branch like
                # linuxprocess. simavr writes the slave dev path into
                # slave_link as a plain file, so it falls through to
                # the original poll branch.
                use_symlink = is_lp or (b['backend'] == 'renode')
                # renode startup includes loading the platform .repl
                # (which downloads + decompresses the SVD on first use),
                # the firmware ELF, and several Monitor commands - so
                # it's substantially slower than simavr. Allow 60 s
                # for the pty to appear; simavr / linuxprocess keep
                # the default snappier deadline so genuine bridge
                # hangs still surface quickly.
                wait_timeout = 60.0 if b['backend'] == 'renode' else 10.0
                b['slave_path'] = self._wait_for_slave_link(
                    b['slave_link'], p, timeout=wait_timeout,
                    is_symlink=use_symlink)
                if is_lp:
                    # No control socket for linuxprocess - peripheral
                    # state is staged via filesystem mocks before spawn.
                    continue
                self._push_fixture_to_control_socket(
                    b['ctl_socket'], fixture_path, config_fname,
                    sim_time_enabled=sim_time_enabled,
                    mcu_name=b['mcu'])
            self._materialize_emulator_config(config_fname, cfg_path,
                                              bridges)
            klippy_args = [sys.executable, './klippy/klippy.py', cfg_path,
                           '-i', gcode_fname, '-l', TEMP_LOG_FILE, '-v']
            for df in dict_fnames:
                klippy_args += ['-d', df]
            klippy_env = None
            if canonical_sim_time_file or tick_socket_paths:
                klippy_env = dict(os.environ)
                if canonical_sim_time_file:
                    klippy_env['KLIPPY_SIM_TIME_FILE'] \
                        = canonical_sim_time_file
                if tick_socket_paths:
                    # ":"-separated so the reactor can connect to each
                    # bridge and broadcast advance/done in lockstep.
                    klippy_env['KLIPPY_TICK_SOCKET'] = ':'.join(
                        tick_socket_paths)
            res = self._run_klippy_with_deadline(klippy_args, env=klippy_env)
        finally:
            for p in emu_procs:
                self._terminate(p)
            for fd in emu_log_fds:
                fd.close()
        return res, TEMP_LOG_FILE

    _STEPPER_RE = re.compile(r'^\[(stepper_[a-z0-9_]+)\]\s*$')
    # Pins may be MCU-prefixed in multi-MCU configs (`zboard:PL3`); the
    # optional `(?:(\w+):)?` group captures that prefix so the fixture
    # pusher can route step_trigger commands to the bridge that owns
    # each pin. Bare pins fall through to the default `[mcu]` section.
    _PIN_RE = re.compile(r'^\s*([a-z_]+_pin)\s*:\s*([!^~]*)'
                         r'(?:([a-z_][a-z0-9_]*):)?(P[A-L]\d+)\s*'
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
                key, _flags, mcu, bare = m.groups()
                if key in ('step_pin', 'endstop_pin'):
                    current[key] = bare
                    current[key + '_mcu'] = mcu or 'mcu'
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
                key, _flags, mcu, bare = m.groups()
                if key in ('step_pin', 'endstop_pin'):
                    current[key] = bare
                    current[key + '_mcu'] = mcu or 'mcu'
        finally:
            f.close()
        if current and 'step_pin' in current \
                and 'endstop_pin' in current:
            steppers.append(current)
        return steppers

    _EXTRUDER_SECTION_RE = re.compile(r'^\[(extruder\d*)\]\s*$')
    _PROBE_PIN_RE = re.compile(r'^\s*pin\s*:\s*([!^~]*)(P[A-L]\d+)\s*'
                               r'(?:#.*)?$')
    _SW_I2C_RE = re.compile(
        r'^\s*i2c_software_(scl|sda)_pin\s*:\s*([!^~]*)(P[A-L]\d+)\s*'
        r'(?:#.*)?$')
    _TMC_UART_SECTION_RE = re.compile(
        r'^\[(tmc220[89])\s+\S+\]\s*$')
    _TMC_UART_PIN_RE = re.compile(
        r'^\s*uart_pin\s*:\s*([!^~]*)(P[A-L]\d+)\s*(?:#.*)?$')
    _TMC2660_SECTION_RE = re.compile(
        r'^\[tmc2660\s+\S+\]\s*$')
    _TMC2660_CS_PIN_RE = re.compile(
        r'^\s*cs_pin\s*:\s*([!^~]*)(P[A-L]\d+)\s*(?:#.*)?$')
    _LOAD_CELL_SECTION_RE = re.compile(
        r'^\[load_cell(?:_probe)?(?:\s+\S+)?\]\s*$')
    _ADS1220_SENSOR_TYPE_RE = re.compile(
        r'^\s*sensor_type\s*:\s*ads1220\s*(?:#.*)?$')
    _ADS1220_CS_PIN_RE = re.compile(
        r'^\s*cs_pin\s*:\s*([!^~]*)(P[A-L]\d+)\s*(?:#.*)?$')
    _ADS1220_DRDY_PIN_RE = re.compile(
        r'^\s*data_ready_pin\s*:\s*([!^~]*)(P[A-L]\d+)\s*(?:#.*)?$')
    _ADS1220_SAMPLE_RATE_RE = re.compile(
        r'^\s*sample_rate\s*:\s*(\d+)\s*(?:#.*)?$')

    @classmethod
    def _parse_tmc_uart_pins(cls, config_fname):
        # Yield uart_pin (e.g. "PA5") for every [tmc2208 ...] / [tmc2209
        # ...] section. Pins are emitted in section-declaration order.
        pins = []
        in_section = False
        try:
            f = open(config_fname)
        except OSError:
            return pins
        try:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('['):
                    in_section = bool(cls._TMC_UART_SECTION_RE.match(stripped))
                    continue
                if not in_section:
                    continue
                m = cls._TMC_UART_PIN_RE.match(line)
                if m:
                    pins.append(m.group(2))
                    in_section = False
        finally:
            f.close()
        return pins

    @classmethod
    def _parse_tmc2660_cs_pins(cls, config_fname):
        # Yield cs_pin (e.g. "PA0") for every [tmc2660 ...] section.
        # The bridge needs to know each TMC2660 chip's CS pin so it
        # can apply the 3-byte / 20-bit datagram protocol while that
        # CS is asserted (vs. the 5-byte default for tmc2130 / 5160 /
        # 2240 chips that may share the same SPI bus).
        pins = []
        in_section = False
        try:
            f = open(config_fname)
        except OSError:
            return pins
        try:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('['):
                    in_section = bool(
                        cls._TMC2660_SECTION_RE.match(stripped))
                    continue
                if not in_section:
                    continue
                m = cls._TMC2660_CS_PIN_RE.match(line)
                if m:
                    pins.append(m.group(2))
                    in_section = False
        finally:
            f.close()
        return pins

    @classmethod
    def _parse_ads1220_chips(cls, config_fname):
        # Yield (cs_pin, drdy_pin, sample_rate) for every [load_cell ...]
        # / [load_cell_probe] section whose sensor_type is ads1220.
        # The bridge needs each chip's CS + DRDY pins so it can pulse
        # DRDY at the chip's configured sample rate (default 660 SPS),
        # replacing the previous "hold DRDY low forever" gpio fixture
        # workaround that overflowed the firmware's wake-task drain.
        chips = []
        in_section = False
        cs = drdy = None
        sensor_is_ads = False
        rate = 660
        try:
            f = open(config_fname)
        except OSError:
            return chips
        try:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('['):
                    if (in_section and sensor_is_ads
                            and cs is not None and drdy is not None):
                        chips.append((cs, drdy, rate))
                    in_section = bool(
                        cls._LOAD_CELL_SECTION_RE.match(stripped))
                    cs = drdy = None
                    sensor_is_ads = False
                    rate = 660
                    continue
                if not in_section:
                    continue
                if cls._ADS1220_SENSOR_TYPE_RE.match(line):
                    sensor_is_ads = True
                    continue
                m = cls._ADS1220_CS_PIN_RE.match(line)
                if m:
                    cs = m.group(2)
                    continue
                m = cls._ADS1220_DRDY_PIN_RE.match(line)
                if m:
                    drdy = m.group(2)
                    continue
                m = cls._ADS1220_SAMPLE_RATE_RE.match(line)
                if m:
                    rate = int(m.group(1))
        finally:
            f.close()
        if (in_section and sensor_is_ads
                and cs is not None and drdy is not None):
            chips.append((cs, drdy, rate))
        return chips

    @classmethod
    def _parse_sw_i2c_pin_pairs(cls, config_fname):
        # Yield (scl_pin, sda_pin) for each section that has matching
        # i2c_software_scl_pin / i2c_software_sda_pin entries. Pairs
        # are emitted in section-declaration order.
        pairs = []
        scl = sda = None
        try:
            f = open(config_fname)
        except OSError:
            return pairs
        try:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('['):
                    if scl and sda:
                        pairs.append((scl, sda))
                    scl = sda = None
                    continue
                m = cls._SW_I2C_RE.match(line)
                if not m:
                    continue
                if m.group(1) == 'scl':
                    scl = m.group(3)
                else:
                    sda = m.group(3)
        finally:
            f.close()
        if scl and sda:
            pairs.append((scl, sda))
        return pairs

    @classmethod
    def _parse_probe_pin(cls, config_fname):
        # Walk a [probe] section and return (flags, bare_pin) for its
        # pin: line. Returns None if no [probe] block or no plain GPIO
        # pin (e.g. virtual endstop). Used to configure a step_trigger
        # on Z step -> probe pin so tests with `[probe]` get a
        # firmware-driven touch trigger after Z motion.
        in_probe = False
        try:
            f = open(config_fname)
        except OSError:
            return None
        try:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('['):
                    in_probe = (stripped == '[probe]')
                    continue
                if not in_probe:
                    continue
                m = cls._PROBE_PIN_RE.match(line)
                if m:
                    return m.group(1), m.group(2)
        finally:
            f.close()
        return None

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
                key, _flags, _mcu, bare = m.groups()
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
                                        config_fname=None,
                                        sim_time_enabled=False,
                                        mcu_name='mcu'):
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
                # Each bridge only sees its own AVR's GPIO IRQs, so
                # step_trigger only makes sense when both pins live on
                # this bridge. Skip steppers whose pins belong to a
                # different MCU section (multi-MCU configs like
                # sample-multi-mcu.cfg with `zboard:PL3` style pins).
                if (stepper.get('step_pin_mcu', 'mcu') != mcu_name
                        or stepper.get('endstop_pin_mcu', 'mcu')
                        != mcu_name):
                    continue
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
        # bltouch fixture pins are bare (assumed on the default `[mcu]`
        # section); a future multi-MCU bltouch fixture can override via
        # `bltouch.mcu`. Skip emitting on bridges that don't own them.
        bltouch_mcu = (bltouch.get('mcu', 'mcu')
                       if isinstance(bltouch, dict) else 'mcu')
        if (bltouch and config_fname is not None
                and bltouch_mcu == mcu_name):
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
                    z_step_mcu = 'mcu'
                    for s in self._parse_stepper_endstops_any(
                            config_fname):
                        if s['name'] == 'stepper_z' and 'step_pin' in s:
                            z_step_pin = s['step_pin']
                            z_step_mcu = s.get('step_pin_mcu', 'mcu')
                            break
                    if z_step_pin is not None and z_step_mcu == mcu_name:
                        lines.append("step_trigger %s %d %d %s %d 1" % (
                            z_step_pin[1], int(z_step_pin[2:]),
                            int(ats),
                            sens[1], int(sens[2:])))
        # probe_pin_after_steps: for tests with a plain `[probe]` (no
        # bltouch state machine), configure step_trigger on the Z
        # stepper's step_pin -> the probe pin after N stepper edges,
        # which drives a firmware-visible "touch" once the toolhead
        # has moved into the bed. Pairs with the bridge's auto-rearm
        # so multi-sample / multi-point probing (bed_mesh, z_tilt,
        # multi_z) works without per-probe re-configuration. The
        # `^pin` invert flag from [probe] determines triggered=1 vs 0.
        probe_steps = raw.get('probe_pin_after_steps')
        if probe_steps is not None and config_fname is not None:
            probe = self._parse_probe_pin(config_fname)
            if probe is not None:
                flags, bare = probe
                # ^ = pull-up + active-low? klippy convention: trigger
                # value is 1 unless `!` invert flag is set.
                trig_val = 0 if '!' in flags else 1
                z_step_pin = None
                z_step_mcu = 'mcu'
                for s in self._parse_stepper_endstops_any(config_fname):
                    if s['name'] == 'stepper_z' and 'step_pin' in s:
                        z_step_pin = s['step_pin']
                        z_step_mcu = s.get('step_pin_mcu', 'mcu')
                        break
                # The probe pin is parsed bare today (_PROBE_PIN_RE
                # doesn't yet handle MCU prefixes); assume it lives
                # on the default `[mcu]` section. Cross-MCU step ->
                # probe wiring would need both pins on the same
                # bridge anyway, so only emit when Z and the probe
                # both belong to this bridge.
                if (z_step_pin is not None
                        and z_step_mcu == mcu_name
                        and mcu_name == 'mcu'):
                    lines.append("step_trigger %s %d %d %s %d %d" % (
                        z_step_pin[1], int(z_step_pin[2:]),
                        int(probe_steps),
                        bare[1], int(bare[2:]), trig_val))
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
        # spi_tmc: switch the bridge's SPI hook into TMC register-file
        # mode. Each 5-byte SPI datagram is decoded as a TMC SPI
        # register access; writes update the register file and reads
        # return the stored value, so klippy's write-then-verify
        # pattern for tmc2130 / tmc5160 / tmc2240 / tmc2660 init
        # succeeds. Mutually exclusive with spi_response.
        if raw.get('spi_tmc'):
            lines.append("spi_tmc")
            # tmc2660 chips on a shared TMC SPI bus need per-chip
            # registration so the bridge can apply its 3-byte (20-bit)
            # datagram protocol while their CS is asserted. Auto-scan
            # the cfg for [tmc2660 ...] sections; each emits a
            # spi_tmc_chip command pointing at the chip's cs_pin.
            if config_fname is not None:
                for cs in self._parse_tmc2660_cs_pins(config_fname):
                    if (len(cs) >= 3 and cs[0] == 'P'
                            and cs[1].isalpha()):
                        lines.append("spi_tmc_chip %s %d tmc2660" % (
                            cs[1], int(cs[2:])))
        # spi_ads1220: switch the bridge's SPI hook into ADS1220
        # register-file mode. Each command is decoded as RREG / WREG /
        # RESET and the bridge maintains a small register file so the
        # ads1220 driver's write-then-verify init pattern succeeds in
        # emulator mode (where MCU.is_fileoutput() is False so the
        # verify mismatch becomes a fatal error). Mutually exclusive
        # with spi_response and spi_tmc.
        if raw.get('spi_ads1220'):
            lines.append("spi_ads1220")
            # Auto-scan the cfg for [load_cell ...] / [load_cell_probe]
            # sections with sensor_type=ads1220 and emit a per-chip
            # spi_ads1220_chip command. The bridge then pulses each
            # chip's DRDY at its configured sample rate (default 660
            # SPS) instead of holding it perpetually low - which the
            # earlier gpio-based approach did and overflowed the
            # firmware's wake-task drain under stepper load.
            if config_fname is not None:
                for cs, drdy, rate in self._parse_ads1220_chips(
                        config_fname):
                    if (len(cs) >= 3 and cs[0] == 'P'
                            and cs[1].isalpha()
                            and len(drdy) >= 3 and drdy[0] == 'P'
                            and drdy[1].isalpha()):
                        lines.append(
                            "spi_ads1220_chip %s %d %s %d %d" % (
                                cs[1], int(cs[2:]),
                                drdy[1], int(drdy[2:]),
                                rate))
        # load_cell_probe_trigger: hook the configured Z step pin and
        # synthesize a ramped ADC sample = (steps_in_burst *
        # force_per_step) raw counts. A step burst starts on the
        # first rising edge after a quiet stretch of reset_us in MCU
        # sim time, so each tare->descent cycle starts from zero and
        # the firmware's drift HPF / buzz LPF / notch SOS filter sees
        # a sustained ramp it can pass through to fire trigger_analog
        # at the configured trigger_force grams. Lets PROBE /
        # BED_MESH_CALIBRATE actually trigger and return in emulator
        # mode where the load_cell_probe driver waits for an analog-
        # trigger trsync that real hardware would close via the load
        # cell flexing under physical contact.
        probe_trig = raw.get('load_cell_probe_trigger')
        if probe_trig:
            step_pin = probe_trig.get('z_step_pin', '')
            reset_us = int(probe_trig.get('reset_quiet_us', 50000))
            force_per_step = int(probe_trig.get('force_per_step', 50))
            if (len(step_pin) >= 3 and step_pin[0] == 'P'
                    and step_pin[1].isalpha()):
                lines.append("probe_step %s %d %d %d" % (
                    step_pin[1], int(step_pin[2:]),
                    reset_us, force_per_step))
        # Software I2C: scan the cfg for any
        # i2c_software_scl_pin / i2c_software_sda_pin pairs and
        # configure the bridge's bit-bang ACK emulator on each. The
        # bridge auto-ACKs every byte on those pins so write-only
        # software-I2C peripherals (e.g. pca9632) succeed.
        if raw.get('sw_i2c_auto_ack', True) and config_fname is not None:
            for scl, sda in self._parse_sw_i2c_pin_pairs(config_fname):
                lines.append("sw_i2c %s %d %s %d" % (
                    scl[1], int(scl[2:]),
                    sda[1], int(sda[2:])))
        # Software UART: scan the cfg for [tmc2208/2209 ...] sections
        # and configure the bridge's UART slave model on each uart_pin.
        # The bridge decodes the bit-banged datagrams (TMC2208/2209
        # half-duplex single-wire UART), services WREG/RREG against a
        # per-chip register file, and pulses the response back over
        # the same wire so klippy's tmc_uart driver succeeds in
        # emulator mode.  bit_time defaults to 1778 cycles
        # (TMC_BAUD_RATE_AVR=9000 baud at 16 MHz).  Override via
        # `sw_uart_bit_time_cycles` in the fixture if needed.
        if raw.get('sw_uart_auto', True) and config_fname is not None:
            bit_time = int(raw.get('sw_uart_bit_time_cycles', 1778))
            for upin in self._parse_tmc_uart_pins(config_fname):
                if (len(upin) >= 3 and upin[0] == 'P'
                        and upin[1].isalpha()):
                    lines.append("sw_uart %s %d %d 0" % (
                        upin[1], int(upin[2:]), bit_time))
        # gpio: list of [port_letter, pin, value] triples. Drives the
        # named GPIO pin to the given level via the bridge's gpio
        # control command. Useful for tests where a sensor's data-
        # ready / interrupt pin needs to be asserted continuously
        # (e.g. LDC1612's intb_pin) so the firmware sees the chip as
        # always having data ready.
        for gpio_cmd in raw.get('gpio', []):
            if isinstance(gpio_cmd, list) and len(gpio_cmd) >= 3:
                port, pin, val = gpio_cmd[0], gpio_cmd[1], gpio_cmd[2]
                lines.append("gpio %s %d %d" % (port, int(pin), int(val)))
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
            # Emit an i2c_reg command for each register so the bridge
            # serves the right bytes when klippy reads that specific
            # register, regardless of read order. Falls back to
            # appending to the flat queue too for backward compat with
            # tests that expect order-dependent reads.
            try:
                reg = int(reg_str, 0)
            except ValueError:
                continue
            byte_strs = ' '.join("%02x" % (int(b) & 0xff) for b in payload)
            lines.append("i2c_reg %02x %s" % (reg, byte_strs))
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
                # In sim-time mode klippy's reactor reads monotonic
                # from simavr's cycle counter, so the moment klippy
                # starts it sees "now" = wherever simavr's clock has
                # advanced to. We pre-run simavr 2 s of simulated
                # time before launching klippy so firmware's ADC
                # peripheral has had time to sample at the
                # by_pin-overridden values - otherwise the combined
                # sensor's 1 s post-ready deviation check can fire
                # before all 3 inputs have updated.
                barrier_us = 2000000 if sim_time_enabled else 500000
                sock.sendall(b'barrier %d\n' % barrier_us)
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

    @staticmethod
    def _backend_for_dict(dict_path):
        # linuxprocess builds yield a host-architecture binary that runs
        # the firmware in-process and exposes its pty directly to klippy
        # - no simavr in the loop. STM32 dicts route through Renode (an
        # external Cortex-M emulator); the renode_launcher.py wrapper
        # presents the same --elf/--slave-link/--control-socket interface
        # as simavr_bridge so the spawn dispatch only differs in which
        # binary is launched. Every other dict (other ARM, pru, ...) is
        # currently routed through simavr; non-AVR/non-STM32 backends
        # remain follow-up work.
        base = os.path.basename(dict_path)
        if base == 'linuxprocess.dict':
            return 'linuxprocess'
        if base.startswith('stm32'):
            return 'renode'
        return 'simavr'

    _DS18B20_SERIAL_RE = re.compile(r'^\s*serial_no\s*:\s*(\S+)\s*'
                                    r'(?:#.*)?$')
    _DS18B20_SENSOR_TYPE_RE = re.compile(r'^\s*sensor_type\s*:\s*'
                                         r'DS18B20\s*(?:#.*)?$')

    @classmethod
    def _parse_ds18b20_serials(cls, config_fname):
        # Walk every section that has both `sensor_type: DS18B20` and a
        # `serial_no:` line and return the configured serial strings.
        # The linux klipper firmware opens /sys/bus/w1/devices/<serial>/
        # w1_slave per sensor; in emulator mode we redirect the prefix
        # to a tempdir and pre-create one mock w1_slave per serial.
        serials = []
        in_section = False
        sensor_is_ds = False
        serial_no = None
        try:
            f = open(config_fname)
        except OSError:
            return serials
        try:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('['):
                    if in_section and sensor_is_ds and serial_no is not None:
                        serials.append(serial_no)
                    in_section = True
                    sensor_is_ds = False
                    serial_no = None
                    continue
                if not in_section:
                    continue
                if cls._DS18B20_SENSOR_TYPE_RE.match(line):
                    sensor_is_ds = True
                    continue
                m = cls._DS18B20_SERIAL_RE.match(line)
                if m:
                    serial_no = m.group(1)
        finally:
            f.close()
        if in_section and sensor_is_ds and serial_no is not None:
            serials.append(serial_no)
        return serials

    def _setup_w1_mocks(self, config_fname, fixture_path, sfx):
        # Pre-create <tempdir>/w1_devices<sfx>/<serial>/w1_slave for every
        # DS18B20 serial declared in the cfg, with a CRC-OK report whose
        # `t=N` is the configured millidegrees C (default 25000 = 25 C,
        # well inside any test's min_temp / max_temp). Returns the root
        # dir to feed into KLIPPER_W1_DEVICES_PATH, or None if there are
        # no DS18B20 sensors (in which case we don't need to set the env
        # var).
        if config_fname is None:
            return None
        serials = self._parse_ds18b20_serials(config_fname)
        if not serials:
            return None
        overrides = {}
        if fixture_path is not None:
            try:
                with open(fixture_path) as ff:
                    fx = json.load(ff)
                overrides = fx.get('w1_devices') or {}
            except (OSError, ValueError):
                overrides = {}
        w1_root = os.path.abspath(
            os.path.join(self.tempdir, 'w1_devices' + sfx))
        for serial in serials:
            spec = overrides.get(serial) or {}
            t_mdeg = int(spec.get('temp_mdeg', 25000))
            dev_dir = os.path.join(w1_root, serial)
            try:
                os.makedirs(dev_dir)
            except OSError:
                pass  # already exists from a prior run; we'll overwrite
            # The reader thread reads up to 128 bytes and looks for the
            # `t=` substring; the leading hex bytes are decorative but
            # match the kernel's w1_therm output format for realism.
            payload = ("31 00 4b 46 7f ff 0c 10 77 : crc=77 YES\n"
                       "31 00 4b 46 7f ff 0c 10 77 t=%d\n" % (t_mdeg,))
            with open(os.path.join(dev_dir, 'w1_slave'), 'w') as f:
                f.write(payload)
        return w1_root

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

    def _wait_for_slave_link(self, link_path, emu_proc, timeout=10.0,
                             is_symlink=False):
        # simavr_bridge writes a regular file containing the pty path;
        # the linuxprocess binary creates a symlink at the path itself.
        # In the symlink case we return the link path - klippy / pyserial
        # will follow it transparently.
        deadline = _monotonic() + timeout
        while _monotonic() < deadline:
            if emu_proc.poll() is not None:
                raise error("emulator exited before publishing slave link "
                            "(returncode=%d)" % (emu_proc.returncode,))
            if is_symlink:
                if os.path.lexists(link_path):
                    return link_path
            elif os.path.exists(link_path):
                with open(link_path) as f:
                    return f.read().strip()
            time.sleep(0.05)
        raise error("emulator did not publish slave link within %.1fs"
                    % (timeout,))

    def _materialize_emulator_config(self, src_path, dest_path, bridges):
        # `bridges` is a list of dicts with 'mcu' (section name) and
        # 'slave_path' (resolved pty). For single-MCU configs (one
        # entry, mcu=='mcu') we keep the legacy behavior: rewrite
        # every `serial:` line OR substitute SERIAL_PLACEHOLDER. For
        # multi-MCU configs we walk `[mcu ...]` sections and rewrite
        # each section's `serial:` line to its bridge's pty.
        with open(src_path) as f:
            cfg = f.read()
        if len(bridges) == 1 and bridges[0]['mcu'] == 'mcu':
            slave_path = bridges[0]['slave_path']
            if SERIAL_PLACEHOLDER in cfg:
                cfg = cfg.replace(SERIAL_PLACEHOLDER, slave_path)
            else:
                cfg, n = re.subn(r'(?m)^(\s*serial\s*:\s*).*$',
                                 r'\1' + slave_path, cfg)
                if n == 0:
                    raise error(
                        "EMULATOR config %r has no serial: line and no %r "
                        "placeholder; nothing to substitute"
                        % (src_path, SERIAL_PLACEHOLDER))
        else:
            slave_by_name = {b['mcu']: b['slave_path'] for b in bridges}
            out_lines = []
            current_mcu = None
            mcu_section_re = re.compile(r'^\s*\[mcu(?:\s+([a-z0-9_]+))?\]\s*$')
            section_re = re.compile(r'^\s*\[')
            serial_re = re.compile(r'^(\s*serial\s*:\s*).*$')
            replaced = set()
            for line in cfg.splitlines(True):
                m = mcu_section_re.match(line)
                if m:
                    current_mcu = m.group(1) or 'mcu'
                elif section_re.match(line):
                    current_mcu = None
                if current_mcu is not None and serial_re.match(line):
                    target = slave_by_name.get(current_mcu)
                    if target is not None:
                        sm = serial_re.match(line)
                        line = sm.group(1) + target + '\n'
                        replaced.add(current_mcu)
                out_lines.append(line)
            missing = set(slave_by_name) - replaced
            if missing:
                raise error(
                    "EMULATOR config %r missing serial: line for "
                    "[mcu %s]" % (src_path, ', '.join(sorted(missing))))
            cfg = ''.join(out_lines)
        with open(dest_path, 'w') as f:
            f.write(cfg)

    def _run_klippy_with_deadline(self, args, env=None):
        proc = subprocess.Popen(args, env=env)
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
