# Test-only injection for the MCU emulator harness.
#
# Python imports a module named ``sitecustomize`` automatically at
# interpreter startup if one is found on sys.path. test_klippy.py puts
# this directory at the front of the klippy subprocess's PYTHONPATH so
# this runs before klippy imports anything, WITHOUT modifying any klippy
# source file.
#
# Its job: relocate the handful of klippy timing constants that the
# deterministic tick-mode emulator needs widened out of production source.
# Those modules keep their real-hardware defaults as plain named constants
# (they never read an env var); when the harness sets the matching env var
# for a tick-mode run, this shim overwrites the constant right after the
# module is imported. The env vars are never set on real hardware.
#
#   KLIPPY_TRSYNC_TIMEOUT  -> mcu.TRSYNC_TIMEOUT
#       The 25ms multi-mcu homing watchdog. The host<->mcu trsync
#       keep-alive rides a closed loop on the serialqueue background
#       thread whose latency in *simulated* time is a whole advance
#       quantum (coarser than 25ms), so it intermittently trips
#       "Communication timeout during homing".
#   KLIPPY_PWM_START_LEAD  -> mcu.PWM_START_LEAD
#       The lead used to queue the first software-PWM cycle. The 0.200s
#       default is computed against the connect-time clock estimate,
#       which is still converging early in a tick-mode session, so the
#       first heater/fan queue_digital_out can land in the firmware's
#       past and trip "Timer too close" during config.
#   KLIPPY_BLTOUCH_CMD_LEAD -> extras.bltouch.CMD_SYNC_LEAD
#       The lead before the first queued BLTouch servo command. The
#       0.1s default is likewise computed against the still-converging
#       connect-time clock estimate, so a probe/self-test servo PWM edge
#       can land in the firmware's past ("Rescheduled timer in the past")
#       during a tick-mode bltouch/screws_tilt_adjust run.
#
# (The serialqueue command pre-transmit lead needs no relocation: the
# reactor's tick lockstep flushes ready serial commands at each advance
# target before the mcu runs forward - serialqueue_flush_ready - so they
# arrive with their normal real-hardware lead.)
#
# Relocating these does not mask bugs: a mis-routed endstop still fails
# the test (homing just times out later, or the run deadline hits).
import os

# (module name, attribute, env var) for each relocatable constant.
_SPECS = (
    ('mcu', 'TRSYNC_TIMEOUT', 'KLIPPY_TRSYNC_TIMEOUT'),
    ('mcu', 'PWM_START_LEAD', 'KLIPPY_PWM_START_LEAD'),
    ('extras.bltouch', 'CMD_SYNC_LEAD', 'KLIPPY_BLTOUCH_CMD_LEAD'),
)
# module name -> {attribute: override value (float)} from the environment.
_OVERRIDES = {}
for _mod, _attr, _env in _SPECS:
    _val = os.environ.get(_env)
    if _val:
        try:
            _OVERRIDES.setdefault(_mod, {})[_attr] = float(_val)
        except (TypeError, ValueError):
            pass

if _OVERRIDES:
    import sys
    from importlib.machinery import PathFinder

    class _ConstPatcher:
        # A meta-path finder that defers to the normal sys.path machinery
        # for any module named in _OVERRIDES, then wraps its loader so the
        # requested constants are overwritten immediately after the module
        # body runs (before any object reads them at config time). Works
        # for top-level (mcu) and packaged (extras.bltouch) modules alike -
        # find_spec receives the parent package's path for submodules.
        @staticmethod
        def find_spec(name, path=None, target=None):
            overrides = _OVERRIDES.get(name)
            if overrides is None:
                return None
            spec = PathFinder.find_spec(name, path, target)
            if spec is None or spec.loader is None:
                return None
            _orig_exec_module = spec.loader.exec_module

            def exec_module(module, _orig=_orig_exec_module, _ov=overrides):
                _orig(module)
                for attr, value in _ov.items():
                    setattr(module, attr, value)

            spec.loader.exec_module = exec_module
            return spec

    # Insert once even if site processing imports this module twice.
    if not any(getattr(f, '__name__', '') == '_ConstPatcher'
               for f in sys.meta_path):
        sys.meta_path.insert(0, _ConstPatcher)
