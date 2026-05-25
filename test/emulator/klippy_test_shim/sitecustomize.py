# Test-only injection for the MCU emulator harness.
#
# Python imports a module named ``sitecustomize`` automatically at
# interpreter startup if one is found on sys.path. test_klippy.py puts
# this directory at the front of the klippy subprocess's PYTHONPATH so
# this runs before klippy imports anything, WITHOUT modifying any klippy
# source file.
#
# Its job: relocate the handful of klippy/mcu.py timing constants that the
# deterministic tick-mode emulator needs widened out of production source.
# klippy/mcu.py keeps its real-hardware defaults as plain named constants
# (it never reads an env var); when the harness sets the matching env var
# for a tick-mode run, this shim overwrites the constant right after mcu
# is imported. The env vars are never set on real hardware.
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
#
# (The serialqueue command pre-transmit lead needs no relocation: the
# reactor's tick lockstep flushes ready serial commands at each advance
# target before the mcu runs forward - serialqueue_flush_ready - so they
# arrive with their normal real-hardware lead.)
#
# Relocating these does not mask bugs: a mis-routed endstop still fails
# the test (homing just times out later, or the run deadline hits).
import os

# Map mcu.py attribute name -> override value (float) from the environment.
_OVERRIDES = {}
for _attr, _env in (('TRSYNC_TIMEOUT', 'KLIPPY_TRSYNC_TIMEOUT'),
                    ('PWM_START_LEAD', 'KLIPPY_PWM_START_LEAD')):
    _val = os.environ.get(_env)
    if _val:
        try:
            _OVERRIDES[_attr] = float(_val)
        except (TypeError, ValueError):
            pass

if _OVERRIDES:
    import sys
    from importlib.machinery import PathFinder

    class _McuConstPatcher:
        # A meta-path finder that defers to the normal sys.path machinery
        # for the top-level ``mcu`` module, then wraps its loader so the
        # requested constants are overwritten immediately after the module
        # body runs (before any MCU object reads them at config time).
        @staticmethod
        def find_spec(name, path=None, target=None):
            if name != 'mcu':
                return None
            spec = PathFinder.find_spec(name, path, target)
            if spec is None or spec.loader is None:
                return None
            _orig_exec_module = spec.loader.exec_module

            def exec_module(module, _orig=_orig_exec_module):
                _orig(module)
                for attr, value in _OVERRIDES.items():
                    setattr(module, attr, value)

            spec.loader.exec_module = exec_module
            return spec

    # Insert once even if site processing imports this module twice.
    if not any(getattr(f, '__name__', '') == '_McuConstPatcher'
               for f in sys.meta_path):
        sys.meta_path.insert(0, _McuConstPatcher)
