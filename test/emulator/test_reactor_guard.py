#!/usr/bin/env python3
# Regression tests for the tick-mode livelock guard (reactor.py
# _tick_decide_advance, TICK_PROTOCOL_DESIGN.md 5.1).
#
# Standalone like validate_renode.py - the emulator gate cannot cover this.
# The guard only fires on a pathological interleaving (a timer permanently
# overdue while sim_time is frozen), which a healthy run never reaches, so a
# regression here is invisible to every .test in the suite. TLA+
# (tla/livelock) proves the shape is right; this pins the shipped Python to
# that shape.
#
#   python3 test/emulator/test_reactor_guard.py
#
# Exit 0 = pass, 1 = failure. Imports reactor.py with chelper/greenlet/util
# stubbed so it runs without the compiled extension.

import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_KLIPPY = os.path.join(_HERE, '..', '..', 'klippy')


def _load_reactor():
    for name in ('chelper', 'greenlet', 'util'):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules['chelper'].get_ffi = lambda: (
        None, types.SimpleNamespace(get_monotonic=lambda: 0.))
    sys.modules['greenlet'].greenlet = object
    sys.path.insert(0, os.path.abspath(_KLIPPY))
    import reactor
    return reactor


reactor_mod = _load_reactor()
LIMIT = reactor_mod.SelectReactor._TICK_STALL_LIMIT


def make():
    # Build a reactor without __init__ (which needs the real chelper) and set
    # only the guard state _tick_decide_advance touches.
    r = reactor_mod.SelectReactor.__new__(reactor_mod.SelectReactor)
    r._tick_stall_iters = 0
    r._tick_stall_max = 0
    r._tick_stall_forced = 0
    r._tick_stall_log = False
    r._TICK_STALL_LIMIT = LIMIT
    r._next_timer = 0.
    return r


def test_healthy_no_fd_advances():
    # timeout > 0 and no fd serviced -> advance, exactly as before the fix.
    r = make()
    assert r._tick_decide_advance(0.5, 1.0, False) is True
    assert r._tick_stall_iters == 0


def test_healthy_after_fd_defers():
    # timeout > 0 on an fd iteration -> do NOT advance now. Pre-fix the guard
    # was not consulted at all on this path, so returning False preserves the
    # old behaviour; the advance happens on the next iteration. This is what
    # keeps a forced advance from ever bypassing the drain (2.5 D-PRE).
    r = make()
    assert r._tick_decide_advance(0.5, 1.0, True) is False
    assert r._tick_stall_iters == 0


def test_timer_rescheduled_forward_is_not_a_stall():
    # _next_timer in the future is normal progress, never a stall - the guard
    # must stay completely inert here no matter how many iterations pass.
    r = make()
    r._next_timer = 5.0
    for _ in range(LIMIT * 3):
        assert r._tick_decide_advance(0., 1.0, True) is False
    assert r._tick_stall_iters == 0


def test_livelock_with_fd_ready_every_iteration_terminates():
    # THE REGRESSION. Overdue timer + frozen sim_time + an fd ready on every
    # iteration. Pre-fix, reactor.py reset the streak unconditionally on the
    # fd path and kept the guard in the `elif`, so the streak oscillated
    # 0->1->0 and the guard could never fire. TLC finds this as a lasso
    # (tla/livelock, MCGuardFd).
    r = make()
    r._next_timer = 1.0            # <= eventtime => overdue
    forced = sum(bool(r._tick_decide_advance(0., 1.0, True))
                 for _ in range(LIMIT * 4))
    assert forced >= 3, "guard never fired despite fds ready (livelock)"


def test_livelock_alternating_fd_terminates():
    # Same, with an fd ready every other iteration.
    r = make()
    r._next_timer = 1.0
    forced = sum(bool(r._tick_decide_advance(0., 1.0, i % 2 == 0))
                 for i in range(LIMIT * 4))
    assert forced >= 3, "alternating-fd livelock not broken"


def test_forced_advance_is_counted_and_resets():
    r = make()
    r._next_timer = 1.0
    for _ in range(LIMIT):
        r._tick_decide_advance(0., 1.0, False)
    assert r._tick_stall_forced >= 1
    assert r._tick_stall_iters == 0, "streak must reset after forcing"


def main():
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_'):
            continue
        try:
            fn()
            sys.stdout.write("PASS  %s\n" % name)
        except AssertionError as e:
            fails += 1
            sys.stdout.write("FAIL  %s: %s\n" % (name, e))
    sys.stdout.write("\n%s\n" % ("ALL PASS" if not fails
                                 else "%d FAILED" % fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
