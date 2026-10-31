# Symbolically check the actual clock timer callback with CrossHair/Z3.
# Install crosshair-tool, then run from the repository root:
# PYTHONPATH=klippy crosshair check test/formal/clocksync_queries.py \
#     --analysis_kind=PEP316 --report_all --per_condition_timeout=30
#
# This is an inductive, single-callback proof, assuming no concurrent response
# during the callback. Responses reset queries_pending to zero; an absent MCU
# supplies no responses. It does not prove transport behavior or thread safety.
# This file may be distributed under the terms of the GNU GPLv3 license.
from clocksync import ClockSync


class Serial:
    def __init__(self):
        self.sent = 0

    def raw_send(self, *args):
        self.sent += 1


class Reactor:
    def register_timer(self, callback):
        return callback


def poll_step(pending: int) -> tuple:
    """
    pre: pending >= 0
    post: __return__[0] == (pending + 1 if pending <= 4 else pending)
    post: __return__[1] == (1 if pending <= 4 else 0)
    post: __return__[2] == (__return__[0] <= 4)
    """
    sync = ClockSync(Reactor())
    sync.serial = Serial()
    sync.queries_pending = pending
    sync._get_clock_event(0.)
    return sync.queries_pending, sync.serial.sent, sync.is_active()


def bounded_step(pending: int) -> int:
    """
    pre: 0 <= pending <= 5
    post: 0 <= __return__ <= 5
    """
    sync = ClockSync(Reactor())
    sync.serial = Serial()
    sync.queries_pending = pending
    sync._get_clock_event(0.)
    return sync.queries_pending


def response_resumes(pending: int) -> tuple:
    """
    pre: pending >= 0
    post: __return__ == (1, 1, True)
    """
    sync = ClockSync(Reactor())
    sync.serial = Serial()
    sync.queries_pending = pending
    sync._handle_clock({'clock': 1, '#sent_time': 0., '#receive_time': 1.})
    sync._get_clock_event(2.)
    return sync.queries_pending, sync.serial.sent, sync.is_active()
