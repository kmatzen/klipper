# File descriptor and timer event helper
#
# Copyright (C) 2016-2026  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, gc, select, socket, math, time, logging, queue
import greenlet
import chelper, util

_NOW = 0.
_NEVER = 9999999999999999.

class ReactorError(Exception):
    pass

class ReactorTimer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime
        self.timer_is_running = False

class ReactorCompletion:
    class sentinel: pass
    def __init__(self, reactor):
        self.reactor = reactor
        self.result = self.sentinel
        self.waiting = []
    def test(self):
        return self.result is not self.sentinel
    def complete(self, result):
        self.result = result
        for wait in self.waiting:
            self.reactor.update_timer(wait.timer, self.reactor.NOW)
    def wait(self, waketime=_NEVER, waketime_result=None):
        if self.result is self.sentinel:
            wait = greenlet.getcurrent()
            self.waiting.append(wait)
            self.reactor.pause(waketime)
            self.waiting.remove(wait)
            if self.result is self.sentinel:
                return waketime_result
        return self.result

class ReactorCallback:
    def __init__(self, reactor, callback, waketime):
        self.reactor = reactor
        self.timer = reactor.register_timer(self.invoke, waketime)
        self.callback = callback
        self.completion = ReactorCompletion(reactor)
    def invoke(self, eventtime):
        self.reactor.unregister_timer(self.timer)
        res = self.callback(eventtime)
        self.completion.complete(res)
        return self.reactor.NEVER

class ReactorFileHandler:
    def __init__(self, fd, read_callback, write_callback):
        self.fd = fd
        self.read_callback = read_callback
        self.write_callback = write_callback

class ReactorGreenlet(greenlet.greenlet):
    def __init__(self, run):
        greenlet.greenlet.__init__(self, run=run)
        self.timer = None

class ReactorMutex:
    def __init__(self, reactor, is_locked):
        self.reactor = reactor
        self.is_locked = is_locked
        self.next_pending = False
        self.queue = []
        self.lock = self.__enter__
        self.unlock = self.__exit__
    def test(self):
        return self.is_locked
    def __enter__(self):
        if not self.is_locked:
            self.is_locked = True
            return
        g = greenlet.getcurrent()
        self.queue.append(g)
        while 1:
            self.reactor.pause(self.reactor.NEVER)
            if self.next_pending and self.queue[0] is g:
                self.next_pending = False
                self.queue.pop(0)
                return
    def __exit__(self, type=None, value=None, tb=None):
        if not self.queue:
            self.is_locked = False
            return
        self.next_pending = True
        self.reactor.update_timer(self.queue[0].timer, self.reactor.NOW)

class ReactorPreventPause:
    def __init__(self, reactor):
        self.reactor = reactor
    def __enter__(self):
        self.reactor._prevent_pause_count += 1
    def __exit__(self, type=None, value=None, tb=None):
        self.reactor._prevent_pause_count -= 1

class SelectReactor:
    NOW = _NOW
    NEVER = _NEVER
    # Tick-mode lockstep (see _tick_request_advance and
    # TICK_PROTOCOL_DESIGN.md section 2.3): per-advance quantum caps.
    # _TICK_MAX_QUANTUM bounds an idle reactor's yield-to-bridge;
    # _TICK_WAIT_QUANTUM is the smaller cap used while a tick MCU is
    # blocked on a specific reply/trigger. It must stay below the
    # multi-MCU trsync watchdog (mcu.py TRSYNC_TIMEOUT); mcu.py enforces
    # that invariant at import time under KLIPPY_TICK_SOCKET.
    _TICK_MAX_QUANTUM = 0.1
    _TICK_WAIT_QUANTUM = 0.008
    # Tick-mode livelock guard (TICK_PROTOCOL_DESIGN.md section 5.1). After this
    # many consecutive no-progress iterations (timer overdue while
    # sim_time frozen) fail fast with a diagnosis instead of hanging to
    # the test deadline. Set well above any healthy-run transient streak
    # (measured high-water is 0; the one real trigger - the async renode
    # pty - was removed at the source by the synchronous AF_UNIX link).
    _TICK_STALL_LIMIT = 64
    # Hard safety net for the lockstep `done` recv (tick mode); fires
    # only if a bridge genuinely stops replying, names the socket, and
    # ends cleanly instead of hanging to the test deadline.
    _TICK_RECV_TIMEOUT = 60.0
    def __init__(self, gc_checking=False):
        # Main code
        self._process = False
        self.monotonic = chelper.get_ffi()[1].get_monotonic
        # Python garbage collection
        self._gc_checking = gc_checking
        self._last_gc_times = [0., 0., 0.]
        # Timers
        self._timers = []
        self._next_timer = self.NEVER
        # Callbacks
        self._pipe_fds = None
        self._async_queue = queue.Queue()
        # File descriptors
        self._dummy_fd_hdl = ReactorFileHandler(-1, (lambda e: None),
                                                (lambda e: None))
        self._fds = {}
        self._read_fds = []
        self._write_fds = []
        self._READ = 1
        self._WRITE = 2
        # Greenlets
        self._g_dispatch = None
        self._cached_dispatch_greenlets = []
        self._all_greenlets = []
        self._prevent_pause_count = 0
        # Tick mode (deterministic-time lockstep with the emulator
        # bridge). KLIPPY_TICK_SOCKET is a `:`-separated list of bridge
        # socket paths; the reactor connects to each, broadcasts each
        # `advance T`, and takes the min reported actual time.
        # TICK_PROTOCOL_DESIGN.md section 2 has the protocol.
        tick_env = os.environ.get('KLIPPY_TICK_SOCKET') or ''
        self._tick_socket_paths = [p for p in tick_env.split(':') if p]
        self._tick_sockets = []
        self._tick_recv_bufs = []
        # Per-transport pre-advance flush + need_prompt callbacks
        # (registered by serialhdl). Empty off tick mode.
        self._tick_flush_callbacks = []
        self._tick_need_prompt_callbacks = []
        # Determinism-proof trace (TICK_PROTOCOL_DESIGN.md 5.1). When
        # KLIPPY_TICK_TRACE is set, append one line per advance request; two
        # runs of the same test give byte-identical traces iff the reactor's
        # advance decisions are deterministic. Paired with the bridge's
        # per-advance trace. Disabled (None) otherwise.
        self._tick_trace_fp = None
        self._tick_trace_seq = 0
        # Livelock guard state (see _TICK_STALL_LIMIT / _tick_decide_advance).
        # _tick_stall_iters counts consecutive dispatch iterations with a timer
        # overdue while sim_time is frozen; _tick_stall_max is the high-water
        # mark, logged at finalize when KLIPPY_TICK_STALL_LOG is set so the
        # limit can be confirmed safely above any healthy-run transient.
        self._tick_stall_iters = 0
        self._tick_stall_max = 0
        # min(actuals) from the previous advance - the slowest bridge's real
        # sim time. See its use in _tick_request_advance. None until the first
        # advance completes; always == monotonic() for a single-mcu run.
        self._tick_last_actual = None
        self._tick_stall_log = bool(os.environ.get('KLIPPY_TICK_STALL_LOG'))
        try:
            self._TICK_STALL_LIMIT = int(
                os.environ.get('KLIPPY_TICK_STALL_LIMIT',
                               self._TICK_STALL_LIMIT))
        except ValueError:
            pass
    # Python garbage collection
    def get_gc_stats(self):
        return tuple(self._last_gc_times)
    def _check_gc(self, eventtime):
        if not self._gc_checking:
            return False
        gi = gc.get_count()
        if gi[0] < 700:
            return False
        # Reactor looks idle and gc is due - run it
        gc_level = 0
        if gi[1] >= 10:
            gc_level = 1
            if gi[2] >= 10:
                gc_level = 2
        self._last_gc_times[gc_level] = eventtime
        gc.collect(gc_level)
        return True
    # Timers
    def update_timer(self, timer_handler, waketime):
        if timer_handler.timer_is_running:
            return
        timer_handler.waketime = waketime
        self._next_timer = min(self._next_timer, waketime)
    def register_timer(self, callback, waketime=NEVER):
        timer_handler = ReactorTimer(callback, waketime)
        timers = list(self._timers)
        timers.append(timer_handler)
        self._timers = timers
        self._next_timer = min(self._next_timer, waketime)
        return timer_handler
    def unregister_timer(self, timer_handler):
        timer_handler.waketime = self.NEVER
        timers = list(self._timers)
        timers.pop(timers.index(timer_handler))
        self._timers = timers
    def _check_timers(self, eventtime, busy):
        if eventtime < self._next_timer:
            if busy:
                return 0.
            gc_busy = self._check_gc(eventtime)
            if gc_busy:
                return 0.
            return min(1., max(.001, self._next_timer - eventtime))
        self._next_timer = self.NEVER
        g_dispatch = self._g_dispatch
        for t in self._timers:
            waketime = t.waketime
            if eventtime >= waketime:
                t.waketime = self.NEVER
                t.timer_is_running = True
                t.waketime = waketime = t.callback(eventtime)
                t.timer_is_running = False
                if g_dispatch is not self._g_dispatch:
                    self._next_timer = min(self._next_timer, waketime)
                    self._end_greenlet(g_dispatch)
                    return 0.
            self._next_timer = min(self._next_timer, waketime)
        return 0.
    # Callbacks and Completions
    def completion(self):
        return ReactorCompletion(self)
    def register_callback(self, callback, waketime=NOW):
        rcb = ReactorCallback(self, callback, waketime)
        return rcb.completion
    # Asynchronous (from another thread) callbacks and completions
    def register_async_callback(self, callback, waketime=NOW):
        self._async_queue.put_nowait(
            (ReactorCallback, (self, callback, waketime)))
        try:
            os.write(self._pipe_fds[1], b'.')
        except os.error:
            pass
    def async_complete(self, completion, result):
        self._async_queue.put_nowait((completion.complete, (result,)))
        try:
            os.write(self._pipe_fds[1], b'.')
        except os.error:
            pass
    def _got_pipe_signal(self, eventtime):
        try:
            os.read(self._pipe_fds[0], 4096)
        except os.error:
            pass
        while 1:
            try:
                func, args = self._async_queue.get_nowait()
            except queue.Empty:
                break
            func(*args)
    def _setup_async_callbacks(self):
        self._pipe_fds = os.pipe()
        util.set_nonblock(self._pipe_fds[0])
        util.set_nonblock(self._pipe_fds[1])
        self.register_fd(self._pipe_fds[0], self._got_pipe_signal)
    # Greenlets
    def _sys_pause(self, waketime):
        # Pause using system sleep for when reactor not running
        delay = waketime - self.monotonic()
        if delay > 0.:
            time.sleep(delay)
        return self.monotonic()
    def pause(self, waketime):
        if self._g_dispatch is None:
            # The reactor is not running - use a system pause instead
            return self._sys_pause(waketime)
        if self._prevent_pause_count:
            self.verify_can_pause()
        # Determine if this greenlet is the main dispatch greenlet
        g = greenlet.getcurrent()
        if g is not self._g_dispatch:
            # This greenlet has called pause() before and has a timer setup,
            # so switch to _check_timers (via g.timer.callback return)
            return self._g_dispatch.switch(waketime)
        # Pausing the dispatch greenlet - setup timer to resume this greenlet
        g.timer = self.register_timer(g.switch, waketime)
        self._next_timer = self.NOW
        if self._cached_dispatch_greenlets:
            # Switch to _end_greenlet to activate cached dispatch greenlet
            g_next = self._cached_dispatch_greenlets.pop()
            eventtime = g_next.switch()
        else:
            # No cached greenlets, switch to run() to create new dispatcher
            eventtime = g.parent.switch()
        # This greenlet activated from g.timer.callback (via _check_timers)
        return eventtime
    def _end_greenlet(self, g_old):
        # A timer/io event that called pause() has completed.
        # Cleanup the internal timer associated with this greenlet.
        self.unregister_timer(g_old.timer)
        g_old.timer = None
        # Cache this greenlet for later use
        self._cached_dispatch_greenlets.append(g_old)
        # Switch to _check_timers (via g_old.timer.callback return)
        self._g_dispatch.switch(self.NEVER)
        # This greenlet reactivated from pause() - return to main dispatch loop
        self._g_dispatch = g_old
    # Support for temporarily disabling pauses
    def assert_no_pause(self):
        return ReactorPreventPause(self)
    def verify_can_pause(self):
        if self._prevent_pause_count:
            raise ReactorError("Internal error - reactor pause disabled")
    # Mutexes
    def mutex(self, is_locked=False):
        return ReactorMutex(self, is_locked)
    # File descriptors
    def register_fd(self, fd, read_callback, write_callback=None):
        file_handler = ReactorFileHandler(fd, read_callback, write_callback)
        self._fds[fd] = file_handler
        self.set_fd_wake(file_handler, True, False)
        return file_handler
    def unregister_fd(self, file_handler):
        self.set_fd_wake(file_handler, False, False)
        del self._fds[file_handler.fd]
    def set_fd_wake(self, file_handler, is_readable=True, is_writeable=False):
        fd = file_handler.fd
        if fd in self._read_fds:
            if not is_readable:
                self._read_fds.remove(fd)
        elif is_readable:
            self._read_fds.append(fd)
        if fd in self._write_fds:
            if not is_writeable:
                self._write_fds.remove(fd)
        elif is_writeable:
            self._write_fds.append(fd)
    def _check_fds(self, eventtime, hdls):
        g_dispatch = self._g_dispatch
        for fd, event in hdls:
            hdl = self._fds.get(fd, self._dummy_fd_hdl)
            if event & self._READ:
                hdl.read_callback(eventtime)
                if g_dispatch is not self._g_dispatch:
                    self._end_greenlet(g_dispatch)
                    return self.monotonic()
            if event & self._WRITE:
                hdl.write_callback(eventtime)
                if g_dispatch is not self._g_dispatch:
                    self._end_greenlet(g_dispatch)
                    return self.monotonic()
        return eventtime
    # Tick-mode lockstep with one or more external time drivers
    def _tick_connect(self):
        if not self._tick_socket_paths or self._tick_sockets:
            return
        deadline = time.monotonic() + 5.0
        for path in self._tick_socket_paths:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            last_err = None
            while time.monotonic() < deadline:
                try:
                    s.connect(path)
                    self._tick_sockets.append(s)
                    self._tick_recv_bufs.append(b'')
                    s = None
                    break
                except OSError as e:
                    last_err = e
                    time.sleep(0.05)
            if s is not None:
                s.close()
                # Tear down anything we already connected so a partial
                # connect doesn't leave dangling fds.
                self._tick_close()
                raise ReactorError(
                    "Could not connect KLIPPY_TICK_SOCKET=%s: %s"
                    % (path, last_err))
    def _tick_close(self):
        for s in self._tick_sockets:
            try:
                s.close()
            except OSError:
                pass
        self._tick_sockets = []
        self._tick_recv_bufs = []
    def register_tick_flush(self, callback):
        # See _tick_flush_callbacks. callback(target) is invoked with the
        # next advance target (simulated seconds) before each bridge advance.
        self._tick_flush_callbacks.append(callback)
    def unregister_tick_flush(self, callback):
        try:
            self._tick_flush_callbacks.remove(callback)
        except ValueError:
            pass
    def register_tick_need_prompt(self, callback):
        # See _tick_need_prompt_callbacks. callback() -> bool.
        self._tick_need_prompt_callbacks.append(callback)
    def unregister_tick_need_prompt(self, callback):
        try:
            self._tick_need_prompt_callbacks.remove(callback)
        except ValueError:
            pass
    def _tick_decide_advance(self, timeout, eventtime, after_fds=False):
        # Decide whether the dispatch loop should hand control to the
        # bridge(s) this iteration. Returns True to advance, False to keep
        # running timers. The normal case is "a future timer is pending"
        # (timeout > 0). The livelock guard covers the case where a timer is
        # overdue (waketime <= eventtime) yet sim_time is frozen because
        # _check_timers keeps returning 0 (see _TICK_STALL_LIMIT): after a
        # streak of such no-progress iterations it raises ReactorError with
        # a diagnosis, so a wedged protocol state surfaces as a prompt,
        # attributable failure instead of a hang (or a silent self-heal
        # that could mask a real protocol bug). Inert on healthy runs (the
        # overdue-with-frozen-sim streak stays far below the limit; measured
        # high-water 0 across the full gate). Streak resets on any progress.
        #
        # after_fds is True when this iteration already ran _check_fds. Such an
        # iteration must still be COUNTED: fd readiness is not sim-time
        # progress, and an fd that goes ready more often than once every
        # _TICK_STALL_LIMIT iterations (the renode link was an async pty drained
        # by a background thread and did exactly that; it now uses the same
        # synchronous AF_UNIX link as simavr, but the guard must not depend on
        # that) would otherwise reset the streak
        # forever and the guard could never fire - the livelock survives the
        # guard. It must not, however, ADVANCE on the healthy timeout > 0 path:
        # that is deferred to the next iteration, so the guard is only
        # consulted after the link was drained (the TICK_PROTOCOL_DESIGN.md
        # 2.5 D-PRE precondition).
        if timeout > 0.:
            self._tick_stall_iters = 0
            return not after_fds
        if self._next_timer > eventtime:
            # Overdue timer already rescheduled into the future (e.g. the
            # _check_timers "busy" short-circuit, or a healthy timer fire):
            # the next iteration takes the timeout > 0 path. Not a stall.
            self._tick_stall_iters = 0
            return False
        # A timer is overdue but sim_time is not advancing.
        self._tick_stall_iters += 1
        if self._tick_stall_iters > self._tick_stall_max:
            self._tick_stall_max = self._tick_stall_iters
            if self._tick_stall_log and not (
                    self._tick_stall_iters & (self._tick_stall_iters - 1)):
                # Power-of-two milestone: trace how high the streak climbs so
                # the limit can be confirmed above any healthy transient.
                logging.warning(
                    "reactor: tick overdue streak=%d (limit=%d) at sim %.6f",
                    self._tick_stall_iters, self._TICK_STALL_LIMIT, eventtime)
        if self._tick_stall_iters < self._TICK_STALL_LIMIT:
            return False
        raise ReactorError(
            "Tick-mode livelock: timer overdue (waketime %.6f) for %d"
            " consecutive iterations with sim time frozen at %.6f. A"
            " firmware reply this timer needs is not being produced -"
            " see TICK_PROTOCOL_DESIGN.md section 5.1 and rerun with"
            " KLIPPY_TICK_STALL_LOG=1 for the streak trace."
            % (self._next_timer, self._TICK_STALL_LIMIT, eventtime))
    def _tick_request_advance(self):
        # Ask all bridges to advance simulated time, in lockstep.
        # Returns the minimum reported actual (so klippy never thinks
        # any bridge is ahead of where it actually is) or None on EOF.
        target = self._next_timer
        eventtime = self.monotonic()
        # monotonic() reads the sim-time mmap of the CANONICAL (first) bridge
        # only. With more than one mcu that is not the same thing as "the
        # simulated time every bridge has reached": a bridge may reply `done`
        # short of the target (the O1 output cap in simavr_bridge.c breaks the
        # run loop once the tx buffer hits OUTPUT_CAP), so the canonical clock
        # can be ahead of a slower bridge. Computing the flush horizon from it
        # then hands that bridge commands whose req_clock it already considers
        # past -> step-queue underrun / "Timer too close". Clamp to the slowest
        # bridge's last reported actual - i.e. actually USE the min(actuals)
        # this function returns, which previously no caller consumed. No-op for
        # a single mcu, where the min IS the canonical clock.
        if (self._tick_last_actual is not None
                and self._tick_last_actual < eventtime):
            eventtime = self._tick_last_actual
        # Ask each transport what it is waiting on (TICK_PROTOCOL_DESIGN.md
        # NEED_PROMPT): >=1 means klippy is blocked on a specific firmware
        # reply (1 = identify/query/clock-sync) or trigger (2 = trsync homing)
        # that may arrive before the next timer, so use the small bounded
        # quantum; 0 means it is merely advancing toward a timer (streaming),
        # so use the full quantum and let unsolicited output coalesce.
        mode = 0
        for need_prompt in self._tick_need_prompt_callbacks:
            m = need_prompt()
            if m > mode:
                mode = m
        if mode >= 2 and len(self._tick_sockets) == 1:
            # Mode 2 is an active trsync (homing / probing). With a single
            # mcu the firmware watchdog is TRSYNC_SINGLE_MCU_TIMEOUT (0.25 s,
            # mcu.py) rather than the 0.025 s multi-mcu TRSYNC_TIMEOUT, and the
            # trigger is handled firmware-side (the host only has to learn of
            # it within a quantum - it never stops the move itself), so the
            # small wait quantum buys nothing here. Use the full quantum so a
            # long probe descent advances at streaming speed instead of ~12x
            # slower (load_cell PROBE / BED_MESH_CALIBRATE was hitting the wall
            # deadline at the 8 ms cap). The heartbeat stays alive: each
            # trsync_set_timeout extension is flushed a MIN_REQTIME_DELTA
            # (0.1 s) lead ahead of its req_clock and read by the firmware at
            # the start of the crossing advance, always before the 0.25 s
            # deadline, so the 0.1 s quantum leaves ample margin. Multi-mcu
            # homing (len > 1, 0.025 s watchdog) keeps the small quantum below.
            cap = eventtime + self._TICK_MAX_QUANTUM
        elif mode:
            cap = eventtime + self._TICK_WAIT_QUANTUM
        else:
            cap = eventtime + self._TICK_MAX_QUANTUM
        if target >= self.NEVER or target > cap:
            target = cap
        if target < eventtime:
            # The next timer is overdue (waketime <= eventtime) on a normal
            # "advance toward the timer" iteration. Clamp to eventtime; the
            # bridge's L1 +1-cycle guard then steps the MCU by one cycle,
            # so sim_time moves forward minimally without overshooting any
            # queued step.
            target = eventtime
        # Transmit any serial commands that are ready to send by `target`
        # before the mcu runs forward, so they reach the firmware with the
        # normal pre-transmit lead rather than a whole quantum late. Pass
        # the current eventtime as the send timestamp (target is only the
        # look-ahead horizon) so clock-sync timing stays honest.
        for flush in self._tick_flush_callbacks:
            flush(eventtime, target)
        # No early-exit flag: the per-advance quantum above (small while
        # waiting, full while streaming) already bounds reply/trigger latency,
        # and a per-byte early-exit would chop concurrent streaming into one
        # round trip per sample. See TICK_PROTOCOL_DESIGN.md.
        msg = ('advance %.9f\n' % target).encode('ascii')
        try:
            for s in self._tick_sockets:
                s.sendall(msg)
        except OSError:
            return None
        actuals = []
        diag = self._tick_stall_log
        for i, s in enumerate(self._tick_sockets):
            try:
                while b'\n' not in self._tick_recv_bufs[i]:
                    # Bounded wait for `done` (mechanism (2),
                    # TICK_PROTOCOL_DESIGN.md 4.1): poll the socket so a bridge
                    # that stops replying surfaces as a prompt failure naming
                    # the culprit, not a silent hang until the test deadline.
                    # The done arrives within ms in a healthy run, so this is
                    # a no-op there; the _TICK_RECV_TIMEOUT net only fires off
                    # a genuine wedge. KLIPPY_TICK_STALL_LOG adds a per-5s
                    # progress line for live diagnosis.
                    waited = 0.
                    while not select.select([s], [], [], 5.0)[0]:
                        waited += 5.
                        if diag:
                            logging.warning(
                                "reactor: tick STALL awaiting 'done' from"
                                " socket %d (%s) for advance %.6f mode %d"
                                " (%.0fs, sim now %.6f)", i,
                                self._tick_socket_paths[i], target, mode,
                                waited, eventtime)
                        if waited >= self._TICK_RECV_TIMEOUT:
                            logging.error(
                                "reactor: tick bridge on socket %d (%s) did"
                                " not reply 'done' within %.0fs; ending"
                                " (advance %.6f mode %d, sim %.6f)", i,
                                self._tick_socket_paths[i],
                                self._TICK_RECV_TIMEOUT, target, mode,
                                eventtime)
                            return None
                    chunk = s.recv(64)
                    if not chunk:
                        if diag:
                            logging.warning(
                                "reactor: tick socket %d (%s) returned EOF"
                                " awaiting 'done' for advance %.6f", i,
                                self._tick_socket_paths[i], target)
                        return None
                    self._tick_recv_bufs[i] += chunk
                line, self._tick_recv_bufs[i] = (
                    self._tick_recv_bufs[i].split(b'\n', 1))
            except OSError:
                return None
            if not line.startswith(b'done '):
                return None
            try:
                actuals.append(float(line[5:]))
            except ValueError:
                return None
        try:
            result = min(actuals)
        except ValueError:
            return None
        # Remember the slowest bridge for the next target computation above.
        self._tick_last_actual = result
        if self._tick_trace_fp is not None:
            self._tick_trace_fp.write(
                "%d %.9f %d %.9f\n" % (self._tick_trace_seq, target,
                                       mode, result))
            self._tick_trace_fp.flush()
            self._tick_trace_seq += 1
        return result
    # Main loop
    def _dispatch_loop(self):
        busy = True
        eventtime = self.monotonic()
        while self._process:
            timeout = self._check_timers(eventtime, busy)
            busy = False
            in_tick = bool(self._tick_sockets)
            wait_timeout = 0 if in_tick else timeout
            res = select.select(self._read_fds, self._write_fds, [],
                                wait_timeout)
            eventtime = self.monotonic()
            after_fds = bool(res[0] or res[1])
            if after_fds:
                busy = True
                hdls = ([(fd, self._READ) for fd in res[0]]
                        + [(fd, self._WRITE) for fd in res[1]])
                # Drain first, unconditionally: the advance below must never
                # run with a readable link still buffered (2.5 D-PRE).
                eventtime = self._check_fds(eventtime, hdls)
            # Consult the livelock guard on EVERY tick iteration, including
            # ones that serviced an fd. It returns True on an fd iteration only
            # when the stall limit is reached, so healthy runs are unaffected.
            if in_tick and self._tick_decide_advance(timeout, eventtime,
                                                     after_fds):
                if self._tick_request_advance() is None:
                    self.end()
                    continue
                eventtime = self.monotonic()
                busy = True
    def run(self):
        if self._pipe_fds is None:
            self._setup_async_callbacks()
        if not self._tick_sockets and self._tick_socket_paths:
            self._tick_connect()
        if (self._tick_sockets and self._tick_trace_fp is None
                and os.environ.get('KLIPPY_TICK_TRACE')):
            self._tick_trace_fp = open(
                os.environ['KLIPPY_TICK_TRACE'] + '.klippy', 'w')
            self._tick_trace_fp.write("# seq target mode actual\n")
        self._process = True
        self._prevent_pause_count = 0
        try:
            while self._process:
                # Create new greenlet to dispatch timers and events
                g_next = ReactorGreenlet(run=self._dispatch_loop)
                self._all_greenlets.append(g_next)
                self._g_dispatch = g_next
                g_next.switch()
                # Control returns here on end() request or switch from pause()
        finally:
            self._g_dispatch = None
            if self._tick_stall_log and self._tick_socket_paths:
                logging.warning(
                    "reactor: tick livelock guard stats: max overdue"
                    " streak=%d (limit=%d)",
                    self._tick_stall_max, self._TICK_STALL_LIMIT)
    def end(self):
        self._process = False
    def finalize(self):
        self._g_dispatch = None
        self._cached_dispatch_greenlets = []
        for g in self._all_greenlets:
            try:
                g.throw()
            except:
                logging.exception("reactor finalize greenlet terminate")
        self._all_greenlets = []
        if self._pipe_fds is not None:
            os.close(self._pipe_fds[0])
            os.close(self._pipe_fds[1])
            self._pipe_fds = None
        self._tick_close()

class PollReactor(SelectReactor):
    def __init__(self, gc_checking=False):
        SelectReactor.__init__(self, gc_checking)
        self._poll = select.poll()
        self._READ = select.POLLIN | select.POLLHUP
        self._WRITE = select.POLLOUT
    # File descriptors
    def register_fd(self, fd, read_callback, write_callback=None):
        file_handler = ReactorFileHandler(fd, read_callback, write_callback)
        self._fds[fd] = file_handler
        self._poll.register(file_handler.fd, select.POLLIN | select.POLLHUP)
        return file_handler
    def unregister_fd(self, file_handler):
        self._poll.unregister(file_handler.fd)
        del self._fds[file_handler.fd]
    def set_fd_wake(self, file_handler, is_readable=True, is_writeable=False):
        flags = select.POLLHUP
        if is_readable:
            flags |= select.POLLIN
        if is_writeable:
            flags |= select.POLLOUT
        self._poll.modify(file_handler.fd, flags)
    # Main loop
    def _dispatch_loop(self):
        busy = True
        eventtime = self.monotonic()
        while self._process:
            timeout = self._check_timers(eventtime, busy)
            busy = False
            in_tick = bool(self._tick_sockets)
            wait_ms = 0 if in_tick else int(math.ceil(timeout * 1000.))
            res = self._poll.poll(wait_ms)
            eventtime = self.monotonic()
            after_fds = bool(res)
            if after_fds:
                busy = True
                # Drain first (2.5 D-PRE), then let the guard see this
                # iteration - see _tick_decide_advance.
                eventtime = self._check_fds(eventtime, res)
            if in_tick and self._tick_decide_advance(timeout, eventtime,
                                                    after_fds):
                if self._tick_request_advance() is None:
                    self.end()
                    continue
                eventtime = self.monotonic()
                busy = True

class EPollReactor(SelectReactor):
    def __init__(self, gc_checking=False):
        SelectReactor.__init__(self, gc_checking)
        self._epoll = select.epoll()
        self._READ = select.EPOLLIN | select.EPOLLHUP
        self._WRITE = select.EPOLLOUT
    # File descriptors
    def register_fd(self, fd, read_callback, write_callback=None):
        file_handler = ReactorFileHandler(fd, read_callback, write_callback)
        self._fds[fd] = file_handler
        self._epoll.register(fd, select.EPOLLIN | select.EPOLLHUP)
        return file_handler
    def unregister_fd(self, file_handler):
        self._epoll.unregister(file_handler.fd)
        del self._fds[file_handler.fd]
    def set_fd_wake(self, file_handler, is_readable=True, is_writeable=False):
        flags = select.EPOLLHUP
        if is_readable:
            flags |= select.EPOLLIN
        if is_writeable:
            flags |= select.EPOLLOUT
        self._epoll.modify(file_handler.fd, flags)
    # Main loop
    def _dispatch_loop(self):
        busy = True
        eventtime = self.monotonic()
        while self._process:
            timeout = self._check_timers(eventtime, busy)
            busy = False
            in_tick = bool(self._tick_sockets)
            wait_timeout = 0. if in_tick else timeout
            res = self._epoll.poll(wait_timeout)
            eventtime = self.monotonic()
            after_fds = bool(res)
            if after_fds:
                busy = True
                # Drain first (2.5 D-PRE), then let the guard see this
                # iteration - see _tick_decide_advance.
                eventtime = self._check_fds(eventtime, res)
            if in_tick and self._tick_decide_advance(timeout, eventtime,
                                                    after_fds):
                if self._tick_request_advance() is None:
                    self.end()
                    continue
                eventtime = self.monotonic()
                busy = True

# Use the poll based reactor if it is available
try:
    select.poll
    Reactor = PollReactor
except:
    Reactor = SelectReactor
