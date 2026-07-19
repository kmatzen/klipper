-------------------------- MODULE TickLivelock --------------------------
(***************************************************************************)
(* Abstract model of the klippy reactor tick-mode dispatch loop and its    *)
(* livelock guard.                                                          *)
(*                                                                          *)
(* Source of truth:                                                         *)
(*   klippy/reactor.py  _check_timers        (lines 222-244)                *)
(*                      _tick_decide_advance (lines 419-465)                *)
(*                      _tick_request_advance(lines 466-512)                *)
(*                      _dispatch_loop       (lines 593-615)                *)
(*   test/emulator/TICK_PROTOCOL_DESIGN.md section 5.1 mechanism 1          *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS MaxTime,        \* bounded abstract sim clock horizon
          StallLimit,     \* model stand-in for _TICK_STALL_LIMIT (64 in prod)
          NormalPeriod,   \* healthy timer reschedules strictly into the future
          QMax,           \* advance quantum, in abstract ticks
          GuardEnabled,   \* TRUE  = _tick_decide_advance stall guard present
          FdEnabled,      \* TRUE  = an fd may become ready at any time
          StallFix        \* "none"      = reactor.py:604-610 as written
                          \* "condreset" = reset stall at :606 only if the
                          \*               previous iteration advanced sim_time
                          \* "forceadv"  = also run the stall accounting in the
                          \*               fd branch and force the +1-cycle
                          \*               advance from there when it trips

Timers == {"path", "norm"}
NEVER  == MaxTime + 10

Min2(a, b) == IF a < b THEN a ELSE b

VARIABLES simtime,        \* abstract sim clock; only an advance round-trip moves it
          wake,           \* Timers -> waketime  (reactor timer_handler.waketime)
          busy,           \* _dispatch_loop `busy` local
          stall,          \* _tick_stall_iters
          fdPending,      \* an fd is ready for select() to report
          lastAdv         \* history var: TRUE iff last Iterate did an advance

vars == <<simtime, wake, busy, stall, fdPending, lastAdv>>

NextTimer(w) == Min2(w["path"], w["norm"])

(* A timer callback's new waketime.  "path" is the pathological clock-sync
   timer of 5.1 mechanism 1: it recomputes a waketime <= eventtime every
   time it fires.  "norm" is a healthy timer. *)
Resched(t) == IF t = "path" THEN simtime ELSE simtime + NormalPeriod

TypeOK ==
  /\ simtime \in 0..MaxTime
  /\ wake \in [Timers -> 0..(NEVER)]
  /\ busy \in BOOLEAN
  /\ stall \in 0..StallLimit
  /\ fdPending \in BOOLEAN
  /\ lastAdv \in BOOLEAN

Init ==
  /\ simtime = 0
  /\ wake = [t \in Timers |-> 0]
  /\ busy = TRUE                      \* _dispatch_loop: busy = True initially
  /\ stall = 0
  /\ fdPending = FALSE
  /\ lastAdv = FALSE

(* Environment: a background thread (renode pty drain) makes an fd readable. *)
FdArrive ==
  /\ FdEnabled
  /\ ~fdPending
  /\ fdPending' = TRUE
  /\ UNCHANGED <<simtime, wake, busy, stall, lastAdv>>

(*--------------------------------------------------------------------------
  One iteration of _dispatch_loop (reactor.py:596-615), atomically.
 --------------------------------------------------------------------------*)
Iterate ==
  LET fire    == simtime >= NextTimer(wake)                 \* reactor.py:223
      neww    == [t \in Timers |->
                    IF fire /\ simtime >= wake[t] THEN Resched(t) ELSE wake[t]]
      timeout == IF ~fire THEN (IF busy THEN 0 ELSE 1) ELSE 0
      nt      == NextTimer(neww)
      \* _tick_request_advance target selection (reactor.py:470-512)
      cap     == simtime + QMax
      tgt0    == IF nt > cap THEN cap ELSE nt
      tgt     == IF tgt0 < simtime THEN simtime ELSE tgt0
      \* bridge L1 +1-cycle guard: a clamped/equal target still steps 1 cycle
      newsim  == IF tgt <= simtime THEN Min2(simtime + 1, MaxTime)
                                   ELSE Min2(tgt, MaxTime)
  IN
  /\ wake' = neww
  /\ IF fdPending
       THEN \* reactor.py:604-609  select() reported an fd
            /\ busy' = TRUE
            /\ fdPending' = FALSE
            /\ IF StallFix = "none"
                 THEN /\ stall' = 0                         \* reactor.py:606
                      /\ simtime' = simtime
                      /\ lastAdv' = FALSE
                 ELSE IF StallFix = "condreset"
                   THEN /\ stall' = IF lastAdv THEN 0 ELSE stall
                        /\ simtime' = simtime
                        /\ lastAdv' = FALSE
                   ELSE IF StallFix = "asimpl"
                   THEN \* EXACTLY as implemented in reactor.py after the fix:
                        \* _tick_decide_advance(timeout, eventtime, after_fds=TRUE).
                        \* timeout > 0  -> reset streak, `return not after_fds`
                        \*                 == FALSE, so no advance this iteration
                        \*                 (deferred; keeps drain-before-advance).
                        \* nt > simtime -> reset streak, no advance.
                        \* else         -> COUNT the iteration; advance only at
                        \*                 the limit, and only after _check_fds.
                        IF timeout > 0
                          THEN /\ stall' = 0
                               /\ simtime' = simtime
                               /\ lastAdv' = FALSE
                          ELSE IF nt > simtime
                            THEN /\ stall' = 0
                                 /\ simtime' = simtime
                                 /\ lastAdv' = FALSE
                            ELSE IF GuardEnabled /\ stall + 1 >= StallLimit
                                   THEN /\ stall' = 0
                                        /\ simtime' = newsim
                                        /\ lastAdv' = TRUE
                                   ELSE /\ stall' = IF stall + 1 > StallLimit
                                                       THEN StallLimit
                                                       ELSE stall + 1
                                        /\ simtime' = simtime
                                        /\ lastAdv' = FALSE
                   ELSE \* "forceadv": stall accounting also on the fd path
                        IF lastAdv \/ nt > simtime
                          THEN /\ stall' = 0
                               /\ simtime' = simtime
                               /\ lastAdv' = FALSE
                          ELSE IF GuardEnabled /\ stall + 1 >= StallLimit
                                 THEN /\ stall' = 0
                                      /\ simtime' = newsim
                                      /\ lastAdv' = TRUE
                                 ELSE /\ stall' = IF stall + 1 > StallLimit
                                                    THEN StallLimit
                                                    ELSE stall + 1
                                      /\ simtime' = simtime
                                      /\ lastAdv' = FALSE
       ELSE \* reactor.py:610  _tick_decide_advance(timeout, eventtime)
            /\ fdPending' = fdPending
            /\ IF timeout > 0                               \* reactor.py:436
                 THEN /\ stall' = 0
                      /\ simtime' = newsim
                      /\ busy' = TRUE
                      /\ lastAdv' = TRUE
                 ELSE IF nt > simtime                       \* reactor.py:439
                   THEN /\ stall' = 0
                        /\ simtime' = simtime
                        /\ busy' = FALSE
                        /\ lastAdv' = FALSE
                   ELSE \* overdue timer with frozen sim_time  reactor.py:446
                        IF GuardEnabled /\ stall + 1 >= StallLimit
                          THEN /\ stall' = 0                \* reactor.py:458
                               /\ simtime' = newsim
                               /\ busy' = TRUE
                               /\ lastAdv' = TRUE
                          ELSE /\ stall' = IF stall + 1 > StallLimit
                                             THEN StallLimit ELSE stall + 1
                               /\ simtime' = simtime
                               /\ busy' = FALSE
                               /\ lastAdv' = FALSE

Next == Iterate \/ FdArrive

Spec == Init /\ [][Next]_vars /\ WF_vars(Iterate)

(*--------------------------------------------------------------------------
  Properties
 --------------------------------------------------------------------------*)
\* The reactor's sim clock always eventually reaches the horizon.
Liveness == <>(simtime = MaxTime)

\* Sim time never stops making progress.
SimProgress == []<><<simtime' > simtime>>_vars

=============================================================================
