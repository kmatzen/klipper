------------------------------ MODULE Lockstep ------------------------------
(***************************************************************************)
(* Multi-MCU tick-mode lockstep: klippy reactor R fans an `advance T` out   *)
(* to N bridges, then collects `done T'` from each.                         *)
(*                                                                          *)
(* Models klippy/reactor.py _tick_request_advance (466-591) and             *)
(* _dispatch_loop (593-615), plus the bridges' early-cap behaviour.         *)
(*                                                                          *)
(* Key structural fact taken from the code: _tick_request_advance computes   *)
(* `result = min(actuals)` (line 582) and returns it, but every call site    *)
(* (611, 699, 743) only tests `is None`.  The reactor's notion of time then  *)
(* comes from `self.monotonic()`, which reads the sim-time mmap of the       *)
(* CANONICAL (first) bridge only.  UseMin lets us compare both designs.      *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets, Sequences, TLC

CONSTANTS
    N,          \* number of bridges (MCUs)
    MaxTime,    \* state-space bound on simulated time
    Quantum,    \* advance quantum (QMAX / QWAIT, in abstract ticks)
    UseBarrier, \* TRUE = retry laggards until all bridges reach the target
    UseMin      \* TRUE  = klippy adopts min(actuals)  (the documented intent)
                \* FALSE = klippy adopts the canonical bridge's mmap (as built)

Bridges   == 1..N
Canonical == 1          \* canonical_sim_time_file = first bridge (test_klippy.py)

Min2(a,b) == IF a < b THEN a ELSE b
MinOf(f)  == CHOOSE v \in {f[b] : b \in Bridges} : \A b \in Bridges : v <= f[b]
MaxOf(f)  == CHOOSE v \in {f[b] : b \in Bridges} : \A b \in Bridges : v >= f[b]

VARIABLES
    bsim,       \* bsim[b]  : bridge b's actual simulated time (avr->cycle/freq)
    ktime,      \* klippy's eventtime  (what monotonic() returns)
    nextTimer,  \* reactor's _next_timer
    lastTarget, \* last advance target broadcast (for reporting)
    reportedMin \* min(actuals) from the last round (computed, then discarded)

vars == <<bsim, ktime, nextTimer, lastTarget, reportedMin>>

TypeOK ==
    /\ bsim \in [Bridges -> 0..MaxTime]
    /\ ktime \in 0..MaxTime
    /\ nextTimer \in 0..MaxTime
    /\ lastTarget \in 0..MaxTime
    /\ reportedMin \in 0..MaxTime

Init ==
    /\ bsim = [b \in Bridges |-> 0]
    /\ ktime = 0
    /\ nextTimer = 0
    /\ lastTarget = 0
    /\ reportedMin = 0

(***************************************************************************)
(* Target computation, _tick_request_advance lines 470-512.                 *)
(* eventtime = monotonic() = ktime;  cap = eventtime + quantum;             *)
(* if target > cap: target = cap;  if target < eventtime: target = eventtime *)
(***************************************************************************)
Target ==
    LET cap == Min2(ktime + Quantum, MaxTime)
        t   == IF nextTimer > cap THEN cap ELSE nextTimer
    IN  IF t < ktime THEN ktime ELSE t

(***************************************************************************)
(* A bridge servicing `advance T`.  Three legal outcomes, all in the code:  *)
(*   - full step:  actual = T                                              *)
(*   - L1 guard:   T <= bsim[b]  =>  actual = bsim[b] + 1 (one cycle)       *)
(*   - O1 early cap: g_suart_tx_len >= OUTPUT_CAP breaks the run loop, so   *)
(*     the bridge replies done with bsim[b] < actual < T.                   *)
(* Bridges never move backwards.                                            *)
(***************************************************************************)
LegalActuals(t) ==
    { a \in [Bridges -> 0..MaxTime] :
        \A b \in Bridges :
            IF t <= bsim[b]
            THEN a[b] = Min2(bsim[b] + 1, MaxTime)    \* L1
            ELSE a[b] >= bsim[b] /\ a[b] <= t }       \* full step or O1 cap

(***************************************************************************)
(* One lockstep round: broadcast advance, collect every done.               *)
(***************************************************************************)
AdvanceRound ==
    /\ ktime < MaxTime
    /\ LET t == Target IN
       \E a \in LegalActuals(t) :
        /\ bsim' = a
        /\ reportedMin' = MinOf(a)
        /\ lastTarget' = t
        \* THE MODELLED DESIGN CHOICE:
        /\ ktime' = IF UseMin THEN MinOf(a) ELSE a[Canonical]
        /\ nextTimer' \in (ktime' .. MaxTime)   \* timers reschedule forward

(***************************************************************************)
(* CANDIDATE FIX: a real barrier.  After collecting `done`, the reactor     *)
(* re-sends `advance t` to any socket whose actual < t, until every bridge  *)
(* has reached t.  Termination is guaranteed by the bridge's own O1 comment *)
(* (simavr_bridge.c 3956-3958): the cap is checked AFTER avr_run, so every  *)
(* advance makes >= 1 cycle of progress.  The fixpoint is therefore:        *)
(*   every bridge ends at t, except the L1 case (t <= bsim) -> bsim + 1.    *)
(***************************************************************************)
BarrierRound ==
    /\ ktime < MaxTime
    /\ LET t == Target
           a == [b \in Bridges |->
                    IF t <= bsim[b] THEN Min2(bsim[b] + 1, MaxTime) ELSE t]
       IN
        /\ bsim' = a
        /\ reportedMin' = MinOf(a)
        /\ lastTarget' = t
        /\ ktime' = MinOf(a)
        /\ nextTimer' \in (ktime' .. MaxTime)

Next == IF UseBarrier THEN BarrierRound ELSE AdvanceRound
Spec == Init /\ [][Next]_vars /\ WF_vars(Next)

(***************************************************************************)
(* PROPERTIES                                                              *)
(***************************************************************************)

\* THE invariant that `min(actuals)` (line 582) exists to establish, per its
\* own docstring (468-469): "so klippy never thinks any bridge is ahead of
\* where it actually is".  I.e. klippy's clock must never OVERSHOOT the
\* slowest bridge.  If it does, the pre-advance flush (518-519) computes its
\* send horizon from an eventtime the lagging bridge has not reached, so
\* commands are issued against clocks that bridge considers already past
\* => step-queue underrun / "Timer too close".
NoKlippyOvershoot == \A b \in Bridges : ktime <= bsim[b]

\* The other direction: no bridge may run past klippy's clock, or klippy
\* flushes commands whose req_clock that bridge already considers past.
NoBridgeAhead == \A b \in Bridges : bsim[b] <= ktime

\* Bridges must not drift unboundedly from each other.
BoundedSkew == MaxOf(bsim) - MinOf(bsim) <= Quantum

\* Sanity: klippy's clock is monotonic.
Monotonic == ktime <= MaxTime

=============================================================================
