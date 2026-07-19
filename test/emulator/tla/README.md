# TLA+ models of the tick-mode protocol

Formal models of the lockstep protocol described in
[`../TICK_PROTOCOL_DESIGN.md`](../TICK_PROTOCOL_DESIGN.md). They exist to check
the protocol's *claimed* invariants — determinism, no drops, no deadlock,
liveness — against an exhaustive search rather than against a proof sketch, and
they found real defects that the empirical trace procedure (§5.2) could not.

These are **design-time models, not part of the test gate.** They do not run in
CI and nothing in the build depends on them. Re-run them by hand when changing
the reactor dispatch loop, the bridge advance loop, or the drain path.

## Running

Needs Java and `tla2tools.jar`
([tlaplus releases](https://github.com/tlaplus/tlaplus/releases)):

```sh
cd lockstep
java -cp /path/to/tla2tools.jar tlc2.TLC -workers 4 -deadlock \
     -config Clamp.cfg Lockstep.tla
```

`-deadlock` disables deadlock checking: every model bounds simulated time with a
`MaxTime` constant, so the bound itself registers as a deadlock and would
otherwise mask the real result. Bounds are deliberately tiny (`MaxTime` 3-8,
`StallLimit` 3 instead of 64) to keep the state space exhaustive.

## `lockstep/` — multi-MCU fan-out (design doc §7.2, C10/C11)

Reactor broadcasts `advance T` to N bridges and collects `done T'`. Models the
bridges' two legal short replies: the `L1 +1`-cycle guard and the `O1`
OUTPUT_CAP early cap.

| Config | Models | Result |
|---|---|---|
| `AsBuilt.cfg` | pre-fix: clock from the canonical bridge's mmap | `NoKlippyOvershoot` **violated in 3 steps** |
| `Clamp.cfg` | as shipped: clock clamped to `min(actuals)` | PASS (N=3, exhaustive) |
| `Clamp2.cfg` | same, checking the converse | `NoBridgeAhead` **violated** — the documented residual |
| `Barrier.cfg` | proposed full fix: retry laggards to a common target | PASS on all three invariants |
| `Skew.cfg` | cross-bridge drift | `BoundedSkew` **violated** (C11) |

`AsBuilt` is the bug that motivated the fix: `_tick_request_advance` computed
`min(actuals)` and returned it, but no call site consumed the return value.

`Barrier` is **not implemented** — a naive retry loop would re-advance without
draining between retries, violating §2.5 D-PRE. See §7.2 for why.

## `livelock/` — reactor dispatch-loop liveness (§5.1)

Reactor timers vs. the stall guard, with an environment action that makes an fd
readable (the renode background pty drain).

| Config | Models | Result |
|---|---|---|
| `MCNoGuard.cfg` | guard disabled | liveness **violated** — reproduces the documented livelock; validates the model |
| `MCGuard.cfg` | guard on, no fds | PASS |
| `MCGuardFd.cfg` | guard on, fds arriving, **pre-fix** loop shape | liveness **violated** — the guard never fires |
| `MCGuardFdCondReset.cfg` | reset conditional on progress, guard still in the `elif` | still **violated** |
| `MCAsImpl.cfg` | **as shipped**: drain first, then guard on every iteration | PASS |
| `MCAsImplNoFd.cfg` | as shipped, no fds | PASS |

`MCGuardFd` is the finding: fd readiness is not sim-time progress, so an fd that
goes ready more often than once every `_TICK_STALL_LIMIT` iterations reset the
streak before it could trip. `MCGuardFdCondReset` shows why fixing the counter
alone is not enough — while the guard sat in the `elif`, an fd-ready iteration
skipped it entirely.

`MCGuardFdForceAdv.cfg` is an earlier near-neighbour of the shipped logic, kept
because it isolates the ordering change from the reset change.

## `drain/` — buffer, KEEP-TAIL, reassembly (§2.5)

Bounded tx buffer, partial writes with tail retention, host-side reassembly, and
the XOFF/refill path in the reverse direction.

| Config | Models | Result |
|---|---|---|
| `DrainA.cfg` / `DrainA2.cfg` | faithful, D-PRE holds | PASS — all safety + liveness |
| `DrainD.cfg` | host does not fully drain before advancing | `EventualDelivery` **violated** |
| `DrainE.cfg` | same, more emission budget | `NoOverflow` **violated**, `dropped = 1` |
| `DrainB.cfg` | `avr_run` strides >1 cycle/step | `EventualFeed` violated — see caveat |
| `DrainC.cfg` / `DrainC2.cfg` | adversarial short `write()` | **modelling artifacts**, discount |
| `DrainF.cfg` | staged-tail probe | **incomplete**, the sharpened property was never run |

`DrainD`/`DrainE` are the load-bearing result: they show §2.5's "R fully drains
the pty" is a *precondition* of the no-drop invariant, not a consequence of it.
That is why it is now named D-PRE in the design doc.

`DrainB`'s starvation depends on every `avr_run` step advancing exactly 2
cycles, which real strides (1-4 cycles plus SLEEP fast-forwards) do not sustain.
Treat it as an unbounded-latency hazard, not a proven live deadlock. `DrainC`'s
`CapBound` failure and `DrainC2`'s temporal failure both stem from the model
permitting a perpetual zero-length `write()`, which a real socket with room does
not do.

## Caveats

The models are abstractions written from the source, and are only as good as
that reading. They use small integer time rather than cycles/floats, treat one
lockstep round as atomic where the real reactor interleaves greenlets, and model
`_check_timers` as a set of periodic timers rather than Klipper's real timer
graph. A PASS here means "no counterexample within these bounds under this
abstraction" — it is not a proof about the shipped binary.
