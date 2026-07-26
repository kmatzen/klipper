# Deterministic tick-mode protocol — design & correctness

Implemented by `test/emulator/simavr_bridge.c`,
`test/emulator/renode_launcher.py`, `klippy/chelper/serialqueue.c`,
`klippy/serialhdl.py` and `klippy/reactor.py`. Determinism is *measured* by the
§5.2 trace procedure, not just argued.

The protocol has to be simultaneously:

1. **Deterministic** — for a fixed test (config + gcode + fixtures + firmware
   ELF), every run produces a bit-identical sequence of MCU cycles, message
   exchanges and klippy decisions, independent of host scheduling.
2. **High-throughput** — streaming-sensor tests (`load_cell`: 4 ADCs × 660 SPS
   ≈ 2640 samples/s) run at or above real time.
3. **Correct** — identify, request/response queries, unsolicited streaming and
   trsync homing all work, single- and multi-MCU.
4. **Real-hardware-safe** — with no `KLIPPY_TICK_SOCKET`, the code path is
   byte-identical to upstream `master`.

The tension between 1 and 2 is not fundamental. Making the reactor drive
receive gives determinism but, with a per-output bridge early-exit, chops every
streaming sample into its own round-trip (`load_cell` at 0.15× real time). The
fix is to make the round-trip cadence conditional on whether klippy is actually
waiting for that output — the **bounded quantum** of §2.3. All policy lives
reactor-side; the bridge is a dumb "run to T" stepper.

---

## 1. System model

### 1.1 Participants

- **R** — the klippy **reactor** (single OS thread, cooperative greenlets). The
  *master* of simulated time: it alone decides when and how far to advance.
- **SQ** — the klippy **serialqueue** (C). Frames outgoing commands, parses
  incoming responses, owns the link fd. In tick mode its background poll thread
  is **not started**.
- **B** — the **bridge** main loop (one OS thread) that steps the MCU.
- **CT** — the bridge **control thread**: applies fixtures and runs the setup
  `barrier`. Active only during setup, idle during the test body.
- **MCU** — simavr or Renode; deterministic given (initial state, the cycles at
  which input bytes arrive, the schedule of registered cycle timers).

### 1.2 Channels

- **TICK** — a stream socket R↔B. R writes `advance <T>\n`; B writes
  `done <T'>\n`. One outstanding request at a time.
- **LINK** — the host serial transport. In tick mode it is an **AF_UNIX
  SOCK_STREAM** socket, *not* a pty, on both backends (`suart_setup` /
  `unix_listen_socket` in the simavr bridge; `_TickHostLink` in the renode
  launcher). klippy connects via `serialhdl.connect_unix`, routed by
  `mcu._is_unix_socket`. A socket is used precisely because delivery is
  **synchronous**: klippy's write is readable by B within the `write()`
  syscall, so B reads each flush whole at a deterministic cycle (see C2). A pty
  is not acceptable here — the n_tty line discipline moves slave→master bytes
  via the `flush_to_ldisc` workqueue, so B can catch a flush mid-delivery at a
  wall-clock-dependent split point. Free-run (non-tick) mode keeps the pty.

### 1.3 Clocks

- **sim-time** `t = cycle / frequency` — the only clock R uses for decisions in
  tick mode. `reactor.monotonic()` is the C `get_monotonic()`, which returns
  `*sim_time_ptr` when `KLIPPY_SIM_TIME_FILE` is set — an `mmap` of a `double`
  the bridge writes at the end of every advance. So every `monotonic()` call
  (timer waketimes, `eventtime`, the advance cap) is sim-time by construction.
- **wall-clock** survives only in the `--duration` safety net (§7.4) and
  `_tick_connect`'s 5 s connect deadline (setup, not hot path). No *hot-path
  decision* may depend on it.

### 1.4 Invariant A1: strict alternation

> At any time during the test body, exactly one of {R running, B running} is
> making progress on the MCU/serial state. B steps the MCU only while servicing
> an `advance`; R never reads `done` until it has sent an `advance`. CT is
> quiescent. Neither klippy background thread exists.

A1 is what buys determinism: there is no parallel reader to race the advance.
Any transport that keeps a background reader thread violates A1 — this is why
both backends had to move off the pty.

---

## 2. The protocol state machine — test body

### 2.1 Reactor R, per dispatch iteration

```
R0  RUN-TIMERS  — run all timers with waketime <= eventtime. Timers may queue
                  commands (SQ) and/or schedule timers.
R1  DRAIN-LINK  — while readable (non-blocking): serialqueue_tick_input()
                  (parse + queue), then dispatch every queued message (§2.4).
R2  DECIDE      — if any timer is now due -> R0. Else pick the quantum from
                  NEED_PROMPT() (§2.3) and set
                  T = min(next_timer, eventtime + QUANTUM); go R3.
R3  FLUSH       — serialqueue_flush_ready(eventtime, T): write all commands
                  whose transmit time <= T. Deterministic SEND.
R4  REQUEST     — send "advance T\n" on TICK.
R5  AWAIT       — recv "done T'" on TICK only (B never blocks on the link, so
                  no link read is needed here).
R6  APPLY       — eventtime = T'; go R0. The advance's output is read and
                  dispatched at R1 of the next iteration, at eventtime = T'.
```

`QMAX = _TICK_MAX_QUANTUM = 100 ms` (streaming); `QWAIT = _TICK_WAIT_QUANTUM =
8 ms` (blocked on a reply/trigger). R reads the link only at R1 and always
fully drains it before the next advance, so the buffer is empty whenever B
drains into it (§2.5).

### 2.2 Bridge B, per request

```
B0  WAIT-REQUEST — read a line from TICK; parse "advance T". EOF -> shutdown.
B1  RUN          — target_cycle = floor(T*freq); ENFORCE
                   target_cycle = max(target_cycle, cycle + 1)   [L1, §3]
                   step until cycle >= target_cycle, OR
                   g_suart_tx_len >= OUTPUT_CAP                  [O1, §2.5].
B2  DRAIN        — write g_suart_tx to the link (non-blocking). KEEP-TAIL: on
                   EAGAIN, memmove the unwritten remainder to the front and
                   keep it; it drains on the next B2. Never drop.
B3  REPLY        — publish sim_time; write "done T'", T' = cycle/freq (= T
                   unless O1 capped it early). -> B0.
```

### 2.3 NEED_PROMPT() — the bounded quantum

`NEED_PROMPT()` is true exactly when R is blocked waiting for a specific
firmware event that may arrive before `target`:

```
NEED_PROMPT() := (SQ has un-acked sent commands)   # reply in flight   -> mode 1
              OR (SQ has registered fast-readers)  # trsync homing     -> mode 2
```

`serialqueue_need_prompt()` returns the int mode 0/1/2 (fast-readers takes
precedence); the reactor takes the max across links. The per-mode quantum:

```
mode 0 (streaming)        -> QMAX  (100 ms)
mode 1 (reply in flight)  -> QWAIT (8 ms)
mode 2 (trsync homing):
    single tick socket    -> QMAX  (100 ms)   # single-mcu: 0.25 s watchdog
    multiple tick sockets -> QWAIT (8 ms)     # multi-mcu:  0.025 s watchdog
```

The int — not a bool — is load-bearing: **mode 2 on a single-mcu run uses
QMAX.** A trsync trigger is handled firmware-side (the firmware stops the move
at the exact trigger clock; the host only has to *learn* of it), so the only
constraint on the mode-2 quantum is the trsync keep-alive watchdog. For a
single MCU that is `TRSYNC_SINGLE_MCU_TIMEOUT` (0.25 s), not the 0.025 s
multi-MCU `TRSYNC_TIMEOUT`. Forcing QWAIT through a long single-MCU probe
descent (`load_cell` PROBE / BED_MESH_CALIBRATE) ran at ~0.12× real time and
hit the 180 s wall deadline. QMAX keeps the heartbeat alive with margin: each
`trsync_set_timeout` extension is flushed a `MIN_REQTIME_DELTA` (0.1 s) lead
ahead of its `req_clock` and read at the start of the crossing advance, always
before the 0.25 s deadline. Multi-MCU homing keeps QWAIT (the 0.025 s watchdog
needs a quantum below ~0.0175 s); the tick-socket count is the discriminator.

> **Rejected refinement — do not retry.** Splitting `need_prompt` into
> independent bits so a query overlapping a probe (mode 3) drops back to QWAIT
> was implemented and regressed throughput straight back to the 180 s deadline:
> queries are near-continuous during a `load_cell` probe (clock-sync plus the
> bulk-sensor `_update_clock` cadence), so mode 3 dominated and pinned the
> descent at QWAIT. Per-advance query promptness is not what cures the
> probe-phase clock-sync wobble; the synchronous transport (C2) is.

When neither signal holds, R is advancing toward a scheduled timer and
unsolicited streaming output is not needed this instant — it is pulled at the
next `batch_timer`. So B runs the full quantum and many samples coalesce into
one round-trip.

### 2.4 Message dispatch (deterministic application point)

`serialqueue_tick_pull()` pops parsed messages FIFO; R dispatches each via the
shared `_dispatch_response`. Dispatch happens **only** at R1, i.e. at a
well-defined `eventtime`, on the single reactor thread, never concurrently with
an advance. Fast-readers (trdispatch) run inside `serialqueue_tick_input`'s
`handle_message`, also on the reactor thread (§7.6).

### 2.5 Buffer / drain handling (no overflow, no drops, no deadlock)

A non-blocking drain that drops the tail on EAGAIN loses *unsolicited* samples
(request/response is masked by retransmit). Full-quantum mode makes bursts
bigger, so dropping is unacceptable. Two bridge-local mechanisms, needing no
change to R's await and unable to deadlock:

- **O1 — output cap.** B1 also stops when `g_suart_tx_len >= OUTPUT_CAP` (3072,
  ≤ the smaller of the link buffer and the 4096 read size). The advance returns
  early with `T' < T`; R simply advances again next iteration. Per-advance
  output is bounded well under `g_suart_tx` (16 KiB).
- **KEEP-TAIL drain.** B2 writes what the link accepts and `memmove`s the
  remainder to the front, FIFO-preserved. SQ reassembles a message split across
  advances via its existing `input_pos` accumulation.

Why this is deterministic *and* drop-free:

- **D-PRE (a precondition, not a consequence).** At the start of every advance
  the link buffer is **empty**: R fully drains it (dispatch-loop
  `select(link, 0)` until no POLLIN) before issuing the next `advance`. So B2
  writes into an empty buffer and the common case is a single write with no
  tail. Remove D-PRE and bytes *are* droppable: the socket stays full,
  `suart_drain_output` writes zero bytes per advance, KEEP-TAIL retains the
  tail, and the advance loop still appends ≥1 byte because O1 is checked
  *after* stepping (deliberate, so every advance makes ≥1 cycle of progress).
  `g_suart_tx_len` then ratchets to `sizeof(g_suart_tx)` and bytes are lost.

  D-PRE is enforced structurally by `_check_fds` running before the advance
  branch on every dispatch iteration. It was previously enforced by an `elif`
  (R advanced only when no fd was ready); the §5.1 livelock fix replaced that
  with an explicit drain-then-decide ordering, which preserves D-PRE while
  letting the guard observe fd-ready iterations. **Do not reorder.**
- Any split point is the deterministic buffer boundary against an empty buffer
  (a constant), not a wall-clock race, so the byte stream R sees — and the
  advance at which each byte arrives — is identical across runs.
- No blocking write anywhere, so B never waits on R and there is no deadlock.

> **B-invariant.** Every byte the MCU emits is delivered to R's SQ in order, at
> a deterministic advance, and dispatched at a deterministic `eventtime`. No
> drops.

> **Dispatch point.** The advance's output is dispatched on the iteration
> *after* the advance: `_check_timers(T')` first, then the link read dispatches
> all queued messages at `eventtime = T'`. The order (timers-of-(prev,T'] then
> messages-of-(prev,T']) is fixed and consistent across runs. Klipper tolerates
> it — it is an async system by design.

---

## 3. Connect / identify

Identify is request/response: klippy sends `identify offset=N count=M`, the
firmware replies with a dictionary chunk, ~250 times. SQ always has an un-acked
sent command during identify, so `NEED_PROMPT()` is true and the small QWAIT
quantum applies — the throughput optimization never slows identify.

Two cold-start fixes:

- **stk500v2-leave swallow.** `stk500v2_leave()` writes a 7-byte datagram
  before identify; real boards have a bootloader that eats it, the emulator
  does not, so it corrupted the firmware's first-identify framing. The bridge
  now models the bootloader and swallows exactly that datagram
  (`suart_feed_one`, `STK500V2_LEAVE`).
- **L1.** The FP round trip `t→cycle→t→cycle` can yield
  `target_cycle == cycle`, a zero-cycle advance: B returns `done T` without
  stepping, R re-advances to the same T, livelock (observed as an "Unknown
  message -16 while identifying" stall). B1 enforces
  `target_cycle = max(target_cycle, cycle + 1)`, mirroring the launcher's
  `if delta_us == 0: delta_us = 1000`.

> **Liveness lemma.** With L1 every `advance` increases the cycle count, and the
> firmware's identify response for a given `(offset,count)` is produced after a
> bounded number of cycles; therefore R receives every chunk after finitely many
> advances and identify terminates. No retry or sleep needed. ∎

---

## 4. Setup phase — deterministic fixture application

If the bridge free-runs on wall-clock before tick-connect, CT applies fixtures
(e.g. `spi_ads1220_chip`, which registers a cycle timer) at whatever cycle the
free-run happens to reach. The DRDY pulse train's *phase* relative to the
firmware timeline then varies run-to-run, and a marginal probe-contact
threshold crossing flips — the original `load_cell` flake.

### 4.1 Setup state machine (B + CT)

Replace "free-run until tick-connect" with "advance only for an explicit
barrier; otherwise idle" (`g_barrier_target`, 0 = none):

```
B (pre-tick):
  if tick_client connected -> enter the §2/§3 test-body loop.
  elif g_barrier_target && cycle < g_barrier_target:
        while cycle < g_barrier_target: step()      # TIGHT loop [B1']
        publish sim_time
  else: nanosleep(200us)                            # PAUSED

CT "barrier <usec>":
  g_barrier_target = cycle + usec*freq              # cycle is STABLE here
  wait until cycle >= g_barrier_target
  write "OK"; g_restart_deadline = 1
```

> **B1' — the barrier must be a tight loop.** The runner sends a multi-second
> *simulated* barrier but waits only ~5 s **wall** for the "OK", then launches
> klippy regardless. Advancing one step per main-loop iteration — with an
> `accept()` and a `read()` syscall each step — does not reach a 2 s simulated
> barrier in 5 s wall, so the runner timed out, launched klippy mid-barrier, and
> the cycle at tick-connect became host-timing dependent. The §5.2 proof caught
> this as a seq-0 divergence (~36 k-cycle spread). A tight loop with no per-step
> syscalls reaches the barrier well inside the timeout.

Setup order is: push fixtures → `barrier` → launch klippy → tick-connect. While
fixtures are pushed B is paused at a fixed cycle, so CT registers timers at a
deterministic phase; after the barrier B pauses again, so the cycle at
tick-connect is Σ barriers.

> **Setup-determinism lemma.** Given identical fixtures and barrier sizes, the
> MCU state (cycle, registered-timer phases, RAM) at tick-connect is identical
> across runs, independent of host scheduling. ∎ The only things that move the
> cycle counter pre-tick are barriers, and barriers are serialized by the
> runner's wait-for-OK, so B is paused when CT samples the cycle (C7).

---

## 5. Determinism — invariants & proof obligations

Claim: for fixed (config, gcode, fixtures, ELF) the observable execution — the
`advance`/`done` sequence, the bytes on the link in each direction, the order
and sim-time of every dispatched message, and every klippy decision — is a pure
function of those inputs.

Proof is by induction over dispatch iterations. The base case is the
setup-determinism lemma (§4). For the inductive step, assume identical state
`S_n = (R, SQ, MCU, link-contents, eventtime)` at the start of iteration *n*;
each sub-step maps `S_n` deterministically (R0/R1/R6 are pure functions of
`S_n`; R2's target and quantum are functions of SQ plus the timer heap; R3/R4
are functions of SQ; B1 is deterministic given the cycles at which input bytes
are consumed and the registered timer phases; B2/B3 are functions of the RUN
result). Hence `S_{n+1}` is identical across runs, modulo the conditions below.

**Conditions the implementation must satisfy** (checkable, not faith):

| | Condition |
|---|---|
| **C1** | No wall-clock or RNG in any reactor timer or message-handler *decision*. Largely free: `reactor.monotonic()` *is* sim-time via the mmap (§1.3). Residual audit = direct `time.*` calls used in a hot-path decision. |
| **C2** | Commands are flushed (R3) before `advance` (R4), and input bytes are consumed at deterministic cycles (`suart_refill_input` is gated purely on the cycle count, one byte/cycle). This requires synchronous delivery — hence the AF_UNIX transport (§1.2). With a pty this was the sole residual leak: `proof.sh load_cell` diverged in B's `out_hash` on ~30 % of runs (same cycles, klippy trace identical), and under host load the RX jitter could tip clock-sync into the §5.1(1) wedge. |
| **C3** | Every fixture/firmware cycle timer is phase-deterministic (§4). |
| **C4** | The quantum choice depends only on SQ state. |
| **C5** | Drop-free, empty-buffer drain: KEEP-TAIL plus O1 plus D-PRE (§2.5). |
| **C6** | `--duration` never fires on a healthy run (§7.4). |
| **C7** | Barriers are serialized; B is paused when CT samples the cycle (§4). |
| **C8** | R visits links in a deterministic order (sorted by socket path). |
| **C9** | Every tick branch is under the `KLIPPY_TICK_SOCKET` guard (§8). |
| **C10** | R's clock must not overshoot the slowest bridge (§7.2). |

### 5.1 Steady-state stall (fixed at the source)

Two distinct mechanisms could freeze a tick-mode run until its wall deadline,
distinguished by CPU profile.

**1. Clock-sync timer livelock (klippy ~100 % CPU, sim_time frozen).**
`_check_timers()` returns `0` whenever any timer was due, and the dispatch loop
only advances sim_time on `timeout > 0`. If a timer keeps recomputing a waketime
`<= eventtime` every iteration — clock-sync does this right after a
`Resetting prediction variance` reset off a momentarily-bad frequency —
`_check_timers` re-fires it forever, the advance branch is skipped, and the
fresh clock sample that would fix the estimate can only arrive once sim_time
advances. The stall is self-sustaining. The trigger was host-scheduling RX
jitter, which the synchronous transport removed on both backends.

*Fix — `_tick_decide_advance` / `_TICK_STALL_LIMIT`.* When a timer is overdue
yet sim_time is frozen for `_TICK_STALL_LIMIT` (64) consecutive iterations, the
reactor raises `ReactorError` naming the overdue waketime, the streak, and (for
a parked greenlet) the `pause()` site that is spinning — so a wedge surfaces as
a prompt, attributable failure rather than a deadline-length hang. Fail-fast
rather than self-heal: a silent forced advance can mask a real protocol bug, and
the guard never fires on a healthy run anyway (measured streak high-water 0
across the full gate, against tens of millions on a wedged run).

*Guard reachability — the ordering matters, not the counter.* As first written
the guard was insufficient: `_check_fds` ran in the `if` branch and the guard in
the `elif`, and the fd branch reset the streak unconditionally. But fd readiness
is not sim-time progress, so a link whose fd goes ready more often than once
every 64 iterations reset the streak before it could trip, and the guard could
never fire — the livelock survives the guard. Making the reset conditional on
real progress is also insufficient, because while the guard sat in the `elif` an
fd-ready iteration skipped it entirely. The fix is to run `_check_fds` first
(preserving D-PRE — the guard must never trip before the drain) and then consult
`_tick_decide_advance(timeout, eventtime, after_fds)` on *every* tick iteration.
On an fd-ready iteration it trips only at the stall limit, so healthy runs are
unchanged.

Switching to fail-fast paid for itself immediately: the guard fired
deterministically on `multi_mcu_avr` and the parked-frame diagnosis named
`motion_queuing.py drip_update_time`. The homing drip loop had computed a
positive `wait_time` below half an ulp of `curtime`
(`1.4155343563970746e-15` at `curtime=23.5929159375`), so `curtime + wait_time`
rounded to exactly `curtime`, and the greenlet parked at an already-due waketime
re-deriving the identical wait from the frozen clock forever. Unobservable on
real hardware (wall time advances between iterations), a hard spin under any
frozen-clock regime. The self-heal had been papering over it on every run; it is
now fixed at the source in `motion_queuing.py` by skipping the pause when the
wake time does not land strictly in the future.

**2. Bridge stall (klippy ~0 % CPU, blocked in the lockstep `recv`).** Two
source fixes harden the handshake:

- *renode launcher (`_tick_serve_client`).* An unhandled exception in the tick
  serve thread used to kill the thread silently, leaving the client socket open
  with nobody replying. It now logs the traceback and still replies `done` (no
  virtual-time progress that quantum, which klippy tolerates and re-requests).
- *reactor (`_TICK_RECV_TIMEOUT`).* The lockstep `done` recv is bounded (60 s,
  orders of magnitude above any single capped advance). If a bridge stops
  replying, the reactor logs *which* socket wedged and ends cleanly.

**Diagnostics (ship disabled).** `KLIPPY_TICK_STALL_LOG=1` logs the
overdue-streak high-water mark and a per-5 s "awaiting `done` from socket N"
line while a recv is blocked. `BRIDGE_TICK_DIAG=1` makes both bridges log every
advance read and `done` written, so a stall localises to the side that stopped
issuing work.

### 5.2 Empirical proof procedure

A proof sketch is necessary but not sufficient, so determinism is also measured:

1. `KLIPPY_TICK_TRACE=<path>` appends, per round-trip, `seq T mode actual` on
   the R side and `seq target_cycle end_cycle out_total out_hash dropped` on the
   B side. `out_hash` is a running FNV-1a over every MCU-emitted byte that was
   actually **retained**; `dropped` counts bytes discarded on a full
   `g_suart_tx` and is 0 on any healthy run. The hash deliberately excludes
   dropped bytes — folding them in first made the procedure blind to the very
   failure mode §2.5's drop guard exists to catch, since two runs that both
   dropped data still produced identical `out_total`/`out_hash`.
2. Run a target test **N≥3** times under deliberately varied host load
   (`stress-ng`, a parallel gate, CPU oversubscription). `proof.sh` does this.
3. **Determinism is proven for that test iff all N R-traces are byte-identical
   and all N B-traces are byte-identical.** A divergence localizes the leak to
   the first differing `seq` and says whether it is input (C2), timer phase
   (C3), quantum choice (C4) or dispatch ordering.

Measured on a 16-core x86_64 Linux host with `PYTHONHASHSEED=0`: `temperature`
(174 advances/run), `load_cell` (490 advances/run, deep into the probe phase),
`stm32f103_tick` and `multi_mcu_stm32_stm32` are all byte-identical over 3 runs
on both backends. `PYTHONHASHSEED=0` is set for the tick subprocess in
`test_klippy.py` to pin dict/set iteration order, so timer ordering is
run-to-run identical.

---

## 6. Throughput analysis

Let `QMAX` = 0.1 s and `c_rt` = the round-trip cost (socket rtt plus R's
per-iteration Python), tens of µs of wall time plus the cost of dispatching that
interval's messages.

- **Streaming (mode 0 ⇒ QMAX):** one round-trip per
  `min(QMAX, time-to-next-timer)`. `load_cell`'s `batch_timer` runs about every
  0.1 s ⇒ ~10 round-trips per second of sim time, each carrying ~264 samples
  (2640 SPS × 0.1 s) ≈ 4.5 KiB, split into ⌈4.5/3⌉ ≈ 2 advances by O1. Dispatch
  is the same Python work the background thread did, now amortized over a 0.1 s
  quantum instead of 264 separate advances.
- **Versus one round-trip per sample:** ~10–20 round-trips/s against ~2640, a
  ~130× reduction. Measured: `load_cell` 180 s (deadline) → ~9 s.
- **Identify / queries / homing (QWAIT):** small-quantum cadence, low volume, so
  no buffer pressure.

> **Throughput bound.** round-trips/s ≤ (rate of `send_with_response` calls) ×
> (1/QWAIT while blocked) + (trsync-active fraction × per-trsync-message rate)
> + 1/QMAX. Independent of the ADC sample rate. ∎

Sizing check for C6: with throughput at or above real time, a 25 s-sim
`load_cell` finishes in ~25 s wall, far under the 180 s `--duration`.

---

## 7. Edge cases

**7.1 Cold-start FP livelock.** L1, §3.

**7.2 Multi-MCU.** Each MCU is an independent (TICK, LINK, B). R advances them
to a common `T`, visiting links in sorted order (C8); `NEED_PROMPT()` is maxed
across links. The determinism proof applies per link and cross-link ordering is
fixed by R's iteration order.

*C10 — R must not overshoot the slowest bridge.* `monotonic()` reads the
sim-time mmap of the *canonical* (first) bridge only, but a bridge may reply
`done` short of the target (O1), so the canonical clock is not "the time every
bridge has reached". Computing the flush horizon from it hands the slow bridge
commands whose `req_clock` it already considers past → step-queue underrun /
"Timer too close". Fixed by clamping the target/flush `eventtime` to the
previous round's `min(actuals)` — i.e. by actually consuming the value
`_tick_request_advance` returns. A no-op for a single MCU, where the min *is*
the canonical clock.

*Residual (known, unfixed): C11 — cross-bridge skew is not bounded.* The
converse case, a *fast* bridge running ahead of klippy's clock, is not addressed
by the clamp. Bridges only ever run to `target`, so the skew is bounded by
roughly one quantum and is covered in practice by the `MIN_REQTIME_DELTA`
(0.1 s) pre-transmit lead — but QMAX is also 0.1 s, so the margin is thin.
Relatedly, L1's `+1` fires whenever `target <= cycle`, so a bridge that has run
past the target keeps stepping one cycle per round (~62 ns at 16 MHz) and never
re-converges. The complete fix for both is a true barrier — re-advance laggards
to the same target until all report it. It is **not** implemented here because a
naive retry loop would re-advance without draining the link between retries,
violating D-PRE and reintroducing byte drops; a correct barrier has to
interleave `_check_fds` between retries, which moves the §2.4 deterministic
dispatch point and needs the emulator suite to re-validate.

*Subsumed knobs.* Two multi-MCU margin-wideners are gone.
`KLIPPY_MIN_REQTIME_DELTA` widened the serialqueue pre-transmit lead to mask a
step-queue underrun caused by the old background thread not transmitting
commands due in (prev,T]; the inline pre-advance flush (R3) transmits exactly
those commands at the right cycles, so the underrun cannot occur.
`KLIPPY_TRSYNC_TIMEOUT` widened the multi-MCU trsync *watchdog* — a
heartbeat-latency bound, not a flush issue, so the inline flush does not address
it. What addresses it is the bounded quantum: multi-MCU homing is mode 2 with
more than one tick socket ⇒ QWAIT (8 ms) < 25 ms, and because that bound is in
*simulated* time it is independent of host load. All six `multi_mcu_*` tests
pass with the stock 0.025 s watchdog and 0 "Communication timeout during
homing", so the suite now exercises real watchdog timing instead of an 80×
widener that would have masked a keep-alive regression.

**7.3 EOF / klippy exit.** If R closes TICK, B reads EOF at B0 and shuts down
cleanly.

**7.4 `--duration` safety net.** Must be ≥ worst-case healthy wall time; keep it
only to kill genuine hangs. C6 = "the net never fires on a healthy run". A trace
that ends at the net is a bug to fix, not a flake to retry.

**7.5 Shutdown messages.** An MCU shutdown is unsolicited, so in full-quantum
mode it is delivered at the end of the current quantum (≤ QMAX late in sim time,
deterministically). The test outcome is identical; only R's reaction is
quantized. A test needing tighter latency has a pending send or trsync anyway,
so it is already in QWAIT.

**7.6 trdispatch.** `trdispatch.c` uses a pthread mutex, not a reader thread;
its `fast_reader` callback runs inside `handle_message` on whatever thread reads
the link — in tick mode, the reactor (R1). So homing triggers are applied on the
reactor thread, deterministically. Only the fast-reader *presence* is reused, as
the mode-2 `NEED_PROMPT` signal.

**7.7 eddy / fixed-sample virtual endstops.** If the fixture's eddy I2C sample
is constant, the virtual endstop never crosses threshold during a homing move
and homing cannot trigger. This is a **fixture** gap, not a protocol one: the
fixture must ramp the sample with Z, as `load_cell`'s `probe_step` ramps force
with Z steps (see `eddy_arm.fixture.json`).

---

## 8. Real-hardware path (unchanged)

All tick behavior is gated on `KLIPPY_TICK_SOCKET` being set. With it unset, SQ
starts its background poll thread, serialhdl starts its receiver thread, and
there is no `advance`/`done`, no inline flush and no bounded quantum — i.e. the
path is byte-identical to upstream `master` (C9).
