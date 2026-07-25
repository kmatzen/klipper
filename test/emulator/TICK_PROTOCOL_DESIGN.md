# Deterministic, high-throughput tick-mode protocol — design & correctness

Status: **implemented** (`test/emulator/simavr_bridge.c`,
`klippy/chelper/serialqueue.c`, `klippy/serialhdl.py`, `klippy/reactor.py`).
Originally written as a design proposal — *document and prove a fully
functional state machine before more guess-and-check coding* — and then built.
Determinism is **proven** by the §5.2 trace procedure (byte-identical R- and
B-traces across runs). Everything below is grounded in the code and the
experimental findings from the investigation log (PR #8).

> **One revision from the original proposal — the throughput knob.** The
> proposal threw the throughput problem at a *bridge-side early-exit* (`E1`):
> klippy would tag an advance `early`, and the bridge would end the advance the
> instant the firmware's response burst went quiet. The implementation instead
> moves the decision **entirely reactor-side** as a **bounded quantum**: when
> klippy is blocked on a firmware reply/trigger (`NEED_PROMPT`, §2.3) the
> reactor caps the advance at a small `_TICK_WAIT_QUANTUM` (8 ms sim); while
> merely streaming toward a future timer it uses the full `_TICK_MAX_QUANTUM`
> (100 ms sim). The bridge no longer parses `early` and never early-exits on
> output — it just runs to the requested `T` (bounded only by the `O1` output
> cap). This is simpler (the bridge is a dumb "run to T" stepper; all policy is
> in one place, the reactor), achieves the same round-trip bound (§6), and is
> what is actually in the tree. The sections below describe the bounded-quantum
> mechanism; the `E1` framing is kept only where it aids the rationale and is
> flagged as superseded.

The goal of this document is to define a state machine for the klippy↔bridge
tick protocol that is **simultaneously**:

1. **Deterministic** — for a fixed test (config + gcode + fixtures + firmware
   ELF), every run produces a bit-identical sequence of AVR cycles, message
   exchanges, and klippy decisions, independent of host wall-clock scheduling.
2. **High-throughput** — streaming-sensor tests (e.g. `load_cell`: 4 ADCs ×
   660 SPS ≈ 2640 samples/s) run at or above real time, not 0.15×.
3. **Correct** — identify, request/response queries, unsolicited streaming, and
   trsync homing all work, for single- and multi-MCU.
4. **Real-hardware-safe** — with no `KLIPPY_TICK_SOCKET`, the code path is
   byte-identical to today (background thread, wall-clock serial).

It also explains, with a proof sketch and an *empirical* proof procedure, why
the design meets goals 1–4, and it enumerates the conditions the implementation
must satisfy (so we can check them rather than discover them by trial).

---

## 0. Why the naïve attempts failed (motivation)

Two implementations were tried and measured:

| Attempt | Determinism | Throughput | Verdict |
|---|---|---|---|
| **Background thread** (today): C `pollreactor` bg-thread reads the pty, Python receiver thread dispatches, in parallel with the reactor's advances. | **No.** Dispatch races the advance; a response for sim-time *T* may be applied before or after the reactor's timer at *T*, depending on host scheduling. Feature tests (temperature) pass 1/5. | **Yes** (parallel). load_cell runs ≈ real time. | Fast, non-deterministic. |
| **Reactor-driven receive**: disable both threads in tick mode; the reactor reads + dispatches the pty itself after each advance. | **Yes** for delivery (temperature 5/5; bulk samples delivered with correct timestamps). | **No.** Single thread now serializes *advance round-trips* + Python dispatch. With the per-output **early-exit** firing on every sample, load_cell issues ≈ 2640 advances/s → 0.15× real time → hits the 180 s wall deadline. | Deterministic, too slow. |

The tension is **not** fundamental. It was an artifact of pairing the
single-threaded reactor with the bridge's **unconditional per-output
early-exit**, which chops every streaming sample into its own
advance/round-trip. The fix is to make the round-trip cadence *conditional on
whether klippy is actually waiting for that output*. The implementation does
this with a **bounded quantum** chosen reactor-side (§2.3): when klippy is
blocked on a specific firmware reply/trigger, cap the advance at a small
quantum so the reply arrives within one round-trip; when klippy is merely
advancing toward a future timer (the streaming case), use the **full quantum**
so many samples accumulate into one round-trip. (The original proposal achieved
the same effect with a bridge-side `early`-tagged early-exit; the bounded
quantum supersedes it — see the revision note at the top.)

That single change converts load_cell's ≈ 2640 round-trips/s into ≈ 10/s
(one per 100 ms `TICK_MAX_QUANTUM`) while keeping the reactor single-threaded
and therefore deterministic. The rest of this document specifies that protocol
precisely and proves it correct, and pins down the two other determinism leaks
the investigation found (setup-phase timer phase; the cold-start FP livelock).

---

## 1. System model

### 1.1 Participants

- **R** — the klippy **reactor** (single OS thread; cooperative greenlets). The
  *master* of simulated time: it alone decides when and how far to advance.
- **SQ** — the klippy **serialqueue** (C). Frames/queues outgoing commands;
  parses/queues incoming responses; owns the pty fd. In tick mode its
  background poll thread is **not started**.
- **B** — the **bridge** main loop (one OS thread) that steps the AVR.
- **CT** — the bridge **control thread**: applies fixtures and runs the setup
  `barrier`. Active only during setup; idle (blocked on `accept`) during the
  test body.
- **AVR** — simavr; cycle-accurate and **deterministic** given (initial state,
  the cycles at which input bytes arrive, the schedule of registered cycle
  timers).

### 1.2 Channels

- **TICK** — a stream socket R↔B. R writes `advance <T>\n`; B writes
  `done <T'>\n`. One outstanding request at a time (strict alternation).
- **LINK** — the host serial transport. In tick mode it is an **AF_UNIX
  SOCK_STREAM** socket (NOT a pty): B binds+listens (`suart_setup` /
  `unix_listen_socket`) and accepts klippy's connection into `g_suart_master`;
  klippy connects with `serialhdl.connect_unix`. klippy→AVR commands flow into
  B's socket (B reads them via `suart_refill_input`, gated on `cycle & 0x1FF`,
  fed one byte/cycle); AVR→klippy responses go the other way: the AVR's UART-out
  IRQ appends to `g_suart_tx[16384]`; `suart_drain_output()` writes that to the
  socket. A socket is used precisely because its delivery is **synchronous** —
  unlike a pty's slave→master n_tty workqueue, klippy's write is readable by B
  within the `write()` syscall, so B reads each flush whole at a deterministic
  cycle (see C2). (Free-run/non-tick mode keeps simavr's `uart_pty`.)

### 1.3 Clocks

- **sim-time** `t = avr->cycle / avr->frequency`. The *only* clock R uses for
  decisions in tick mode.
- **How R's clock is already sim-time (verified).** `reactor.monotonic` is the C
  `get_monotonic()` (`pyhelper.c:50`). When `KLIPPY_SIM_TIME_FILE` is set,
  `get_monotonic()` returns `*sim_time_ptr` — an `mmap` of a `double` the bridge
  writes with `avr->cycle/freq` at the end of every advance (`simavr_bridge.c`
  ~3290/3322). So **every** `reactor.monotonic()` call (timer waketimes
  `monotonic()+delay`, `eventtime`, the advance `cap`) is sim-time, by
  construction. This is the existing tick infrastructure, not new work; it is the
  reason C1 mostly holds for free.
- **wall-clock** — true wall-clock survives only in: the `--duration` safety net
  (§7.4); `_tick_connect`'s 5 s connect deadline (setup, not hot path); and any
  module that calls Python `time.*` directly instead of `reactor.monotonic()`
  (the residual C1 audit). No *hot-path decision* may depend on it.

### 1.4 Invariant: strict alternation (no concurrency on the hot path)

> **A1.** At any time during the test body, exactly one of {R is running, B is
> running} is making progress on the AVR/serial state. B steps the AVR only
> while servicing an `advance`; R never reads `done` until it has sent an
> `advance`. CT is quiescent. Neither klippy thread (SQ poll, receiver) exists.

A1 is what buys determinism: there is no parallel reader to race the advance.
The background-thread design violates A1; this design preserves it.

---

## 2. The protocol state machine — test body

### 2.1 Reactor R (tick mode), per dispatch iteration

R's existing dispatch loop is: run due timers; service ready fds; else compute
the next wake and sleep. In tick mode "sleep" becomes "advance the AVR." States:

```
R0  IDLE/RUN-TIMERS  — run all timers with waketime <= eventtime.
                       Timers may queue commands (SQ) and/or schedule timers.
R1  DRAIN-PTY        — while PTY readable (non-blocking): serialqueue_tick_input()
                       (parse + queue messages). Then dispatch every queued
                       message (§2.4). (Handles data that arrived as a side
                       effect of a previous advance.)
R2  DECIDE           — if any timer is now due -> R0.
                       else pick the quantum from NEED_PROMPT() (§2.3):
                         QUANTUM = (NEED_PROMPT() ? QWAIT : QMAX)
                       and target T = min(next_timer, eventtime+QUANTUM); go R3.
R3  FLUSH            — serialqueue_flush_ready(eventtime, target): write all
                       commands whose transmit time <= target to the PTY.
                       *(reactor `_tick_flush_callbacks` (reactor.py) ->
                       serialhdl `_tick_flush_cb` -> serialqueue_flush_ready.
                       Deterministic SEND.)
R4  REQUEST          — send "advance T\n" on TICK (no `early` token).
R5  AWAIT            — recv "done T'" on TICK (TICK-only; B never blocks on the
                       pty, see §2.5, so no PTY read is needed here).
R6  APPLY            — set eventtime = T'; go R0. The advance's pty output is
                       read+dispatched at R1 of the next iteration, at
                       eventtime = T' (§2.5 revised-R6).
```

`QMAX = _TICK_MAX_QUANTUM = 100 ms` (streaming); `QWAIT = _TICK_WAIT_QUANTUM =
8 ms` (blocked on a firmware reply/trigger — §2.3). R reads the PTY only at R1
(dispatch loop `select(pty)`), and always fully drains it before issuing the
next advance, so the pty buffer is empty whenever B drains into it (§2.5).

### 2.2 Bridge B (tick mode, test body), per request

```
B0  WAIT-REQUEST   — read a line from TICK. Parse "advance T".
                     (If EOF -> shutdown, §7.3.)
B1  RUN            — set target_cycle = floor(T*freq); ENFORCE
                     target_cycle = max(target_cycle, avr->cycle + 1)   [L1, §7.1]
                     loop avr_run(avr) until avr->cycle >= target_cycle,
                       OR g_suart_tx_len >= OUTPUT_CAP                  [O1, §2.5].
                     AVR-out bytes accumulate in g_suart_tx during the loop.
                     (No early-exit on output: the round-trip cadence is set by
                     the reactor's quantum, §2.3 — B is a dumb "run to T".)
B2  DRAIN          — write g_suart_tx to the master (non-blocking). KEEP-TAIL:
                     on EAGAIN, memmove the unwritten remainder to the front and
                     keep it (do NOT drop); it drains on the next B2. (§2.5 —
                     replaces today's drop-on-EAGAIN.)
B3  REPLY          — publish sim_time; write "done T'" on TICK, where
                     T' = avr->cycle/freq (= T unless O1 capped it early). -> B0.
```

`done T'` means "the AVR ran to T' and its output for [prev,T'] is in
g_suart_tx, the written part already on the pty." With O1 bounding per-advance
output and KEEP-TAIL never dropping, R reads everything across one or a few
advances; no R-side await draining (R5) is required (superseded — see §2.5).

### 2.3 NEED_PROMPT() — the bounded-quantum predicate (throughput knob)

> **Bounded quantum.** Each iteration R caps the advance at `QWAIT` (8 ms sim)
> if `NEED_PROMPT()`, else at `QMAX` (100 ms sim). The bridge always runs to
> the requested `T` (it does not early-exit on output). So a reply/trigger is
> seen within at most one `QWAIT`, while streaming coalesces a full `QMAX` of
> samples per round-trip.
>
> *(Superseded `E1` framing, kept for rationale: the original proposal instead
> tagged the advance `early` and had the bridge end the advance on an
> output-burst-then-quiet. Same goal — bound reply latency without one
> round-trip per sample — but the bounded quantum keeps all policy reactor-side
> and the bridge a dumb "run to T". See the top revision note.)*

`NEED_PROMPT()` is true exactly when R is **blocked waiting for a specific
firmware event that may arrive before `target`**:

```
NEED_PROMPT() := (SQ has un-acked sent commands)        # send_with_response in flight   -> mode 1
              OR (SQ has registered fast-readers)         # trsync/trdispatch homing active -> mode 2
```

`serialqueue_need_prompt()` returns the int mode `0`/`1`/`2` (fast-readers takes
precedence, so a trsync reports `2` even when a query also happens to be in
flight); the reactor takes the max across links. The per-mode quantum is:

```
mode 0 (streaming)         -> QMAX  (100 ms)
mode 1 (reply in flight)   -> QWAIT (8 ms)
mode 2 (trsync homing):
    single tick socket     -> QMAX  (100 ms)   # single-mcu: 0.25 s watchdog
    multiple tick sockets  -> QWAIT (8 ms)      # multi-mcu:  0.025 s watchdog
```

The int — not a bool — is load-bearing: **mode 2 on a single-mcu run uses the
full `QMAX`, not `QWAIT`.** A trsync trigger is handled *firmware-side* — the
firmware stops the move at the exact trigger clock; the host only has to *learn*
of it within a quantum to proceed, which never affects accuracy — so the only
constraint on the mode-2 quantum is the trsync keep-alive watchdog. That watchdog
is `TRSYNC_SINGLE_MCU_TIMEOUT` (0.25 s, `mcu.py`) for a single mcu, not the
0.025 s multi-mcu `TRSYNC_TIMEOUT`, so an 8 ms quantum is ~10× more conservative
than needed there. Forcing `QWAIT` during a long single-mcu probe descent
(load_cell `PROBE` / `BED_MESH_CALIBRATE`) ran the bridge at ~0.12× real time →
the 180 s wall deadline (RC=124). `QMAX` advances the descent at streaming speed
and keeps the heartbeat alive with margin: the `trsync_set_timeout` extension is
flushed a `MIN_REQTIME_DELTA` (0.1 s) lead ahead of its `req_clock` and read by
the firmware at the start of the crossing advance, always before the 0.25 s
deadline. Multi-mcu homing keeps `QWAIT` (the 0.025 s watchdog needs quantum
< ~0.0175 s; the count of tick sockets is the single-vs-multi discriminator).

> **Rejected refinement (don't retry).** Splitting `need_prompt` into independent
> bits so that a query overlapping a probe (mode 3) drops back to `QWAIT` — on
> the theory that a query stuck behind `QMAX` destabilises clock-sync — was
> implemented and **regressed throughput right back to the 180 s wall deadline**:
> queries are near-continuous during a load_cell probe (clock-sync + the
> bulk-sensor `_update_clock` cadence), so mode 3 dominated and pinned the
> descent at `QWAIT`. The probe-phase clock-sync wobble is real but is *not*
> cured by per-advance query promptness; see the **C2** condition note for the
> host-load-dependent stall it can cause and the transport fix that would close
> it.

Both signals already live in SQ:
- *un-acked sent commands*: `send_with_response`/`get_clock`/identify/bulk
  `_update_clock` put a `notify`-tagged message on the sent queue; it clears
  when the matching ack/response arrives. Non-empty ⇒ R will block on a
  completion until the response, which can come well before the completion's
  (far) timeout. The small quantum fetches it promptly. **This is the identify
  and per-batch clock-sync case.**
- *registered fast-readers*: `serialqueue_add_fastreader` is called for the
  duration of a trsync (homing/probe contact, incl. load_cell's
  `trigger_analog`). While present, R is watching for a trigger so it can react
  *after* the move (query the triggered position, proceed to the next probe);
  the move itself is stopped firmware-side at the trigger clock, so the quantum
  bounds only when R *learns* of the trigger, not where the move stops. **This
  is the homing case — `QWAIT` only for multi-mcu (see the per-mode table
  above); single-mcu uses `QMAX`.**

When **neither** holds, R is simply advancing toward a scheduled timer (stats,
`batch_timer`, a dwell). Unsolicited streaming output (ADC samples) is *not*
something R needs this instant — it will be pulled at the next `batch_timer`.
So B runs the **full quantum** and many samples coalesce into one round-trip.

> **Throughput claim (proved in §6):** the number of advance round-trips per
> second of sim time is bounded by the rate of (commands-with-response +
> active-trsync windows) × (1/QWAIT) + 1/QMAX, *not* by the streaming sample
> rate.

### 2.4 Message dispatch (deterministic application point)

`serialqueue_tick_pull()` pops parsed messages FIFO. R dispatches each via the
shared `_dispatch_response` used by the (now-unused-in-tick) receiver thread:
`notify_id ⇒ complete the inflight send`; else look up `(name, oid)` handler.
Dispatch happens **only** at R1 (the dispatch loop's `select(pty)` callback,
`_tick_receive`), i.e. at a well-defined `eventtime`, in the single reactor
thread, never concurrently with an advance. Fast-readers (trdispatch) still run
inside `serialqueue_tick_input`'s `handle_message`, i.e. also on the reactor
thread (no separate trdispatch reader thread is needed; trdispatch's pthread is
only for its mutex — see §7.6).

### 2.5 Buffer / drain handling (no overflow, no drops, no deadlock)

Today `suart_drain_output()` is non-blocking and **drops** the tail on EAGAIN
(`g_suart_tx_len = 0` unconditionally). For request/response that is masked by
retransmit; for **unsolicited** samples a drop is a *lost* sample — exactly the
load_cell failure under big bursts. Full-quantum mode makes bursts bigger (up to
one QMAX of samples), so dropping is unacceptable.

The original idea (R5: R drains the pty inside its `done`-await so B can do a
blocking drain) works but forces a select-on-{TICK,PTY} await with multi-MCU
fan-out. **Superseded** by a simpler, fully bridge-local scheme that needs *no*
change to R's await and cannot deadlock:

- **O1 — output cap.** B1 also stops when `g_suart_tx_len >= OUTPUT_CAP`
  (a constant ≤ the smaller of the pty buffer and the 4096 read size; use
  3072). The advance returns early with `T' < T`; R simply advances again next
  iteration. So per-advance output is bounded by `OUTPUT_CAP + (one avr_run
  step of output)` < 16 KiB (`g_suart_tx`).
- **KEEP-TAIL drain.** B2 writes as much as the pty accepts; on EAGAIN it
  `memmove`s the unwritten remainder to the front and keeps it (no drop). The
  remainder drains on the next advance's B2, FIFO-preserved. SQ reassembles a
  message split across advances via its existing `input_pos` accumulation.

Why this is deterministic *and* drop-free:

- **D-PRE (precondition, not merely rationale).** At the start of every advance
  the pty buffer is **empty**: R fully drains the pty (dispatch-loop
  `select(pty,0)` repeats until no POLLIN) before it ever issues the next
  `advance`. So B2 writes into an empty buffer; with O1, `g_suart_tx_len` at B2
  is ≤ ~OUTPUT_CAP, which fits → the common case is a single `write`, no tail.

  **This is a precondition of the B-invariant below, not a consequence of it.**
  A TLA+ model of this section (`tla/drain/Drain.tla`, config `DrainE`) reaches a byte-dropping
  state as soon as D-PRE is removed: the socket stays full, `suart_drain_output`
  writes zero bytes per advance, KEEP-TAIL retains the tail, and the advance
  loop still appends ≥1 byte per advance because O1 is checked *after* `avr_run`
  (`simavr_bridge.c` — deliberate, so every advance makes ≥1 cycle of progress).
  `g_suart_tx_len` then ratchets to `sizeof(g_suart_tx)` and bytes are lost. The
  model confirms no drops and no overflow *with* D-PRE, and a reachable drop
  *without* it — so any change to the dispatch loop must preserve it.

  D-PRE is enforced structurally by `_check_fds` running before the advance
  branch on every dispatch iteration. Note this was previously enforced by the
  `elif` (R advanced only when no fd was ready); the livelock guard fix (§5.1)
  replaced that with an explicit drain-then-decide ordering, which preserves
  D-PRE while letting the guard observe fd-ready iterations. Do not reorder.
- Any split point is the deterministic pty-buffer boundary against an
  empty buffer (a constant), not a wall-clock race ⇒ the byte stream R sees,
  and the advance at which each byte arrives, are identical across runs.
- No blocking write anywhere ⇒ B never waits on R ⇒ no deadlock; R's await is
  unchanged (TICK-only recv), and the existing dispatch-loop `select(pty)` reads
  the output on the **next** iteration at `eventtime = T'`.

> **B-invariant:** every byte the AVR emits is eventually delivered to R's SQ in
> order, at a deterministic advance, and dispatched at a deterministic
> `eventtime`. No drops. (Replaces the lossy drop-on-EAGAIN; replaces R5.)

> **Dispatch point (revised R6).** With no R5, the advance's output is dispatched
> on the dispatch-loop iteration *after* the advance: `_check_timers(T')` runs
> first, then `select(pty)` → `_tick_receive` dispatches all queued messages at
> `eventtime = T'`. Order is fixed (timers-of-(prev,T'] then messages-of-(prev,
> T']) and consistent across runs ⇒ deterministic. Klipper tolerates this order
> (it is an async system by design); validated empirically (temperature 5/5 in
> the prototype, which already used exactly this dispatch-next-iteration flow).

### 2.6 Sequence diagram (one streaming quantum, NEED_PROMPT=false ⇒ QMAX)

```
iter N:
  R: _check_timers(eventtime); no fd ready; timer in future -> advance
  R: R2 QUANTUM = QMAX (not blocked)      B: B0 read "advance T"
  R: R3 flush (none pending)              B: B1 run until target OR OUTPUT_CAP
  R: R4 send "advance T"        ───────►  B:    (no early-exit on output)
  R: await recv("done T'") ◄────────────  B: B2 drain (one write, <=3KiB) ; B3 done
  R: eventtime = monotonic() = T'         B: B0 wait next
iter N+1:
  R: _check_timers(T')                    (timers due in (prev,T'])
  R: select(pty) READable -> _tick_receive: read+pull+dispatch @eventtime=T'
```

One streaming quantum is split into ⌈4.5KiB / 3KiB⌉ ≈ 2 advances instead of 264
(one per sample). round-trips/s drops ~130× — see §6.

---

## 3. The protocol state machine — connect / identify

Identify is the cold-start phase the original retry loop papered over. It is
request/response: klippy sends `identify offset=N count=M`, the firmware replies
with a dictionary chunk, repeat (~250 chunks).

- During identify, SQ always has an un-acked sent command ⇒ `NEED_PROMPT()` =
  true ⇒ **the small `QWAIT` quantum**. So identify keeps a fast cadence (the
  chunk reply lands within ≤ QWAIT of the request); the throughput optimization
  does not slow it.
- **The cold-start identify framing fix (this PR).** `stk500v2_leave()` writes a
  7-byte stk500v2 datagram before identify; real boards have a bootloader that
  eats it, the emulator does not, so it corrupted the firmware's first-identify
  framing (parser resync → first identify swallowed). The bridge now models the
  bootloader and swallows that exact datagram (`simavr_bridge.c suart_feed_one`,
  `STK500V2_LEAVE`), so the application firmware sees a clean stream. See the
  cold-start memo for the full byte-level analysis.
- **L1 (the cold-start fix).** The FP round trip `t→cycle→t→cycle` can yield
  `target_cycle == avr->cycle`, a zero-cycle advance: B returns `done T` with
  the AVR not stepped, R re-advances to the same T, livelock — observed as the
  ≈11th-test "Unknown message -16 while identifying" stall. B1 enforces
  `target_cycle = max(target_cycle, avr->cycle + 1)`, mirroring
  `renode_launcher.py`'s `if delta_us == 0: delta_us = 1000`. This guarantees
  every advance steps ≥1 cycle ⇒ strictly monotone progress ⇒ identify
  terminates. (Measured: 12 M no-op advances → ≈3 k real advances.)

> **Liveness lemma.** With L1, each `advance` increases `avr->cycle`, and the
> firmware's identify response for a given `(offset,count)` is produced after a
> bounded number of cycles; therefore R receives every chunk after finitely many
> advances and identify completes. No retry/sleep needed. ∎

---

## 4. Setup phase — deterministic fixture application

This is the second determinism leak (the load_cell DRDY-phase flake). Cause:
before tick-connect the bridge **free-runs on wall-clock**; CT applies fixtures
(e.g. `spi_ads1220_chip`, which does
`avr_cycle_timer_register(avr, period_cycles, drdy_assert,…)`) at whatever
`avr->cycle` the free-run happens to be at. So the DRDY pulse train's **phase**
relative to the firmware/step timeline varies run-to-run, and probe-contact
timing (a marginal threshold crossing) flips → flake.

### 4.1 Setup state machine (B + CT)

Replace "free-run until tick-connect" with "advance **only** for an explicit
barrier; otherwise idle." `g_barrier_target` is a `volatile
avr_cycle_count_t`, 0 = none.

```
B (pre-tick):
  if tick_client connected -> enter §2/§3 test-body loop.
  elif g_barrier_target && avr->cycle < g_barrier_target:
        while avr->cycle < g_barrier_target: avr_run(avr)   # TIGHT loop [B1']
        publish sim_time
  else: nanosleep(200us)                           # PAUSED (no wall-clock stepping)

CT "barrier <usec>":
  g_barrier_target = avr->cycle + usec*freq        # avr->cycle is STABLE here
  wait until avr->cycle >= g_barrier_target
  write "OK"; g_restart_deadline = 1
```

> **B1' — the barrier MUST be a tight loop (empirically critical).** The runner
> sends a multi-second simulated barrier (`barrier 2000000` µs) but waits only
> ~5 s **wall** for the "OK" (test_klippy.py:1565), then launches klippy
> regardless. An earlier version advanced one `avr_run` per *main-loop*
> iteration — i.e. with an `accept()` and a `suart_feed_one()` (a `read()`)
> syscall each step — which under that overhead did not reach a 2 s simulated
> barrier within 5 s wall. The runner timed out, launched klippy **mid-barrier**,
> and the AVR cycle (and every fixture-timer phase) at tick-connect became
> host-timing dependent. The §5.2 trace proof caught this as a seq-0 divergence
> (≈2 ms / 36 k-cycle spread). Advancing to the target in a *tight* `avr_run`
> loop with no per-step syscalls reaches the barrier in well under the timeout,
> so klippy always connects at the same deterministic cycle.

Order of setup (unchanged on the runner side): push fixtures → `barrier` →
launch klippy → tick-connect.

- While fixtures are pushed, B is **paused at a fixed cycle** (0 on the first
  barrier, or the previous barrier's target). CT registers the DRDY timer at
  `fixed_cycle + period_cycles` → **deterministic phase**.
- The `barrier` advances a deterministic `usec*freq` cycles (boots the firmware;
  AVR boot is « 1 ms so any barrier ≥ ~1 ms suffices).
- After the barrier B **pauses** (no wall-clock free-run) until tick-connect, so
  `avr->cycle` at tick-connect = Σ barriers — deterministic.

> **Setup-determinism lemma.** Given identical fixtures and barrier sizes, the
> AVR state (cycle, registered-timer phases, RAM) at tick-connect is identical
> across runs, independent of host scheduling. ∎ (Because the only things that
> move the cycle counter pre-tick are barriers, whose start cycle is stable —
> barriers are serialized by the runner's wait-for-OK, so B is paused when CT
> samples `avr->cycle`.)

Barriers requiring `avr->cycle` to be sampled by CT while B might be mid-step
are avoided because the runner sends one barrier at a time and waits for `OK`;
between barriers B is in the paused branch. (Condition C7, §5.)

---

## 5. Determinism — invariants & proof obligations

Claim: **for fixed (config, gcode, fixtures, ELF), the observable execution
(sequence of `advance`/`done` values, the bytes on PTY in each direction, the
order and sim-time of every dispatched message, and every klippy decision) is a
pure function of those inputs.**

Proof is by induction over dispatch iterations, given the conditions below. The
base case is the setup-determinism lemma (§4): identical AVR state and SQ state
at tick-connect.

Inductive step: assume identical global state `S_n = (R-state, SQ-state,
AVR-state, PTY-contents, eventtime)` at the start of iteration *n* on every run.
Each sub-step maps `S_n` deterministically:

- R0/R1/R6 (run timers, dispatch): pure functions of `S_n` (greenlets are
  cooperative; handlers are deterministic). **C1:** no handler/timer may read
  wall-clock or RNG for a *decision*. (Klipper uses `reactor.monotonic()` only
  via `eventtime`, which is sim-time here.)
- R2 DECIDE: `T` and the quantum (NEED_PROMPT mode) are functions of SQ + timer
  heap. Deterministic.
- R3/R4: bytes flushed and the request line are functions of SQ. Deterministic.
- B1 RUN: AVR stepping is deterministic given (AVR-state, the cycles at which
  PTY input bytes are consumed, registered timer phases). **C2:** input bytes
  are consumed at deterministic cycles — `suart_refill_input` is gated purely on
  `avr->cycle` and reads bytes R already flushed *before* sending `advance`
  (so they are present), one byte/cycle. **C3:** all cycle timers (DRDY, sw_uart,
  etc.) were registered at deterministic phases (§4). **C4:** the quantum choice
  (NEED_PROMPT mode) is a pure function of SQ state — deterministic.
- B2/B3: the drained bytes and `T'` are functions of the RUN result.
  Deterministic (B-invariant: nothing dropped, §2.5).
- R5/R6: R reads exactly those bytes, queues, dispatches in FIFO order at
  `eventtime=T'`. Deterministic.

Hence `S_{n+1}` is identical across runs. ∎ (modulo conditions C1–C7)

**Conditions the implementation must satisfy (checklist, not faith):**

- **C1** No wall-clock / RNG in any reactor timer or message handler decision in
  tick mode. **Largely satisfied already:** `reactor.monotonic()` *is* sim-time
  via the `KLIPPY_SIM_TIME_FILE` mmap (§1.3), so anything timing off
  `reactor.monotonic()` is deterministic. Residual audit = grep modules for
  direct `time.time()` / `time.monotonic()` used in a *decision* on the hot path
  and route them through `reactor.monotonic()` under the tick guard.
- **C2** Commands are flushed (R3) *before* `advance` (R4) so they are on the
  host link when B1 consumes them; `suart_refill_input` gating is cycle-only.
  **The async-delivery SOURCE is removed — the host link is now a Unix domain
  socket, not a pty.** The premise "on the link when B1 consumes them" requires
  that klippy's `write()` returning make the bytes immediately readable on B's
  fd. A **pty** does not give this: the n_tty line discipline moves slave→master
  bytes via the `flush_to_ldisc` workqueue, so B's non-blocking `read()` at a
  512-cycle refill could catch a flush mid-delivery and split klippy's byte
  stream at a wall-clock-dependent point — delaying an RX byte by a few hundred
  µs of sim time, jittering the firmware's execution by ±cycles, and (deep in a
  long probe) perturbing a synthesized ADC sample's low bits. **Observed:**
  `proof.sh load_cell` diverged on ~30 % of runs — but ONLY in B's `out_hash`
  (same `out_total`, same cycles); klippy's trace stayed byte-identical and the
  test passed every run (the perturbed bytes were sub-threshold ADC noise the
  SOS filter absorbs). Its one non-benign consequence was that under host load
  the RX jitter could tip clock-sync into the §5.1(1) self-rescheduling-timer
  wedge. **NB: that ~30 % out_hash divergence was measured with the rest of this
  document's determinism machinery present** — reactor-driven receive, bounded
  quantum, barrier-pause setup, `PYTHONHASHSEED=0` — i.e. with C2 as the *sole*
  residual.
  **The transport fix (implemented):** an **AF_UNIX SOCK_STREAM** host link
  (`simavr_bridge.c suart_setup` binds a listen socket via `unix_listen_socket`;
  the main loop `accept()`s it into `g_suart_master`; klippy connects with
  `serialhdl.connect_unix`, routed by `mcu._is_unix_socket`). A stream socket has
  no line discipline: klippy's write lands in the peer receive buffer within the
  `write()` syscall, so B's read sees each flush **whole, at a deterministic
  cycle** — with no waiting (B never blocks for delivery, so the throughput is
  unchanged). This is *not* the rejected length-prefix `advance T N` fix, which
  made B *wait* for async pty delivery on every advance and regressed `load_cell`
  8 s → 180 s; the socket removes the asynchrony at the source rather than
  waiting it out. The transport is gated to tick mode (free-run still uses
  `uart_pty`) and is byte-identical to real hardware off tick mode (C9). (A
  `BRIDGE_HOST_PTY` A/B knob restored the legacy pty during this comparison;
  it was removed once the comparison concluded.) The
  bridge writes the §5.2 `out_hash` B-trace under `KLIPPY_TICK_TRACE`.
  **Proof status (landed).** The reactor-driven receive, bounded quantum,
  barrier-pause setup and `PYTHONHASHSEED=0` are now in the tree alongside the
  socket transport, so the §5.2 byte-identical result is observable. Measured on
  a 16-core x86_64 Linux host with `proof.sh` (3 runs each, `PYTHONHASHSEED=0`): `temperature` is
  **DETERMINISTIC** (174 advances/run; R- and B-traces — incl. the `out_hash` C2
  oracle — byte-identical) and `load_cell` is **DETERMINISTIC** (490 advances/run,
  deep into the probe phase). So the C2 residual (the ~30 % `out_hash` divergence
  the pty's async slave→master delivery caused) is closed by the socket. The
  barrier-pause (C3, §4) was the decisive coarser source: without it the very
  first advance's `target_cycle` varied run-to-run (the AVR cycle at tick-connect
  was wall-clock-dependent), which the §5.2 proof caught as a seq-0 divergence —
  exactly the failure mode this section warned about. (`load_cell.test` still
  fails its *feature* assertion deterministically — the probe-phase SAMPLES=0
  item, §10 Remaining — but its trace is byte-identical, which is what C2
  asserts.)
- **C3** Every fixture/firmware cycle timer is phase-deterministic (§4).
- **C4** The quantum choice (NEED_PROMPT mode) depends only on SQ state.
- **C5** Drop-free, empty-buffer drain: B never drops (KEEP-TAIL) and B1 bounds
  output (O1), and R fully drains the pty before issuing the next advance
  (dispatch-loop `select(pty,0)` to no-POLLIN, reactor.py 447) ⇒ B always drains
  into an empty buffer and any split is at the constant pty-buffer boundary (§2.5).
- **C6** `--duration` never fires during a healthy run (§7.4) — otherwise a
  wall-clock-dependent early termination breaks determinism.
- **C7** Barriers are serialized (runner waits for `OK`); B is paused when CT
  samples `avr->cycle` (§4).

These conditions are *checkable* statically (C1, C2, C4, C5, C7) and by sizing
(C6); C3 is the §4 mechanism. This is the "prove before code" contract: each is
a concrete property to verify, not a hope.

### 5.1 sim_time-wedge steady-state stall (root-caused and fixed at the source)

A steady-state stall — distinct from the cold-start identify race — could
occasionally freeze a tick-mode run: sim_time stopped updating and the test
eventually hit its wall deadline. The whole-test connect retry the runner
carried at the time only ever covered the *cold-start* connect race, so it
never actually re-ran a steady-state stall — the stall just had to be fixed. There were **two distinct mechanisms**,
distinguished by CPU profile; both are now fixed at the source.

1. **Clock-sync timer livelock (klippy ~100 % CPU, sim_time frozen).**
   `_check_timers()` returns `0` whenever any timer was due, and the dispatch
   loop only advances sim_time on `timeout > 0`. If a timer keeps recomputing a
   waketime `<= eventtime` every iteration — clock-sync does this right after a
   `Resetting prediction variance` reset off a momentarily-bad frequency —
   `_check_timers` re-fires it forever, the advance branch is skipped, and
   sim_time never moves; the fresh clock sample that would fix the estimate can
   only arrive once sim_time advances, so the stall is self-sustaining.
   **Trigger:** host-scheduling RX jitter perturbing clock-sync. The socket
   transport removed this for the simavr link; the **renode** link was still an
   async pty drained by a background thread, so renode multi-MCU runs
   (`multi_mcu_avr_stm32`, `…_xmcu`) hit it intermittently. *(Renode now uses
   the same synchronous AF_UNIX host link — see the §7.2 history note — so this
   trigger is gone at the source. The guard below stays: it is cheap, and it
   bounds any future transport that reintroduces a parallel reader.)* Measured: with
   the guard disabled the overdue-timer streak on a wedged renode run climbs
   into the **tens of millions** (sim frozen, spinning to the deadline), whereas
   every healthy run — `temperature`, `load_cell`, all simavr multi-MCU — peaks
   at a streak of **≤ 3**.

   **Fix — reactor `_dispatch_loop` livelock guard (`_tick_decide_advance`,
   `_TICK_STALL_LIMIT`).** When a timer is overdue (`_next_timer <= eventtime`)
   yet sim_time is frozen for `_TICK_STALL_LIMIT` (64) consecutive iterations,
   the reactor raises `ReactorError` naming the overdue waketime and the
   streak, so the wedge surfaces as a prompt, attributable failure instead of
   a deadline-length hang. (As first shipped the guard instead *healed* the
   wedge with a single `+1`-cycle forced advance — minimal so it never ran the
   MCU past klippy's queued steps. That heal predated the synchronous renode
   host link: once the link became an AF_UNIX socket, the only observed
   trigger — the async pty's background reader — was gone at the source and
   the measured streak high-water dropped to 0 across the full gate. The heal
   was then replaced by fail-fast, since a silent self-heal can mask a real
   protocol bug and never fires on a healthy run anyway.)

   The switch to fail-fast paid for itself on its first gate run: the guard
   fired deterministically on `multi_mcu_avr`, and the parked-frame diagnosis
   named `motion_queuing.py drip_update_time` - the homing drip loop computed
   a positive `wait_time` below half an ulp of `curtime`
   (`1.4155343563970746e-15` at `curtime=23.5929159375`), so
   `curtime + wait_time` rounded to exactly `curtime` and the greenlet parked
   at an already-due waketime, re-deriving the identical wait from the frozen
   clock forever. Unobservable on real hardware (wall time advances between
   iterations); a hard spin under any frozen-clock regime. The heal had been
   silently papering over this on every run; it is now fixed at the source in
   `motion_queuing.py` (skip the pause when the wake time does not land
   strictly in the future).

   **Guard reachability (TLA+, `tla/livelock/TickLivelock.tla`, config `MCGuardFd`).** The guard as
   first written was **not sufficient**, for a reason the CPU-profile analysis
   above misses. `_check_fds` ran in the `if` branch and the guard in the
   `elif`, and the fd branch reset `_tick_stall_iters` to 0 unconditionally. But
   fd readiness is *not* sim-time progress: on the renode link — at the time an
   async pty drained by a background thread, i.e. exactly the link this section
   describes as livelocking — an fd can go ready more often than once every 64
   iterations, so the streak was reset before it could ever trip and the guard
   never fired.
   TLC finds a genuine lasso (streak oscillating 0→1→0, sim frozen forever).
   Making the reset conditional on real progress is *also* insufficient: while
   the guard sat in the `elif`, an fd-ready iteration skipped it entirely.

   The fix is an ordering, not a counter tweak: `_check_fds` runs first
   (preserving §2.5 D-PRE — the guard must never trip before the drain), then
   `_tick_decide_advance(timeout, eventtime, after_fds)` is consulted on *every*
   tick iteration. On an fd-ready iteration it trips only at the stall
   limit, so healthy runs are unchanged. Verified in TLC: the wedge cannot
   persist silently with fds arriving; the pre-fix shape lets it.

   The `64` limit sits an order
   of magnitude above the ≤ 3 healthy peak and far below the millions a real
   livelock reaches, so the guard is **inert** on healthy runs (verified:
   `temperature`/`load_cell` traces stay byte-identical, 174 / 490 advances/run;
   max streak 1 / 2). The renode-multi-MCU livelock it originally healed
   (intermittent ~370 s hangs on `multi_mcu_avr_stm32` / `…_xmcu`; 16/16 pass
   with the heal) is now prevented at the source by the synchronous renode
   link; with that trigger gone, the guard's remaining job is to make any
   future wedge fail loudly and diagnosably.

2. **Bridge-stall (klippy ~0 % CPU, blocked in lockstep `recv`).** A bridge could
   stop replying `done` to an `advance`, blocking the reactor's `recv` until the
   test deadline. Two source fixes harden the handshake:
   - **renode launcher (`_tick_serve_client`).** An unhandled exception in the
     tick serve thread used to kill the thread silently, leaving the client
     socket open with nobody replying — an infinite klippy `recv`. It now logs
     the traceback and still replies `done` (no virtual-time progress that
     quantum, which klippy tolerates and re-requests), so a fault surfaces as a
     timely failure, never a hang.
   - **reactor (`_TICK_RECV_TIMEOUT`).** The lockstep `done` recv is now bounded
     (60 s — orders of magnitude above any single capped advance). If a bridge
     genuinely stops replying, the reactor logs *which* socket wedged and ends
     cleanly, turning a silent deadline-length hang into a prompt, diagnosable
     failure.

   The historical trigger here — a cross-thread `pollreactor` timer-plane race
   between `serialqueue_flush_ready` and the renode background thread — was
   already fixed under its own lock (`c6c439813`); during the test body only the
   tick thread touches the renode Monitor socket, so no Monitor race remains. The
   natural `…_xmcu` wedge no longer reproduces (stress: 0 hangs across dozens of
   runs); the two robustness fixes above are the structural safety net.

**Diagnostics (ship disabled).** `KLIPPY_TICK_STALL_LOG=1` makes the reactor log
the overdue-streak high-water mark and a per-5 s
"awaiting `done` from socket N" line while a `recv` is blocked.
`BRIDGE_TICK_DIAG=1` makes both bridges log every advance read and `done` written,
so a stall localises to the side that stopped issuing work.

**Result:** both mechanisms are fixed in the reactor / launcher (real hardware
byte-identical — every change is under the tick guard). The cold-start connect
race itself was also fixed at the source by the barrier-pause setup + the
synchronous AF_UNIX host link (measured: 0 races across 60 cold bridge starts,
vs ~20% per start when the runner-level retry was introduced), so the
whole-test connect retry has been removed from `test_klippy.py` entirely.

### 5.2 Empirical proof procedure (the decisive test)

A proof sketch is necessary but not sufficient; we also *measure* determinism:

1. A `KLIPPY_TICK_TRACE=<path>` mode appends, per round-trip, a CSV line
   `seq T mode actual` on the R side (mode = the NEED_PROMPT mode that chose the
   quantum) and `seq target_cycle end_cycle out_total out_hash dropped` on the B
   side (out_hash = a running FNV-1a over every AVR-emitted byte that was
   actually **retained**; `dropped` counts bytes discarded on a full
   `g_suart_tx`, and is 0 on any healthy run).

   Note the hash deliberately excludes dropped bytes. It previously folded every
   byte in *before* the buffer-full check, which made this procedure blind to the
   one failure mode §2.5's drop guard exists to catch: two runs that both dropped
   data still produced identical `out_total`/`out_hash`, so "byte-identical"
   could not distinguish a clean run from a corrupt one. Byte-identical traces
   now mean identical *delivered* streams. *(Implemented.)*
2. Run a target test (`temperature`, then `load_cell`, then a homing test)
   **N≥3** times under deliberately varied host load (`stress-ng`/parallel gate
   / CPU oversubscription).
3. **Determinism is proven for that test iff all N R-traces are byte-identical
   and all N B-traces are byte-identical.** Any divergence localizes the leak to
   the first differing `seq` and tells us whether it is input (C2), timer phase
   (C3), quantum choice (C4), or dispatch ordering.
4. Keep the trace behind the env flag; it ships disabled.

This converts "is it deterministic?" from a judgment call into a reproducible
pass/fail, which is what the request asks for.

---

## 6. Throughput analysis

Let `f` = AVR freq, `QMAX` = 0.1 s. Round-trip cost ≈ `c_rt` (socket rtt + R's
per-iteration Python) ≈ tens of µs of wall time plus the cost of dispatching the
messages in that interval.

- **Streaming quantum (NEED_PROMPT=false ⇒ QMAX):** one round-trip per
  `min(QMAX, time-to-next-timer)`. load_cell `batch_timer` ≈ every ~0.1 s ⇒ ≈ 10
  round-trips/s of sim. Each carries ~264 samples (2640 SPS × 0.1 s) ≈ 4.5 KiB
  (< 16 KiB `g_suart_tx`; split into ⌈4.5/3⌉≈2 advances by the O1 cap, each a
  single-write drain). Dispatch of 264 samples is one bulk message batch, the
  same Python work the bg-thread did — but now amortized over a 0.1 s quantum
  instead of 264 separate advances.
- **Compared to the failed reactor attempt:** ≈10–20 round-trips/s vs ≈2640 ⇒
  ~130× fewer round-trips. Expected wall time drops from 0.15× real time to well
  above real time (the dispatch work is unchanged; only the round-trip overhead
  is removed). This is what closes the 180 s-deadline gap (measured: load_cell
  180 s → ~9 s).
- **Identify / queries / homing (NEED_PROMPT=true ⇒ QWAIT):** small-quantum
  cadence (reply/trigger seen within ≤ QWAIT). These phases are low-volume, so
  no buffer pressure.

> **Throughput bound:** round-trips/s ≤ (rate of send_with_response calls)×(1/QWAIT
> while blocked) + (trsync-active fraction × per-trsync-message rate) + 1/QMAX.
> Independent of the ADC sample rate. ∎

Sizing check for C6: with throughput ≥ real time, a 25 s-sim load_cell test
finishes in ≤ ~25 s wall ≪ 180 s `--duration`. Headroom is large; C6 holds.

---

## 7. Edge cases

### 7.1 Cold-start FP livelock — L1 (§3). Already validated in isolation.

### 7.2 Multi-MCU. Each MCU is an independent (TICK, PTY, B). R advances them in
a fixed order each iteration (sorted by socket path) to a common `T`; it sends
all `advance` then awaits all `done` (or does them sequentially — sequential is
simpler and still deterministic, since A1 holds per link and R imposes a total
order). `NEED_PROMPT()` is OR'd (max) across links. Determinism proof applies
per link; cross-link ordering is fixed by R's iteration order. **C8:** R visits
links in a deterministic order.

**C10 — the reactor's clock must not overshoot the slowest bridge.** `monotonic()`
reads the sim-time mmap of the *canonical* (first) bridge only, but a bridge may
reply `done` short of the target (O1 output cap), so the canonical clock is not
"the time every bridge has reached". `_tick_request_advance` computed
`min(actuals)` and returned it with a docstring saying it exists "so klippy never
thinks any bridge is ahead of where it actually is" — but **no call site ever
consumed the return value**; all three only test `is None`. TLC violates the
invariant in 3 steps (`tla/lockstep/Lockstep.tla`, config `AsBuilt`): canonical reaches the target,
a second bridge caps early, klippy adopts the canonical clock and flushes against
a horizon the slow bridge has not reached → its commands carry a `req_clock` that
bridge already considers past → step-queue underrun / "Timer too close".

Fixed by clamping the target/flush `eventtime` to the previous round's
`min(actuals)` — i.e. by actually using the value. No-op for a single mcu, where
the min *is* the canonical clock. Verified: `NoKlippyOvershoot` holds
exhaustively at N=3.

**Residual (known, unfixed).** The converse — a *fast* bridge running ahead of
klippy's clock — is not addressed by the clamp, and TLC still violates
`NoBridgeAhead` under it. Bridges only ever run to `target`, so the skew is
bounded by roughly one quantum and is covered in practice by the
`MIN_REQTIME_DELTA` (0.1 s) pre-transmit lead; but QMAX is also 0.1 s, so the
margin is thin. The complete fix is a true barrier — re-advance laggards to the
same target until all report it — which TLC shows restores `NoKlippyOvershoot`,
`NoBridgeAhead`, *and* `BoundedSkew` at N=3. It is **not** implemented here
because a naive retry loop would re-advance without draining the link between
retries, violating §2.5 D-PRE and reintroducing byte drops; a correct barrier has
to interleave `_check_fds` between retries, which moves the §2.4 deterministic
dispatch point and needs the emulator suite to validate.

**C11 — cross-bridge skew is not bounded.** L1's `+1` fires whenever
`target <= avr->cycle`, so a bridge that has already run past the target keeps
stepping one cycle per round and never re-converges; `BoundedSkew` fails in TLC.
One cycle per advance (~62 ns at 16 MHz) is slow enough not to matter in a test
run, but it does not self-correct. Subsumed by the barrier above if implemented.

**Knobs — what was actually subsumed.** The pre-determinism path carried two
multi-MCU margin-widening env knobs (PART 12).

- `KLIPPY_MIN_REQTIME_DELTA` — **dropped (subsumed).** It widened the
  serialqueue pre-transmit lead to mask a step-queue *underrun*: the old C
  bg-thread saw frozen sim_time during an advance and didn't transmit commands
  due in (prev,T]. The inline pre-advance flush (R3 — `serialqueue_flush_ready`
  before every `advance`) transmits exactly those commands at the right cycles,
  so the underrun cannot occur. The knob is gone from `test_klippy.py`.
- `KLIPPY_TRSYNC_TIMEOUT` — **also dropped (NOT an underrun — a watchdog).** This
  is the multi-MCU trsync *watchdog* (default 0.025 s): the host's trsync
  keep-alive must reach each MCU within that window or homing aborts with
  "Communication timeout during homing". It is a heartbeat-*latency* bound, not
  a command-flush issue, so the inline flush does not address it (the original
  §7.2 text wrongly lumped it with the underrun). What addresses it is the
  bounded quantum: during *multi-mcu* homing a trsync fast-reader is registered →
  `NEED_PROMPT()` = mode 2 → with more than one tick socket the reactor caps the
  advance at `QWAIT` (8 ms) < 25 ms, so the keep-alive's latency in *simulated*
  time stays inside the watchdog — and because that bound is in sim time it is
  independent of host load. (A *single*-mcu run has the 0.25 s
  `TRSYNC_SINGLE_MCU_TIMEOUT` watchdog instead, so mode 2 there uses `QMAX` for
  throughput — see §2.3.) **Verified:** all six `multi_mcu_*` tests (incl. the 5-homing `_xmcu`
  variants and the non-tick `multi_mcu_linuxprocess_avr`) pass with the stock
  0.025 s watchdog, 0 "Communication timeout during homing". The 2.0 s widener
  is removed from `test_klippy.py`, so the suite now exercises the real watchdog
  timing (an 80× widener would have masked a genuine keep-alive-latency
  regression).

To confirm determinism: the empirical trace (§5.2) on `multi_mcu_stm32_stm32`
is byte-identical *and* free of "Timer too close" with `MIN_REQTIME_DELTA` at
its default. **Both PART-12 multi-MCU knobs are now gone; the §7.2 claim holds.**

> **HISTORY — this claim was false when written, and is now true.** As
> originally committed the byte-identical half did not reproduce: measured on a
> 16-core x86_64 Linux host, `proof.sh multi_mcu_stm32_stm32 3` diverged on
> **every** attempt (0 byte-identical / 3 diverged, repeated 3×), as did
> `proof.sh stm32f103_tick 3`. It reproduced identically on unmodified code, so
> it was pre-existing rather than a regression.
>
> **Root cause: the renode link violated A1.** `tick_mode` is gated on
> `SQT_PIPE` (`serialqueue.c`). The renode host link was a **pty** (`SQT_UART`),
> so it kept `serialhdl`'s background reader thread — a parallel reader racing
> the advance, exactly what invariant A1 (§1.4) forbids, and A1 is what buys
> determinism. The simavr link had already been moved to a synchronous AF_UNIX
> socket; renode had not. That asymmetry, not anything about Renode itself, was
> the whole defect. It also explains the §5.1 observation that only *renode*
> multi-MCU runs hit the clock-sync livelock: they were the only links with a
> background reader left to perturb clock-sync.
>
> **Fix:** the launcher now binds an AF_UNIX `SOCK_STREAM` at `slave_link`
> instead of opening a pty (`_TickHostLink`, `renode_launcher.py`). In tick mode
> the launcher already bypassed `CreateUartPtyTerminal` and shuttled bytes
> itself, so it owned both ends and this is a transport swap, not a protocol
> change. No klippy-side change was needed: `mcu.py:_is_unix_socket()` sees
> `S_ISSOCK`, routes to `connect_unix()` → `serial_fd_type 'p'` → `tick_mode` on
> → reactor-driven receive, no background thread. Writes use the same KEEP-TAIL
> discipline as `suart_drain_output` (§2.5) — blocking would deadlock, since
> klippy is awaiting `done` and therefore not reading while an advance runs.
>
> **Verified after the fix** (same host): `stm32f103_tick`,
> `multi_mcu_stm32_stm32` and `temperature` are all byte-identical over 3 runs,
> with the full 53-test gate green. §5.2's proof procedure now covers **both**
> backends, which is what it always claimed.

### 7.3 EOF / klippy exit. If R closes TICK (klippy done/killed), B reads EOF at
B0 and shuts down cleanly. The `start_ts`/`--duration` restart on the setup
barrier and on tick-connect (existing PART 13 fix) stays.

### 7.4 `--duration` safety net. Must be ≥ worst-case healthy wall time. With §6
throughput, healthy runs finish far under the default; keep the net only to kill
genuine hangs. **C6** = "net never fires on a healthy run." If a trace ever ends
at the net, that's a bug to fix, not a flake to retry.

### 7.5 Shutdown messages. An MCU shutdown is an unsolicited message; in
full-quantum mode it is delivered at the end of the current quantum (≤ QMAX
late in sim time, deterministically). Acceptable: the test outcome is identical
(the shutdown still happens at the same sim-time message-wise); only R's
reaction is quantized to the quantum boundary. If a test needs tighter shutdown
latency it will have a pending send/trsync anyway (NEED_PROMPT=true ⇒ QWAIT).

### 7.6 trdispatch. `trdispatch.c` uses a pthread **mutex**, not a reader
thread; its `fast_reader` callback runs inside `handle_message` on whatever
thread reads the pty — in tick mode, the reactor (R1). So homing triggers are
delivered and applied on the reactor thread, deterministically. No separate
synchronous-trdispatch conversion is required (an earlier worry); only the
fast-reader presence is reused as the `NEED_PROMPT` homing signal (§2.3).

### 7.7 eddy / fixed-sample virtual endstops. The eddy probe's I2C sample in the
fixture is *constant*, so the virtual endstop never crosses threshold during a
homing move and homing can't trigger. This is a **fixture** gap, independent of
the protocol: the fixture must ramp the eddy sample with Z (as load_cell's
`probe_step` ramps force with Z steps). Tracked separately; not part of the
state machine, but listed so the validation suite accounts for it.

---

## 8. Real-hardware path (unchanged)

All tick behavior is gated on `KLIPPY_TICK_SOCKET` being set AND the serial fd
being a UART (`SQT_UART`). With it unset:

- SQ starts its background poll thread (today's code).
- serialhdl starts its receiver thread (today's code).
- No `advance/done`, no inline flush, no bounded-quantum / NEED_PROMPT.

So production and real-MCU regression are byte-identical to current `master`.
**C9:** every tick branch is under that guard; no shared-path behavior changes.

---

## 9. Implementation mapping (what actually changes)

### 9.0 Current code state (implemented; verified by reading the tree)

**The deterministic-SEND + clock half:**
- sim-time clock: `pyhelper.c get_monotonic` reads `KLIPPY_SIM_TIME_FILE` mmap →
  `reactor.monotonic()` = sim-time (C1).
- advance/done round-trip: `reactor._tick_request_advance` (R2/R4 + a
  TICK-socket-only await).
- inline pre-advance flush (R3): `reactor._tick_flush_callbacks` ←
  `serialhdl._tick_flush_cb` → `serialqueue_flush_ready`.
- reactor-driven receive: `serialhdl._tick_receive` +
  `serialqueue_tick_input`/`_tick_pull`, registered via `register_fd`; the bg
  poll + receiver threads are not started in tick mode.

**The throughput + determinism deltas (all implemented):**
- **Bounded quantum (replaces the proposal's bridge `E1` early-exit).**
  `serialqueue_need_prompt()` (mode 0/1/2) → `serialhdl._tick_need_prompt_cb` →
  `reactor._tick_request_advance` ORs it across links and caps the advance at
  `_TICK_WAIT_QUANTUM` (8 ms) when ≥1, else `_TICK_MAX_QUANTUM` (100 ms). The
  reactor sends a bare `advance T` (no `early`); the bridge's `early` parse and
  early-exit were **removed** (it now just runs to T, bounded by O1).
- **O1 output cap + KEEP-TAIL drain** in `simavr_bridge.c` (replaces the lossy
  drop-on-EAGAIN; bounds per-advance output).
- **Paused/barrier setup** in `simavr_bridge.c` (`g_barrier_target`, tight loop)
  replaces the pre-tick wall-clock free-run → deterministic timer phases.
- **L1 +1 cycle guard** in `simavr_bridge.c` (cold-start FP livelock).
- **stk500v2-leave swallow** in `simavr_bridge.c suart_feed_one` (cold-start
  identify framing — models the bootloader; this PR).
- `PYTHONHASHSEED=0` for the klippy tick subprocess (`test_klippy.py`) —
  pins dict/set iteration order so timer ordering is run-to-run identical.
- **Livelock guard + recv safety net** in `reactor.py` (§5.1): a fail-fast
  `ReactorError` after `_TICK_STALL_LIMIT` overdue-with-frozen-sim iterations
  (mechanism 1), and a `_TICK_RECV_TIMEOUT`-bounded lockstep recv that names a
  wedged bridge instead of hanging (mechanism 2). Plus the renode launcher's
  `_tick_serve_client` no longer dies silently. Both fix the steady-state wedge
  at the source, so it no longer needs the retry.
- The runner-level connect retry (`_emulator_connect_flake` /
  `EMULATOR_CONNECT_RETRIES`) is removed: the cold-start race it patched is
  fixed at the source by the barrier + synchronous host link (measured 0/60),
  and the §5.1 steady-state wedge is fixed at the source too.

(The `register_fd` receive callback + dispatch-next-iteration flow is correct
as-is — no R5 await-drain is needed, see §2.5.)

### 9.1 Change table (as built)

| Component | Change |
|---|---|
| `simavr_bridge.c` B1 | **L1** (`target_cycle = max(target_cycle, cycle+1)`). |
| `simavr_bridge.c` parse | Parse bare `advance T` (the `early` token + bridge-side E1 early-exit were **removed** — superseded by the reactor bounded quantum). |
| `simavr_bridge.c` B1 loop | Run to target; **O1** output cap (stop at `g_suart_tx_len >= OUTPUT_CAP`). No output early-exit. |
| `simavr_bridge.c` B2 | **KEEP-TAIL** drain: memmove unwritten remainder, never drop. |
| `simavr_bridge.c` setup | Pre-tick **paused/barrier** loop (§4); `g_barrier_target`. |
| `simavr_bridge.c` suart | **stk500v2-leave swallow** (cold-start framing, models the bootloader). |
| `serialqueue.c` | `serialqueue_need_prompt()` → 0/1/2 = (streaming)/(un-acked sends)/(fast-readers). |
| `chelper/__init__.py` | cdef for `serialqueue_need_prompt`. |
| `serialhdl.py` | `_tick_need_prompt_cb` registration; reactor-driven `_tick_receive`; no bg/receiver thread in tick mode. |
| `reactor.py` | `_tick_request_advance` ORs `need_prompt` across links → caps the advance at `_TICK_WAIT_QUANTUM` (≥1) else `_TICK_MAX_QUANTUM`; sends bare `advance T`. |
| `reactor.py` | **§5.1 mechanism (1):** `_tick_decide_advance` livelock guard (`_TICK_STALL_LIMIT` overdue-with-frozen-sim iterations → fail-fast `ReactorError`). |
| `reactor.py` | **§5.1 mechanism (2):** `_TICK_RECV_TIMEOUT`-bounded lockstep `done` recv (names the wedged socket + ends, vs. silent hang). `KLIPPY_TICK_STALL_LOG` diag. |
| `renode_launcher.py` | **§5.1 mechanism (2):** `_tick_serve_client` catches any RunFor exception, logs it, and still replies `done` (no silent thread death). `BRIDGE_TICK_DIAG` advance/done diag. |
| `simavr_bridge.c` | `BRIDGE_TICK_DIAG` advance-read / done-written diag (§5.1). |
| both sides | `KLIPPY_TICK_TRACE` per-round-trip CSV (§5.2), env-gated. |
| `test_klippy.py` | `PYTHONHASHSEED=0` for the tick subprocess. (The one-time whole-test connect retry is gone - the cold-start race is fixed at the source by the barrier + synchronous host link.) |
| (fixtures) | eddy ramped-sample fixture (§7.7) — separate. |

---

## 10. Open questions — resolved

1. **`need_prompt` exactness.** "(un-acked sends) || (fast-readers present)" has
   held across the feature suite — the §5.2 traces are byte-identical and no
   test needed prompt delivery it didn't get. (Backstop stands: a test that
   needed prompt delivery but got the full quantum would diverge/stall and point
   right at the missing signal.)
2. **Multi-MCU advance discipline.** Sequential `advance` to a common `T`,
   links visited in sorted order (C8). Fast enough for `multi_mcu_*`.
3. **`monotonic` in tick mode (C1).** No hot-path decision reads true wall-clock;
   `reactor.monotonic()` is sim-time via the mmap.
4. **pty buffer size.** With the O1 cap (3072 < pty buffer), B2 single-writes in
   the common case; KEEP-TAIL covers the rest (correctness independent of size).

**Status:** §10 resolved; the §5.2 proof is byte-identical (measured on a 16-core x86_64 Linux host:
`temperature` 174 advances/run and `load_cell` 490 advances/run, R+B traces
identical over 3 runs — see the C2 "Proof status (landed)" note; both unchanged
with the §5.1 livelock guard present, confirming it is inert off the wedge). The
§5.1 steady-state wedge is now fixed at the source (the reactor livelock guard +
bounded recv + the renode serve-thread hardening), and the cold-start connect
race is fixed at the source as well (barrier + synchronous host link; measured
0 races / 60 cold starts), so the runner carries no retry at all.

> **Remaining (tracked, not protocol):** `load_cell.test` still fails
> deterministically (collector `SAMPLES=0` at PROBE) — a per-test feature-timing
> calibration item, downstream of a clean connect; see §7.7's sibling note. The
> protocol/determinism layer is complete.
```
