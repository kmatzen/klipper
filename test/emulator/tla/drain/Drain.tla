--------------------------------- MODULE Drain ---------------------------------
(***************************************************************************)
(* Abstract model of the Klipper MCU emulator "tick mode" byte pipeline.    *)
(*                                                                          *)
(* Models, from test/emulator/simavr_bridge.c:                              *)
(*   - suart_out_hook (3351-3359): AVR emits a byte into g_suart_tx, and    *)
(*     SILENTLY DROPS it when g_suart_tx is full (the `if` at 3357).        *)
(*   - the tick advance loop (3929-3961): run avr_run until target cycle,   *)
(*     early-exit when g_suart_tx_len >= OUTPUT_CAP (3959, checked AFTER    *)
(*     avr_run so >=1 cycle of progress always happens).                    *)
(*   - suart_feed_one (3458-3470): at most one host byte per avr_run step,  *)
(*     gated on !g_suart_xoff, refilling the staging buffer only when it is *)
(*     empty AND (cycle & 0x1FF)==0.                                        *)
(*   - suart_drain_output (3473-3504): non-blocking write, possibly         *)
(*     partial, KEEP-TAIL memmove of the unwritten remainder (3498-3503).   *)
(*   - host: reactor fully drains the socket before issuing the next        *)
(*     advance (TICK_PROTOCOL_DESIGN.md 2.5), then serialqueue.c            *)
(*     input_event()/input_pos accumulates bytes into whole messages.       *)
(***************************************************************************)
EXTENDS Naturals, Sequences, FiniteSets, TLC

CONSTANTS
    OUTPUT_CAP,     \* O1 cap on g_suart_tx_len (real: 3072)
    TXMAX,          \* sizeof(g_suart_tx) (real: 16384)
    SockCap,        \* socket send-buffer capacity, bridge -> host
    Quantum,        \* target cycles per advance
    MaxEmit,        \* bound: total bytes the AVR ever emits
    MaxHostBytes,   \* bound: total bytes the host ever sends
    HSockCap,       \* socket capacity host -> bridge
    RefillPeriod,   \* abstraction of (cycle & 0x1FF)==0  -> cycle % RefillPeriod == 0
    MaxStride,      \* max cycles one avr_run() step consumes
    MsgLen,         \* bytes per framed message (host reassembly)
    ShortWrite,     \* TRUE: write() may return an arbitrary short count
    HostDrainsFirst \* TRUE: R fully drains the socket before each advance
                    \*       (TICK_PROTOCOL_DESIGN.md 2.5 assumption)

VARIABLES
    phase,      \* "idle" (B0, waiting for advance) | "run" (B1) | "drain" (B2)
    steps,      \* avr_run steps done in the current advance
    cycmod,     \* avr->cycle mod RefillPeriod
    tx,         \* g_suart_tx contents, as a sequence of byte ids
    nextId,     \* next AVR byte id to emit
    dropped,    \* bytes lost at simavr_bridge.c:3357 (tx full)
    sock,       \* bridge -> host socket buffer
    rcvd,       \* everything the host has read (serialqueue input_buf history)
    msgs,       \* framed messages the host has reassembled
    hstage,     \* g_suart_in[g_suart_in_pos .. g_suart_in_len)
    hsock,      \* host -> bridge socket buffer
    nextHId,    \* next host byte id
    fed,        \* bytes actually raised on the AVR RX irq
    xoff        \* g_suart_xoff

vars == <<phase, steps, cycmod, tx, nextId, dropped, sock, rcvd, msgs,
          hstage, hsock, nextHId, fed, xoff>>

Min(a, b) == IF a < b THEN a ELSE b
Range(s)  == [i \in 1..s |-> i]        \* the sequence <<1,2,...,s>>

TypeOK ==
    /\ phase \in {"idle", "run", "drain"}
    /\ steps \in 0..Quantum
    /\ cycmod \in 0..(RefillPeriod - 1)
    /\ nextId \in 1..(MaxEmit + 1)
    /\ nextHId \in 1..(MaxHostBytes + 1)
    /\ dropped \in Nat
    /\ msgs \in Nat
    /\ xoff \in BOOLEAN

Init ==
    /\ phase = "idle"
    /\ steps = 0
    /\ cycmod = 0
    /\ tx = << >>
    /\ nextId = 1
    /\ dropped = 0
    /\ sock = << >>
    /\ rcvd = << >>
    /\ msgs = 0
    /\ hstage = << >>
    /\ hsock = << >>
    /\ nextHId = 1
    /\ fed = << >>
    /\ xoff = FALSE

(*------------------------- B0: reactor issues an advance ----------------*)
(* R only advances once it has fully drained the socket (2.5): sock = <<>>. *)
ReqAdvance ==
    /\ phase = "idle"
    /\ HostDrainsFirst => sock = << >>
    /\ phase' = "run"
    /\ steps' = 0
    /\ UNCHANGED <<cycmod, tx, nextId, dropped, sock, rcvd, msgs,
                   hstage, hsock, nextHId, fed, xoff>>

(*----------------- B1: one loop iteration = feed_one + avr_run ----------*)
Step ==
    /\ phase = "run"
    /\ steps < Quantum
    \* --- suart_feed_one(avr->cycle), simavr_bridge.c:3936 / 3458 ---
    /\ LET doRefill == (~xoff) /\ hstage = << >> /\ cycmod = 0
           stage1   == IF doRefill THEN hstage \o hsock ELSE hstage
           hsock1   == IF doRefill THEN << >> ELSE hsock
           deliver  == (~xoff) /\ stage1 # << >>
           stage2   == IF deliver THEN Tail(stage1) ELSE stage1
           fed1     == IF deliver THEN Append(fed, Head(stage1)) ELSE fed
           xoff1    == IF deliver THEN TRUE ELSE xoff   \* 1-deep RX FIFO
       IN \E stride \in 1..MaxStride, emit \in BOOLEAN :
            /\ emit => nextId <= MaxEmit
            /\ LET full  == Len(tx) >= TXMAX
                   tx1   == IF emit /\ ~full THEN Append(tx, nextId) ELSE tx
                   st1   == steps + 1
               IN /\ tx' = tx1
                  /\ nextId' = IF emit THEN nextId + 1 ELSE nextId
                  /\ dropped' = IF emit /\ full THEN dropped + 1 ELSE dropped
                  /\ steps' = st1
                  /\ cycmod' = (cycmod + stride) % RefillPeriod
                  \* O1 at 3959: checked AFTER avr_run
                  /\ phase' = IF Len(tx1) >= OUTPUT_CAP \/ st1 = Quantum
                              THEN "drain" ELSE "run"
            /\ hstage' = stage2
            /\ hsock' = hsock1
            /\ fed' = fed1
            /\ xoff' = xoff1
    /\ UNCHANGED <<sock, rcvd, msgs, nextHId>>

(* The firmware reads its RX FIFO, clearing XOFF (suart_xon_hook, 3360). *)
AvrConsume ==
    /\ xoff
    /\ xoff' = FALSE
    /\ UNCHANGED <<phase, steps, cycmod, tx, nextId, dropped, sock, rcvd,
                   msgs, hstage, hsock, nextHId, fed>>

(*------------------ B2: suart_drain_output, partial + KEEP-TAIL ---------*)
DrainOut ==
    /\ phase = "drain"
    /\ LET maxk == Min(SockCap - Len(sock), Len(tx))
       IN \E k \in (IF ShortWrite THEN 0..maxk ELSE {maxk}) :
         /\ sock' = sock \o SubSeq(tx, 1, k)
         /\ tx' = SubSeq(tx, k + 1, Len(tx))      \* memmove KEEP-TAIL, 3499
    /\ phase' = "idle"
    /\ UNCHANGED <<steps, cycmod, nextId, dropped, rcvd, msgs,
                   hstage, hsock, nextHId, fed, xoff>>

(*------------------------- Host: R1 read + reassembly -------------------*)
HostRead ==
    /\ phase = "idle"
    /\ sock # << >>
    /\ rcvd' = rcvd \o sock
    /\ sock' = << >>
    /\ UNCHANGED <<phase, steps, cycmod, tx, nextId, dropped, msgs,
                   hstage, hsock, nextHId, fed, xoff>>

(* serialqueue.c input_event(): msgblock_check consumes MsgLen bytes when
   enough have accumulated in input_buf (input_pos accumulation). *)
HostMsg ==
    /\ Len(rcvd) - msgs * MsgLen >= MsgLen
    /\ msgs' = msgs + 1
    /\ UNCHANGED <<phase, steps, cycmod, tx, nextId, dropped, sock, rcvd,
                   hstage, hsock, nextHId, fed, xoff>>

HostWrite ==
    /\ nextHId <= MaxHostBytes
    /\ Len(hsock) < HSockCap
    /\ hsock' = Append(hsock, nextHId)
    /\ nextHId' = nextHId + 1
    /\ UNCHANGED <<phase, steps, cycmod, tx, nextId, dropped, sock, rcvd,
                   msgs, hstage, fed, xoff>>

Next == ReqAdvance \/ Step \/ AvrConsume \/ DrainOut \/ HostRead
        \/ HostMsg \/ HostWrite

Fairness ==
    /\ WF_vars(ReqAdvance)
    /\ WF_vars(Step)
    /\ WF_vars(AvrConsume)
    /\ WF_vars(DrainOut)
    /\ WF_vars(HostRead)
    /\ WF_vars(HostMsg)

Spec == Init /\ [][Next]_vars /\ Fairness

(*=========================== PROPERTIES ================================*)

\* P2. No overflow of g_suart_tx (and the 3357 drop never fires).
NoOverflow == Len(tx) <= TXMAX /\ dropped = 0

\* P2b. The advertised per-advance bound: tx never exceeds OUTPUT_CAP plus
\* one avr_run step's worth of output.
CapBound == Len(tx) <= OUTPUT_CAP + 1

\* P1. No drops / order preserved: what the host has, plus what is in
\* flight in the socket, plus what is still in g_suart_tx, is EXACTLY the
\* sequence of emitted ids 1..nextId-1, in order.
NoDrop == rcvd \o sock \o tx = Range(nextId - 1)

\* the host-visible stream alone is a prefix of the emitted stream
HostPrefix == rcvd = Range(Len(rcvd))

\* host->MCU direction preserves order and loses nothing
NoDropIn == fed \o hstage \o hsock = Range(nextHId - 1)

\* P4. Every emitted byte eventually reaches the host.
EventualDelivery ==
    \A i \in 1..MaxEmit : (nextId > i) ~> (Len(rcvd) >= i)

\* P5. Every host byte eventually reaches the AVR.
EventualFeed ==
    \A i \in 1..MaxHostBytes : (nextHId > i) ~> (Len(fed) >= i)

\* P5b. An advance can complete with host bytes still staged/undelivered.
\* (Checked as an invariant that we EXPECT to fail, exhibiting the trace.)
NoStagedAtIdle == (phase = "idle") => (hstage = << >> /\ hsock = << >>)

\* P5b': at the end of an advance, is g_suart_in guaranteed empty (i.e. did
\* every staged host byte get fed during the advance)?
NoStagedTail == (phase = "idle") => hstage = << >>

=============================================================================
