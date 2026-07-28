# SAME70 EFC (Embedded Flash Controller) read-result stub.
#
# klipper SAME70 firmware (src/atsam/same70_sysinit.c:85) checks
# whether GPNVM bits 7 and 8 (TCM_SIZE) are set on cold boot:
#
#     if ((EFC->EEFC_FRR & GPNVM_TCM_MASK) != GPNVM_TCM_MASK) {
#         /* Configure GPNVM 7 + 8 to enable 128 KiB TCM and reboot. */
#         EFC->EEFC_FCR = ... SGPB FARG 7 ...;
#         EFC->EEFC_FCR = ... SGPB FARG 8 ...;
#         RSTC->RSTC_CR = RST_PARAMS;   // request software reset
#         for (;;) ;                    // spin until reboot
#     }
#
# On real silicon the GPNVM bits persist across reboots, so the second
# boot's GPNVM read returns TCM_MASK and the branch is skipped.
#
# Renode's upstream sam_e70.repl tags the EFC region from the
# ATSAME70Q21 SVD as a generic stub - reads return 0 from the SVD's
# default-values. EEFC_FRR therefore reports `0`, the firmware sees
# `(0 & MASK) != MASK = true`, enters the GPNVM-config branch, hits
# `for (;;)` waiting for a reset RSTC doesn't drive (Renode has no
# RSTC reset model either), and never reaches the application.
#
# This stub overrides the SVD layer at the EFC base (0x400E0C00,
# size 0x10 covering FMR/FCR/FSR/FRR) and synthesizes:
#
#   * FRR (0x0C) -> always returns GPNVM_TCM_MASK (= (1<<7) | (1<<8))
#     so the firmware sees TCM as already-configured and skips the
#     reset-and-spin branch entirely.
#   * FCR (0x04) -> writes are absorbed (no-op). klipper writes here
#     to issue SGPB / etc. commands; with the FRR override above the
#     firmware never needs to read back the command result, so we
#     just swallow.
#   * FMR (0x00), FSR (0x08) -> dict-backed storeback. FMR is set
#     once during init (wait-state config) and never read; FSR's
#     FRDY=1 bit is needed by the (now-skipped) GPNVM-config branch
#     to gate progress between SGPB writes - synthesizing that field
#     too future-proofs the stub if klipper ever drives FCR for
#     non-GPNVM reasons.
#
# Loaded into Renode at startup via a runtime
# `LoadPlatformDescriptionFromString` block in
# renode_launcher.py:_render_resc, same pattern as afec_stub.py.

# Renode's PythonPeripheral re-executes the script body on every
# bus access, so any top-level `regs = {}` resets state per call.
# Guard via `if 'regs' not in dir()` so the dict is created exactly
# once - same idiom samd_storeback.py uses for the SAMD51/RP2040
# storeback regions. Without this the FRDY transition counter is
# wiped before the next FSR read sees it.
if 'regs' not in dir():
    regs = {}

# GPNVM bits 7 and 8 set, matching src/atsam/same70_sysinit.c's
# GPNVM_TCM_MASK = ((1 << 7) | (1 << 8)).
GPNVM_TCM_MASK = (1 << 7) | (1 << 8)

# FSR.FRDY = bit 0. Real silicon transitions FRDY:
#   * 1 (idle) before any command
#   * 0 transiently while a command is executing
#   * 1 again once the command completes
# klipper's atsam/chipid.c read_chip_id() (which produces the USB
# serial-number string) waits for both transitions:
#       wait while FRDY == 0 (idle check)
#       FCR <- GETD_UID command
#       wait while FRDY == 1 (loop until FRDY goes 0)
#       wait while FRDY == 0 (loop until FRDY goes 1)
#       read flash-result words
# A static FRDY=1 deadlocks the second loop. Return FRDY toggling
# pattern: after every FCR write the next FSR read returns FRDY=0
# (command just started), all other reads return FRDY=1 (idle).
# That satisfies both the start-of-command and end-of-command spins
# without modeling actual command timing.
FSR_FRDY = 1 << 0

# Per-call FRDY synthesis state piggybacks on the `regs` dict (which
# Renode preserves across invocations) under a key that can never
# collide with a real EFC offset. Python.PythonPeripheral re-binds
# loose locals on every call, but mutations of module-level mutable
# containers (dicts, lists) are visible across calls - same trick
# scripts/pydev/flipflop.py and the SAMD storeback stubs use.
_FRDY_COUNTER_KEY = -1

if request.IsInit:
    pass
elif request.IsRead:
    offset = request.Offset
    if offset == 0x0C:    # FRR - flash result register
        request.Value = GPNVM_TCM_MASK
    elif offset == 0x08:  # FSR - flash status register
        pending = regs.get(_FRDY_COUNTER_KEY, 0)
        if pending > 0:
            request.Value = 0   # FRDY = 0, command in progress
            regs[_FRDY_COUNTER_KEY] = pending - 1
        else:
            request.Value = FSR_FRDY
    elif offset == 0x00:  # FMR - flash mode register, plain storeback
        request.Value = regs.get(offset, 0)
    elif offset == 0x04:  # FCR - flash command register, write-only on
                          # real silicon; reads return last write.
        request.Value = regs.get(offset, 0)
    else:
        request.Value = regs.get(offset, 0)
elif request.IsWrite:
    regs[request.Offset] = request.Value
    if request.Offset == 0x04:
        # FCR write - schedule one transient FRDY=0 read so the
        # "wait until FRDY drops" loop in read_chip_id observes the
        # transition (the firmware then waits for FRDY=1 again to
        # detect command completion, which the post-counter path
        # serves on the very next read).
        regs[_FRDY_COUNTER_KEY] = 1
