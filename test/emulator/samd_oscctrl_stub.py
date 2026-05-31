# Renode Python.PythonPeripheral script for the SAMD51 OSCCTRL clock
# controller. Renode does not ship an OSCCTRL model, so without this
# stub klipper firmware spins forever in samd51_clock.c waiting for
# XOSCRDY1 / DPLLSTATUS bits that the SVD-derived read-as-zero stubs
# never raise.
#
# Implements only what's needed for klipper SystemInit() to complete:
#
#   - STATUS (offset 0x10): XOSCRDY0/1 (bits 0/1) reflect XOSCCTRL[N]
#     ENABLE; DFLLRDY (bit 8) reflects DFLLCTRLA ENABLE. Used by
#     clock_init_25m() / clock_init_internal().
#   - XOSCCTRL[0/1] (offsets 0x14, 0x18): plain store/return.
#     ENABLE bit (bit 1) drives the matching STATUS bit on read.
#   - DFLLCTRLA (offset 0x1C): plain store/return; ENABLE drives
#     STATUS.DFLLRDY.
#   - DFLLSYNC (offset 0x2C): always 0. klipper waits on
#     `DFLLSYNC.reg & ENABLE/DFLLMUL/DFLLCTRLB/DFLLVAL` after each
#     DFLL register write; reading 0 means "sync done immediately".
#   - Dpll[N].DPLLCTRLA (offsets 0x30, 0x44): plain store/return.
#     ENABLE bit (bit 1) drives DPLLSTATUS.LOCK|CLKRDY.
#   - Dpll[N].DPLLSYNCBUSY (offsets 0x3C, 0x50): always 0.
#   - Dpll[N].DPLLSTATUS (offsets 0x40, 0x54): LOCK|CLKRDY (bits 0,1)
#     set whenever the matching DPLLCTRLA has ENABLE set. klipper
#     spins on `(DPLLSTATUS & (CLKRDY|LOCK)) != (CLKRDY|LOCK)`.
#
# All other OSCCTRL registers (INTENCLR/INTENSET/INTFLAG, DFLLVAL,
# DFLLMUL, DFLLCTRLB, DPLLRATIO, DPLLCTRLB, EVCTRL) are passive
# store/return - klipper writes them but doesn't gate progress on
# their readback values.
#
# See samd_clock_stub notes in rcc_stub.py for IronPython
# PythonPeripheral conventions (PascalCase, module-level state dict).

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    last = regs.get(off, 0)
    if off == 0x10:  # STATUS
        v = 0
        if regs.get(0x14, 0) & (1 << 1):  # XOSCCTRL[0].ENABLE -> XOSCRDY0
            v |= (1 << 0)
        if regs.get(0x18, 0) & (1 << 1):  # XOSCCTRL[1].ENABLE -> XOSCRDY1
            v |= (1 << 1)
        if regs.get(0x1C, 0) & (1 << 1):  # DFLLCTRLA.ENABLE -> DFLLRDY
            v |= (1 << 8)
        request.Value = v
    elif off == 0x2C:  # DFLLSYNC - always done
        request.Value = 0
    elif off == 0x3C or off == 0x50:  # Dpll[N].DPLLSYNCBUSY
        request.Value = 0
    elif off == 0x40:  # Dpll[0].DPLLSTATUS
        if regs.get(0x30, 0) & (1 << 1):
            request.Value = 0x3  # LOCK|CLKRDY
        else:
            request.Value = 0
    elif off == 0x54:  # Dpll[1].DPLLSTATUS
        if regs.get(0x44, 0) & (1 << 1):
            request.Value = 0x3
        else:
            request.Value = 0
    else:
        request.Value = last
