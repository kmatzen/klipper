# Renode Python.PythonPeripheral script for the RP2040 PLL register
# blocks at 0x40028000 (PLL_SYS) and 0x4002C000 (PLL_USB). Same layout
# per regs/pll.h, four registers each:
#   - CS        (0x00) - REFDIV (bits 0..5) RW + LOCK (bit 31) RO +
#                         BYPASS (bit 8) RW.
#   - PWR       (0x04) RW - PD (bit 0), DSMPD (bit 2), POSTDIVPD (bit
#                            3), VCOPD (bit 5). All 1 at reset (PLL
#                            powered down).
#   - FBDIV_INT (0x08) RW - feedback divider.
#   - PRIM      (0x0C) RW - POSTDIV1 (bits 16..18), POSTDIV2 (bits
#                            12..14).
#
# klipper's pll_setup() in src/rp2040/main.c does (per PLL):
#   pll->cs        = refdiv;
#   pll->fbdiv_int = fbdiv;
#   pll->pwr       = DSMPD_BITS | POSTDIVPD_BITS;       // clears PD
#   while (!(pll->cs & PLL_CS_LOCK_BITS)) ;
#   pll->prim      = POSTDIV1 | POSTDIV2;
#   pll->pwr       = DSMPD_BITS;                         // clears POSTDIVPD
#
# The stub synthesizes CS.LOCK (bit 31) once PWR.PD (bit 0) has been
# cleared. On real silicon this takes a few hundred microseconds while
# the analog VCO settles; in emulation we present the lock immediately,
# which is what every other clock-controller stub in this directory
# does (lpc_sc_stub.py PLL0STAT, samd_oscctrl_stub.py PLLSTATUS).
# The same script services both PLL_SYS and PLL_USB - regs is per-
# instance because Renode gives each Python.PythonPeripheral its own
# ScriptScope.

if 'regs' not in dir():
    regs = {}

_PWR_PD_BIT = 1 << 0
_CS_LOCK_BIT = 1 << 31

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    if off == 0x00:  # CS
        v = regs.get(0x00, 0)
        pwr = regs.get(0x04, _PWR_PD_BIT)  # default = powered down
        if not (pwr & _PWR_PD_BIT):
            v |= _CS_LOCK_BIT
        request.Value = v
    else:
        request.Value = regs.get(off, 0)
