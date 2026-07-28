# Renode Python.PythonPeripheral script for plain store/return
# register regions. Used for SAMD51 MCLK and CMCC where klipper
# writes register values and reads them back unchanged - no status
# bits or busy flags need synthesizing. Renode upstream's
# pydev/flipflop.py alternates 0/0xFFFFFFFF, which is wrong for
# this case: klipper's enable_pclock() does
# `(&MCLK->APBAMASK.reg)[pm_port] |= pm_bit;` (read-modify-write)
# and reads the bits back when later code re-reads the same register.
#
# Generic enough to reuse for any "trivial RAM-backed peripheral"
# situation where the firmware just needs writes to be observable on
# subsequent reads at the same offset.

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    request.Value = regs.get(request.Offset, 0)
