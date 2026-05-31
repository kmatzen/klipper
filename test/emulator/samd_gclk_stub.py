# Renode Python.PythonPeripheral script for the SAMD51 GCLK
# (Generic Clock Generator). Renode upstream models a single GCLK
# PCHCTRL slot via flipflop.py in atsamd51g19a.repl, but klipper's
# enable_pclock() does `while (PCHCTRL[id].reg != val)` and
# flipflop.py alternates 0/0xFFFFFFFF, so the loop never exits.
# We replace the whole GCLK MMIO region with this stub.
#
# Layout (component/gclk.h):
#   0x00       CTRLA       (1 byte)  - SWRST bit 0
#   0x04       SYNCBUSY    (4 byte)  - all "busy" bits
#   0x20-0x4F  GENCTRL[12] (4 byte each)
#   0x80-0x13F PCHCTRL[48] (4 byte each)
#
# klipper-relevant behaviors:
#   - SYNCBUSY (0x04): always 0 so `while (SYNCBUSY & GENCTRL(N))`
#     and `while (SYNCBUSY & SWRST)` complete on first read.
#   - CTRLA / GENCTRL[N] / PCHCTRL[N]: plain store/return.
#     enable_pclock() spins on `PCHCTRL[id] != val` and gen_clock()
#     writes GENCTRL[N] then waits SYNCBUSY; the first is satisfied
#     by store/return, the second by SYNCBUSY=0.

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    if request.Offset == 0x04:  # SYNCBUSY
        request.Value = 0
    else:
        request.Value = regs.get(request.Offset, 0)
