# Renode Python.PythonPeripheral script for the SAMD21 GCLK
# (Generic Clock Generator). Replaces the entire 0x40000C00 region
# (32 bytes) so the SWRST self-clear and SYNCBUSY synthesis don't
# fight with Renode's plain Memory.ArrayMemory default in upstream
# atsamd21j17d-aft.repl - klipper SystemInit writes GCLK_CTRL_SWRST
# then `while (GCLK->CTRL.reg & GCLK_CTRL_SWRST)`, which a plain
# RAM region reads back as set forever.
#
# Layout (component/gclk.h):
#   0x0   CTRL    (1 byte)  - SWRST at bit 0
#   0x1   STATUS  (1 byte)  - SYNCBUSY at bit 7
#   0x2   CLKCTRL (2 byte)  - peripheral clock select / enable
#   0x4   GENCTRL (4 byte)  - generic clock generator control
#   0x8   GENDIV  (4 byte)  - generic clock generator division
#
# klipper-relevant behaviors:
#   - CTRL.SWRST (offset 0x0 bit 0) reads back as 0 after any write:
#     SystemInit's `while (GCLK->CTRL.reg & SWRST)` completes.
#   - STATUS.SYNCBUSY (offset 0x1 bit 7) reads as 0: route_pclock's
#     `while (STATUS & SYNCBUSY)` after writing CLKCTRL completes.
#   - CLKCTRL / GENCTRL / GENDIV: plain store/return. Nothing in the
#     klipper path reads these back to gate progress.

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    if request.Offset == 0x0:
        # CTRL: mask off SWRST so the post-write spin terminates.
        request.Value = regs.get(0x0, 0) & ~0x01
    elif request.Offset == 0x1:
        # STATUS.SYNCBUSY (bit 7) reads as 0.
        request.Value = 0
    else:
        request.Value = regs.get(request.Offset, 0)
