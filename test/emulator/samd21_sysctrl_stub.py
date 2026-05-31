# Renode Python.PythonPeripheral script for the SAMD21 SYSCTRL block.
# Renode upstream models the region as read-as-zero tags with a single
# 0xFFFF override at PCLKSR (component/sysctrl.h: 0x0C); that gets the
# DFLL open-loop init through but the X32K / DPLL paths in klipper's
# clock.c spin on DPLLSTATUS at 0x50 because nothing flips its
# LOCK / CLKRDY bits. We also want PCLKSR in our (locally rendered)
# .repl to track the actual XOSC32K.ENABLE / DFLLCTRL.ENABLE writes
# rather than always read 0xFFFF, so a Python.PythonPeripheral with
# explicit status synthesis is the cleanest fit - same pattern as
# samd_oscctrl_stub.py / samd_osc32kctrl_stub.py for SAMD51.
#
# Layout (component/sysctrl.h, byte offsets relative to 0x40000800):
#   0x00  INTENCLR  (4 byte)  storeback
#   0x04  INTENSET  (4 byte)  storeback
#   0x08  INTFLAG   (4 byte)  storeback
#   0x0C  PCLKSR    (4 byte)  read-only; bit 1 XOSC32KRDY,
#                             bit 4 DFLLRDY synthesized
#   0x10  XOSC      (2 byte)  storeback
#   0x14  XOSC32K   (2 byte)  storeback (bit 1 ENABLE consumed by PCLKSR)
#   0x18  OSC32K    (4 byte)  storeback
#   0x1C  OSCULP32K (1 byte)  storeback
#   0x20  OSC8M     (4 byte)  storeback
#   0x24  DFLLCTRL  (2 byte)  storeback (bit 1 ENABLE consumed by PCLKSR)
#   0x28  DFLLVAL   (4 byte)  storeback
#   0x2C  DFLLMUL   (4 byte)  storeback
#   0x30  DFLLSYNC  (1 byte)  storeback
#   0x34  BOD33     (4 byte)  storeback
#   0x3C  VREG      (2 byte)  storeback
#   0x40  VREF      (4 byte)  storeback
#   0x44  DPLLCTRLA (1 byte)  storeback (bit 1 ENABLE consumed by DPLLSTATUS)
#   0x48  DPLLRATIO (4 byte)  storeback
#   0x4C  DPLLCTRLB (4 byte)  storeback
#   0x50  DPLLSTATUS (1 byte) read-only; bit 0 LOCK, bit 1 CLKRDY
#                             both synthesized from DPLLCTRLA.ENABLE
#
# klipper-relevant behaviors:
#   - PCLKSR (0x0C) read returns DFLLRDY + XOSC32KRDY synthesized so
#     `while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY))` and
#     `while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY))`
#     both complete the cycle after their respective ENABLE write.
#   - DPLLSTATUS (0x50) read returns 0x3 (LOCK | CLKRDY) once
#     DPLLCTRLA.ENABLE is set. clock_init_32k issues the enable then
#     `while ((DPLLSTATUS & mask) != mask)` for that mask.
#   - All control registers are plain storeback; nothing in the
#     klipper path reads them back beyond the synthesised ready bits.

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    if request.Offset == 0x0C:
        # PCLKSR: synthesise XOSC32KRDY (bit 1) and DFLLRDY (bit 4)
        # from the matching ENABLE writes.
        v = 0
        if regs.get(0x14, 0) & 0x02:  # XOSC32K.ENABLE
            v |= (1 << 1)
        if regs.get(0x24, 0) & 0x02:  # DFLLCTRL.ENABLE
            v |= (1 << 4)
        request.Value = v
    elif request.Offset == 0x50:
        # DPLLSTATUS: bit 0 LOCK, bit 1 CLKRDY synthesised from
        # DPLLCTRLA.ENABLE so the init spin in config_dpll completes.
        if regs.get(0x44, 0) & 0x02:
            request.Value = 0x3
        else:
            request.Value = 0x0
    else:
        request.Value = regs.get(request.Offset, 0)
