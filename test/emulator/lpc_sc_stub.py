# Renode Python.PythonPeripheral script for the LPC176x System
# Control (LPC_SC) register block. Renode does not ship an LPC176x
# clock controller model, so without this stub klipper SystemInit
# (lib/lpc176x/device/system_LPC17xx.c) spins forever waiting for
# OSCSTAT / PLOCK0 / PLLE0_STAT / PLLC0_STAT bits.
#
# Layout (LPC17xx user manual + lib/lpc176x/device/LPC17xx.h):
#   0x000 FLASHCFG
#   0x080 PLL0CON       - bit 0 PLLE0, bit 1 PLLC0 (write 0x01 then 0x03)
#   0x084 PLL0CFG
#   0x088 PLL0STAT      - bit 24 PLLE0_STAT, 25 PLLC0_STAT, 26 PLOCK0
#   0x08C PLL0FEED
#   0x0A0 PLL1CON       - same layout as PLL0CON (USB PLL)
#   0x0A4 PLL1CFG
#   0x0A8 PLL1STAT      - same layout as PLL0STAT
#   0x0AC PLL1FEED
#   0x0C4 PCONP         - peripheral clock power control
#   0x104 CCLKCFG
#   0x108 USBCLKCFG
#   0x10C CLKSRCSEL
#   0x1A0 SCS           - bit 4 OSCRANGE, 5 OSCEN, 6 OSCSTAT
#   0x1A8 PCLKSEL0
#   0x1AC PCLKSEL1
#
# klipper-relevant synthesized behaviors:
#   - SCS (0x1A0): if OSCEN (bit 5) is written, OSCSTAT (bit 6) reads
#     back as set. SystemInit waits `while ((SCS & (1<<6)) == 0)`.
#   - PLL0STAT (0x088): if PLL0CON has bit 0 (PLLE0) set, return
#     bit 26 (PLOCK0) and bit 24 (PLLE0_STAT). If PLL0CON also has
#     bit 1 (PLLC0) set, also return bit 25 (PLLC0_STAT). klipper
#     waits `while (!(PLL0STAT & (1<<26)))` then
#     `while (!(PLL0STAT & ((1<<25) | (1<<24))))`.
#   - PLL1STAT (0x0A8): mirror PLL0STAT logic from PLL1CON
#     (only relevant in CONFIG_USB builds; the serial-mode test
#     config doesn't enable PLL1 but the stub is harmless either
#     way).
# All other registers are plain store/return so PCONP read-modify-
# write (enable_pclock) and CCLKCFG/PCLKSEL0/1 round-trip work.

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    if off == 0x1A0:  # SCS
        v = regs.get(off, 0)
        if v & (1 << 5):  # OSCEN -> OSCSTAT
            v |= (1 << 6)
        request.Value = v
    elif off == 0x088:  # PLL0STAT
        con = regs.get(0x080, 0)
        v = 0
        if con & 0x1:  # PLLE0
            v |= (1 << 26) | (1 << 24)  # PLOCK0 | PLLE0_STAT
        if con & 0x2:  # PLLC0
            v |= (1 << 25)  # PLLC0_STAT
        request.Value = v
    elif off == 0x0A8:  # PLL1STAT
        con = regs.get(0x0A0, 0)
        v = 0
        if con & 0x1:
            v |= (1 << 10) | (1 << 8)  # PLL1 PLOCK | PLLE_STAT
        if con & 0x2:
            v |= (1 << 9)  # PLLC_STAT
        request.Value = v
    else:
        request.Value = regs.get(off, 0)
