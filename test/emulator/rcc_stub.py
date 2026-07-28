# Renode Python.PythonPeripheral script for the STM32 RCC clock
# controller. Renode's STM32 platform .repls (F1/F4/F7/H7/G0) do not
# model RCC; without a stub the firmware spins forever in clock
# setup waiting for HSERDY / PLLRDY bits that the SVD-derived
# read-as-zero stubs never raise.
#
# Implements only what's needed for klipper's clock_setup() to
# complete:
#   - CR (offset 0x00): when HSEON / HSION / PLLON written, the
#     corresponding READY bit reads back as set on the next read.
#   - CFGR (offset 0x04): SWS field (system clock switch status, bits
#     2-3) reflects SW field (bits 0-1) - whichever clock was
#     selected, the firmware sees confirmation that the switch
#     completed.
#   - BDCR (offset 0x20): LSEON -> LSERDY (klipper firmware doesn't
#     hang on LSE today but harmless to model).
#   - CSR  (offset 0x24): LSI always ready.
#
# All other RCC registers (CIR, AHB/APB enables, peripheral resets)
# are passive store-and-return - klipper writes peripheral-clock
# enables but never reads them back to gate progress.
#
# Renode injects `request` (PythonRequest), `self` (the
# PythonPeripheral instance), `size` and the Logger imports into
# this script's scope. State is hung off `self` so it persists
# across reads/writes for the lifetime of the Renode process.

# C# property names stay PascalCase in IronPython - the auto-PEP8
# lowercase form some Renode docs show (`request.isInit`) is NOT
# available on PythonRequest. Use IsInit / IsRead / IsWrite / Value /
# Offset directly.
#
# Per-peripheral state lives in script-global variables (NOT on
# `self` - PythonPeripheral is a C# object without __dict__, so
# attribute assignment doesn't stick). The PeripheralPythonEngine
# preserves its compiled-code scope across Execute calls, so a
# module-level dict declared once persists for the lifetime of this
# peripheral instance.
if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    last = regs.get(request.Offset, 0)
    if request.Offset == 0x00:  # CR
        v = last
        if v & (1 << 16):  # HSEON -> HSERDY
            v |= (1 << 17)
        if v & (1 << 24):  # PLLON -> PLLRDY
            v |= (1 << 25)
        v |= (1 << 1)      # HSI always ready (set after reset)
        request.Value = v
    elif request.Offset == 0x04:  # CFGR
        sw = last & 0x3
        v = (last & ~0xC) | (sw << 2)
        request.Value = v
    elif request.Offset == 0x20:  # BDCR
        v = last
        if v & (1 << 0):   # LSEON -> LSERDY
            v |= (1 << 1)
        request.Value = v
    elif request.Offset == 0x24:  # CSR
        v = last | (1 << 1)  # LSI always ready
        request.Value = v
    else:
        request.Value = last
