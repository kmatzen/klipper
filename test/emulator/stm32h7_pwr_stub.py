# Renode Python.PythonPeripheral script for the STM32H7 PWR (power
# control) block at 0x58024800. Renode's upstream stm32h743.repl
# models RCC (Miscellaneous.STM32H7_RCC) and the flash controller but
# leaves PWR as a bare `Tag <0x58024800, 0x58024BFF> "PWR"` - reads
# return 0. klipper's src/stm32/stm32h7.c clock_setup() busy-waits on
# two PWR ready bits before it ever touches RCC, so without this stub
# the firmware spins at the very first loop and never reaches identify:
#   - PWR->CR3 = LDOEN;  while (!(PWR->CSR1 & ACTVOSRDY)) ;
#   - PWR->D3CR = VOS;   while (!(PWR->D3CR & VOSRDY)) ;
#
# Implements only what clock_setup() needs:
#   - CSR1 (offset 0x04): ACTVOSRDY (bit 13) always reads set - the
#     active voltage scaling is "ready" the moment the LDO is enabled.
#   - D3CR (offset 0x18): VOSRDY (bit 13) reads set, OR'd over the VOS
#     field the firmware just wrote (so a later read still sees its
#     selected VOS level). Both the H723 (VOS=3 or 0) and H743
#     (VOS=3, optionally + overdrive) paths poll the same bit.
#
# Every other PWR register (CR1/CR2/CR3/CPUCR/WKUP*) is plain
# store-and-return: the firmware writes them during init but never
# reads them back to gate boot progress.
#
# PythonPeripheral protocol notes: `request` (PythonRequest) and
# `self` are injected into this script's scope; C# property names stay
# PascalCase in IronPython (IsInit / IsRead / IsWrite / Value /
# Offset). Per-peripheral state lives in a module-level dict (the
# PeripheralPythonEngine preserves the compiled scope across Execute
# calls; PythonPeripheral is a C# object without __dict__, so
# attributes assigned to `self` would not stick).

_ACTVOSRDY = 1 << 13   # PWR_CSR1_ACTVOSRDY
_VOSRDY = 1 << 13      # PWR_D3CR_VOSRDY

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    last = regs.get(request.Offset, 0)
    if request.Offset == 0x04:    # CSR1
        request.Value = last | _ACTVOSRDY
    elif request.Offset == 0x18:  # D3CR
        request.Value = last | _VOSRDY
    else:
        request.Value = last
