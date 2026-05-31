# Renode Python.PythonPeripheral script for the RP2040 CLOCKS register
# block at 0x40008000. Plain dict-backed storeback for every register
# except the CLK_<x>_SELECTED status registers, which the firmware busy-
# waits on during clock_setup() in src/rp2040/main.c. SELECTED encodes
# the active source as a one-hot mask of (1 << CTRL.SRC) (RP2040
# datasheet 2.15.7); on real silicon this updates a few cycles after a
# CTRL write, so we synthesize the same value at read time from whatever
# CTRL.SRC was last written.
#
# klipper firmware busy-waits on three SELECTED registers during boot:
#   - clk_sys.selected (offset 0x44): waits for 0x1 with CTRL=0
#     (initial state, internal source) and then for (1<<1) after CTRL is
#     re-written with SRC_VALUE_AUX (1).
#   - clk_ref.selected (offset 0x38): waits for 0x1 with CTRL=0 and
#     then for (1<<2) after CTRL is re-written with XOSC_CLKSRC (2).
# CTRL field widths come from the regs/clocks.h SDK header:
#   - CLK_REF_CTRL.SRC: bits 0..1 (2 bits, 4 sources)
#   - CLK_SYS_CTRL.SRC: bit 0 (1 bit, 2 sources)
# All other CTRL/DIV/AUXSRC/etc. writes are non-gating - they configure
# downstream peripheral clocks (clk_peri / clk_adc / clk_usb /
# clk_rtc / clk_gpout*) but klipper firmware never reads them back to
# gate progress, so plain storeback is sufficient.

if 'regs' not in dir():
    regs = {}

# (selected_offset, ctrl_offset, src_field_mask) for the SELECTED
# registers the firmware busy-waits on. Keep narrow - other clocks have
# SELECTED registers too but the firmware never polls them, so handing
# them a synthesized response would be harmless dead code.
_SELECTED_FOR_CTRL = {
    0x38: (0x30, 0x3),  # CLK_REF: SELECTED@0x38, CTRL@0x30, 2-bit SRC
    0x44: (0x3C, 0x1),  # CLK_SYS: SELECTED@0x44, CTRL@0x3C, 1-bit SRC
}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    spec = _SELECTED_FOR_CTRL.get(off)
    if spec is not None:
        ctrl_off, src_mask = spec
        src = regs.get(ctrl_off, 0) & src_mask
        request.Value = 1 << src
    else:
        request.Value = regs.get(off, 0)
