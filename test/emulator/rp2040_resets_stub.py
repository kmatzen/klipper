# Renode Python.PythonPeripheral script for the RP2040 RESETS register
# block at 0x4000C000. Three registers per the SDK header:
#   - RESET       (0x00) RW - bitmask, 1 = peripheral held in reset.
#   - WDSEL       (0x04) RW - which peripherals get reset by the
#                              watchdog. Plain storeback; klipper
#                              writes but never reads back.
#   - RESET_DONE  (0x08) RO - bitmask, 1 = peripheral has come out of
#                              reset. On real silicon this lags the
#                              corresponding RESET bit clear by a few
#                              cycles; we synthesize the steady-state
#                              value RESET_DONE = ~RESET & 0x01ffffff
#                              (25 implemented peripheral bits per the
#                              datasheet 2.14.3).
#
# klipper's enable_pclock() in src/rp2040/main.c pulses RESETS.RESET
# (set then clear the bit for the peripheral) and busy-waits on
# RESET_DONE matching the cleared bit. Without this synthesized
# RESET_DONE behavior the firmware spins forever on the first
# enable_pclock call (PLL_SYS, used during clock_setup before XOSC is
# even probed).

# Implemented-peripheral bit mask. Bits beyond 24 are reserved on real
# silicon; we mask them off so RESET_DONE doesn't claim peripherals
# that don't exist.
_RESET_BITS_MASK = 0x01FFFFFF

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    if off == 0x08:  # RESET_DONE
        reset_val = regs.get(0x00, 0)
        request.Value = (~reset_val) & _RESET_BITS_MASK
    else:
        request.Value = regs.get(off, 0)
