# Renode Python.PythonPeripheral script for the RP2040 XOSC register
# block at 0x40024000. Layout per regs/xosc.h:
#   - CTRL    (0x00) RW - FREQ_RANGE (bits 0..11) + ENABLE (bits 12..23)
#   - STATUS  (0x04) -    STABLE (bit 31), BADWRITE (bit 24),
#                          ENABLED (bit 12), FREQ_RANGE (bits 0..1).
#                          STABLE is RO; the rest are RW.
#   - DORMANT (0x08) RW - klipper does not touch.
#   - STARTUP (0x0C) RW - X1+DELAY config; klipper writes a startup
#                          time value but never reads back.
#   - COUNT   (0x1C) RW - downcounting cycle counter; not used by
#                          klipper.
#
# klipper's xosc_setup() in src/rp2040/main.c writes
#   XOSC_CTRL = (FREQ_RANGE_VALUE_1_15MHZ |
#                (XOSC_CTRL_ENABLE_VALUE_ENABLE << ENABLE_LSB))
# and then busy-waits on `XOSC_STATUS & XOSC_STATUS_STABLE_BITS`. We
# synthesize STATUS.STABLE (bit 31) when the CTRL.ENABLE field equals
# the magic 0xFAB enable value (the only enable encoding the SDK or
# datasheet ever specifies; 0xD1E disables, anything else is a
# BADWRITE).

if 'regs' not in dir():
    regs = {}

_ENABLE_VALUE_ENABLE = 0xFAB
_ENABLE_FIELD_LSB = 12
_ENABLE_FIELD_MASK = 0xFFF << _ENABLE_FIELD_LSB

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    if off == 0x04:  # STATUS
        ctrl = regs.get(0x00, 0)
        enable_field = (ctrl & _ENABLE_FIELD_MASK) >> _ENABLE_FIELD_LSB
        v = 0
        if enable_field == _ENABLE_VALUE_ENABLE:
            v |= 1 << 31  # STABLE
            v |= 1 << 12  # ENABLED (mirrors CTRL.ENABLE != DISABLE)
        request.Value = v
    else:
        request.Value = regs.get(off, 0)
