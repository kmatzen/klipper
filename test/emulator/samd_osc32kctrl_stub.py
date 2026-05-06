# Renode Python.PythonPeripheral script for the SAMD51 OSC32KCTRL
# 32kHz oscillator block. Renode does not model OSC32KCTRL; without
# this stub klipper's clock_init_32k() spins on STATUS.XOSC32KRDY
# after enabling XOSC32K.
#
# Layout (component/osc32kctrl.h):
#   0x00 INTENCLR  (4 byte)
#   0x04 INTENSET  (4 byte)
#   0x08 INTFLAG   (4 byte)
#   0x0C STATUS    (4 byte)  - XOSC32KRDY bit 0
#   0x10 RTCCTRL   (1 byte)
#   0x14 XOSC32K   (2 byte)  - ENABLE bit 1
#   0x16 CFDCTRL   (1 byte)
#   0x17 EVCTRL    (1 byte)
#
# klipper-relevant behavior: STATUS.XOSC32KRDY (bit 0) is set
# whenever XOSC32K.ENABLE (bit 1) is set. All other registers are
# plain store/return; klipper writes them but never gates progress
# on their readback values.

if 'regs' not in dir():
    regs = {}

if request.IsInit:
    regs.clear()
elif request.IsWrite:
    regs[request.Offset] = int(request.Value)
elif request.IsRead:
    off = request.Offset
    if off == 0x0C:  # STATUS
        if regs.get(0x14, 0) & (1 << 1):  # XOSC32K.ENABLE
            request.Value = (1 << 0)  # XOSC32KRDY
        else:
            request.Value = 0
    else:
        request.Value = regs.get(off, 0)
