# Renode Python.PythonPeripheral script for the Microchip SAMD21 ADC
# (single ADC @ 0x42004000). The samd21g18.repl maps no ADC, so this
# stub is the only thing at that base. It completes klipper's SAMD21 ADC
# handshake (src/atsamd/adc.c, CONFIG_MACH_SAMD21 branch) with
# fixture-poked values - same magic-offset protocol as afec_stub.py /
# samd51_adc_stub.py.
#
# The SAMD21 ADC register map differs from the SAMX5 (SAMD51) one, so it
# needs its own stub (register offsets from the SAMD21 component header
# lib/samd21/samd21a/include/component/adc.h):
#   SWTRIG    @ 0x0C (8-bit)   START = bit 1, FLUSH = bit 0
#   INPUTCTRL @ 0x10 (32-bit)  MUXPOS = low 5 bits (the channel index)
#   INTFLAG   @ 0x18 (8-bit)   RESRDY = bit 0
#   RESULT    @ 0x1A (16-bit)
# SAMD21 has no SYNCBUSY gating (the SAMD51_ADC_SYNC macro is a no-op for
# CONFIG_MACH_SAMD21), so unlike the SAMD51 stub there is nothing to
# synthesize there.
#
# klipper handshake (src/atsamd/adc.c gpio_adc_sample / gpio_adc_read):
#   - INPUTCTRL.reg = MUXPOS(chan) | MUXNEG_GND | GAIN_DIV2
#   - SWTRIG.reg = START
#   - poll INTFLAG.RESRDY, then read RESULT.
#   - gpio_adc_cancel_sample: SWTRIG = FLUSH, then INTFLAG = RESRDY.
#
# Synthesis model (mirrors the AFEC / SAMD51 DRDY-arming idea):
#   - SWTRIG START arms RESRDY for the next INTFLAG poll; RESULT read
#     (and an INTFLAG write-1-to-clear) disarms. Pre-START INTFLAG reads
#     0 so klippy proceeds to SWTRIG.
#   - INPUTCTRL records the selected channel; RESULT returns that
#     channel's fixture value, else the default.
#
# Conversion values are injected by renode_hooks adc_default / adc_set
# via writes to magic offsets reserved above the real register block
# (the ADC's last real register DBGCTRL sits at 0x2A):
#   0x100        - default conversion value for every channel
#   0x104+ch*4   - per-channel override (channels 0..19)
# Values are 12 bits (klipper ADC_MAX = 4095); RESULT readback masks 0xFFF.

if 'init_done' not in dir():
    init_done = False
    selected_channel = 0
    resrdy_armed = False
    default_value = 0
    channel_values = {}
    regs = {}

ADC_SWTRIG    = 0x0C
ADC_INPUTCTRL = 0x10
ADC_INTFLAG   = 0x18
ADC_RESULT    = 0x1A

ADC_SWTRIG_FLUSH = 0x1
ADC_SWTRIG_START = 0x2
ADC_INTFLAG_RESRDY = 0x1

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..19

if request.IsInit:
    init_done = True
    selected_channel = 0
    resrdy_armed = False
    regs.clear()
    # default_value / channel_values intentionally preserved across
    # reset: renode_hooks pushes fixture values BEFORE Renode's `start`,
    # so we must not wipe them when the reset vector runs.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_INPUTCTRL:
        selected_channel = val & 0x1F  # MUXPOS field
        regs[off] = val
    elif off == ADC_SWTRIG:
        if val & ADC_SWTRIG_START:
            resrdy_armed = True
        if val & ADC_SWTRIG_FLUSH:
            resrdy_armed = False
        regs[off] = val
    elif off == ADC_INTFLAG:
        # RESRDY is write-1-to-clear (gpio_adc_cancel_sample).
        if val & ADC_INTFLAG_RESRDY:
            resrdy_armed = False
        regs[off] = val
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 20 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_INTFLAG:
        request.Value = ADC_INTFLAG_RESRDY if resrdy_armed else 0
    elif off == ADC_RESULT:
        request.Value = channel_values.get(selected_channel,
                                           default_value) & 0xFFF
        # Reading the result consumes the conversion; a fresh START is
        # needed for the next sample.
        resrdy_armed = False
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 20 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
