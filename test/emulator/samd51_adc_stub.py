# Renode Python.PythonPeripheral script for the Microchip SAMD51 /
# SAME54 ADC (ADC0 @ 0x43001C00, ADC1 @ 0x43002000). The samd51p20.repl
# maps no ADC, so this stub is the only thing at each base. It completes
# klipper's SAMX5 ADC handshake (src/atsamd/adc.c) with fixture-poked
# values - same magic-offset protocol as afec_stub.py / sam4s_adc_stub.py.
# One instance is registered per base by the launcher; each keeps its own
# state, so channel pokes that land on both serve the ADC the firmware
# actually reads (chan 0..15 -> ADC0, 16..31 -> ADC1, chan %= 16).
#
# klipper SAMX5 ADC handshake (src/atsamd/adc.c, register offsets from
# the SAME54 component header lib/same54/include/component/adc.h):
#   - adc_init busy-waits on SYNCBUSY (offset 0x30) after REFCTRL /
#     SAMPCTRL / CTRLA.ENABLE writes -> the stub must return SYNCBUSY=0.
#   - gpio_adc_sample(): INPUTCTRL.reg = MUXPOS(chan) | MUXNEG_GND
#     (offset 0x04, 16-bit; MUXPOS = low 5 bits), SYNC, then
#     SWTRIG.reg = START (offset 0x14, 8-bit; START = bit 1), SYNC, then
#     polls INTFLAG.RESRDY (offset 0x2E, 8-bit; RESRDY = bit 0).
#   - gpio_adc_read(): RESULT.reg (offset 0x40, 16-bit).
#   - gpio_adc_cancel_sample(): SWTRIG = FLUSH (bit 0), then writes
#     INTFLAG = RESRDY to clear.
#
# Synthesis model (mirrors the AFEC / SAM4S DRDY-arming idea):
#   - SWTRIG START arms RESRDY so the next INTFLAG poll observes
#     conversion-complete; RESULT read (and an INTFLAG write-1-to-clear)
#     disarms. The pre-START INTFLAG poll reads 0, so klippy proceeds to
#     SWTRIG instead of reading a stale ready bit.
#   - INPUTCTRL records the selected channel (MUXPOS); RESULT returns
#     that channel's fixture value, else the default.
#   - SYNCBUSY always reads 0 so the init busy-waits fall through.
#
# Conversion values are injected by renode_hooks adc_default / adc_set
# via writes to magic offsets reserved above the real register block
# (the ADC's last real register CALIB sits at 0x48):
#   0x100        - default conversion value for every channel
#   0x104+ch*4   - per-channel override (channels 0..31)
# Values are 12 bits (klipper ADC_MAX = 4095); RESULT readback masks 0xFFF.

if 'init_done' not in dir():
    init_done = False
    selected_channel = 0
    resrdy_armed = False
    default_value = 0
    channel_values = {}
    regs = {}

ADC_INPUTCTRL = 0x04
ADC_SWTRIG    = 0x14
ADC_INTFLAG   = 0x2E
ADC_SYNCBUSY  = 0x30
ADC_RESULT    = 0x40

ADC_SWTRIG_FLUSH = 0x1
ADC_SWTRIG_START = 0x2
ADC_INTFLAG_RESRDY = 0x1

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..31

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
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 32 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_SYNCBUSY:
        request.Value = 0
    elif off == ADC_INTFLAG:
        request.Value = ADC_INTFLAG_RESRDY if resrdy_armed else 0
    elif off == ADC_RESULT:
        request.Value = channel_values.get(selected_channel,
                                           default_value) & 0xFFF
        # Reading the result consumes the conversion; a fresh START is
        # needed for the next sample (matches the firmware's sequential
        # per-channel sampling).
        resrdy_armed = False
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 32 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
