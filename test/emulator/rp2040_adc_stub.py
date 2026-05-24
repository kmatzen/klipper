# Renode Python.PythonPeripheral script for the RP2040 ADC. Renode's
# local rp2040 platform (test/emulator/repl/rp2040.repl) models no ADC,
# so the 0x4004C000 region is unmapped and klipper's thermistor reads
# return 0 / never complete. This stub drives klipper's RP2040 ADC
# handshake (src/rp2040/adc.c) and serves values poked through magic
# offsets by the renode_hooks adc_default / adc_set path (same protocol
# as afec_stub.py / sam4s_adc_stub.py / stm32_adc_stub.py).
#
# klipper's handshake (src/rp2040/adc.c):
#   gpio_adc_sample(): read ADC_CS; if !READY -> wait; else (software
#     tracks the in-flight channel) write ADC_CS = START_ONCE | EN |
#     (chan << AINSEL_LSB) to start a single conversion.
#   gpio_adc_read(): read ADC_RESULT.
# The conversion completes instantly in the model: ADC_CS always reads
# READY=1, and ADC_RESULT returns the value for the channel last
# selected in ADC_CS.AINSEL.
#
# Register map (RP2040 datasheet, offsets relative to base 0x4004C000):
#   ADC_CS     0x00  EN(0) TS_EN(1) START_ONCE(2) READY(8) AINSEL(12:14)
#   ADC_RESULT 0x04  12-bit conversion result
# Channels: GPIO26=0, GPIO27=1, GPIO28=2, GPIO29=3, temp sensor=4.
#
# Magic offsets (reserved on real silicon - the ADC's last register,
# ADC_INTS, is at 0x20):
#   0x100        - default conversion value for every channel
#   0x104..0x140 - per-channel override (channels 0..15, 4 bytes each)
# Values are 12 bits (klipper's ADC_MAX=4095); ADC_RESULT is masked to
# 0xFFF.

if 'init_done' not in dir():
    init_done = False
    ainsel = 0
    cs = 0
    default_value = 0
    channel_values = {}
    regs = {}

ADC_CS     = 0x00
ADC_RESULT = 0x04

ADC_CS_READY      = 0x100      # bit8
ADC_CS_START_ONCE = 0x04       # bit2 (auto-clears on real silicon)
ADC_CS_AINSEL_LSB = 12
ADC_CS_AINSEL_MSK = 0x7

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..15

if request.IsInit:
    init_done = True
    ainsel = 0
    cs = 0
    regs.clear()
    # default_value / channel_values intentionally preserved across
    # reset: renode_hooks pushes fixture values BEFORE the CPU starts
    # running, so we must not wipe them when the reset vector runs.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_CS:
        cs = val
        ainsel = (val >> ADC_CS_AINSEL_LSB) & ADC_CS_AINSEL_MSK
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 16 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_CS:
        # Conversion is always complete in the model: report READY and
        # clear the self-clearing START_ONCE bit.
        request.Value = (cs & ~ADC_CS_START_ONCE) | ADC_CS_READY
    elif off == ADC_RESULT:
        request.Value = channel_values.get(ainsel, default_value) & 0xFFF
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 16 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
