# Renode Python.PythonPeripheral script for the HDSC HC32F460 ADC
# (M4_ADC1 @ 0x40040000). Renode has no HDSC peripheral models, and the
# hc32f460.repl maps no ADC at all, so without this stub klipper's
# thermistor reads (src/hc32f460/adc.c) return 0 and every heater sits
# at temp=0.0. This stub serves the slice of the ADC the firmware
# actually drives so fixture-poked conversion values reach klippy -
# same magic-offset protocol as afec_stub.py / sam4s_adc_stub.py.
#
# klipper HC32F460 ADC handshake (src/hc32f460/adc.c + the HDSC lib
# functions in lib/hc32f460/driver/src/hc32f460_adc.c, all DIRECT
# register access - no Cortex-M bit-band aliasing):
#   - gpio_adc_sample():
#       if ADC_GetEocFlag(SEQ_A): `ADCx->ISR & 0x1`  -> conversion done
#           ADC_ClrEocFlag(SEQ_A): `ADCx->ISR &= ~0x1`
#           return 0 (ready)
#       elif M4_ADC1->STR & 1:  -> conversion running, wait
#       else: ADC_StartConvert(): `ADCx->STR = 1`  -> kick a conversion
#   - gpio_adc_read(): ADC_GetValue(chan) = (&ADCx->DR0)[chan], i.e.
#       the 16-bit DRn register at 0x50 + chan*2.
#
# Register offsets walked from the M4_ADC_TypeDef struct
# (lib/hc32f460/mcu/common/hc32f460.h):
#   STR  @ 0x00 (8-bit)   STRT = bit 0
#   ISR  @ 0x46 (8-bit)   EOCAF (SEQ A end-of-conversion) = bit 0
#   DR0  @ 0x50 (16-bit)  DRn at 0x50 + n*2, n = 0..16 (the channel
#                         index from src/hc32f460/adc.c adc_gpio[])
#
# Synthesis model (mirrors the AFEC / SAM4S stubs' DRDY-arming idea):
#   - A write to STR with bit 0 set (ADC_StartConvert) arms EOCAF so the
#     very next ISR read observes conversion-complete. STR reads back 0
#     (real silicon auto-clears STRT in single-shot SAOnce mode), so the
#     "still running" branch is never taken - and the EOCAF check runs
#     first anyway.
#   - ISR read returns EOCAF=1 while armed, else 0 (so the pre-start
#     poll falls through to ADC_StartConvert instead of reading a stale
#     ready bit). A write to ISR clearing bit 0 (ADC_ClrEocFlag's
#     read-modify-write) disarms.
#   - DRn read returns the per-channel fixture value (channel index =
#     (offset - 0x50) / 2), else the default.
#
# Conversion values are injected by renode_hooks adc_default / adc_set
# via writes to magic offsets reserved above the real register block
# (the ADC's last real register PGAINSR1 sits near 0xC2):
#   0x100        - default conversion value for every channel
#   0x104+ch*4   - per-channel override (channels 0..16)
# Values are 12 bits (klipper ADC_MAX = 4095); DR readback masks 0xFFF.

if 'init_done' not in dir():
    init_done = False
    eoc_armed = False
    default_value = 0
    channel_values = {}
    regs = {}

ADC_STR = 0x00
ADC_ISR = 0x46
ADC_DR0 = 0x50
ADC_DR_LAST = 0x50 + 16 * 2  # DR16

ADC_STR_STRT = 0x1
ADC_ISR_EOCAF = 0x1

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..16

if request.IsInit:
    init_done = True
    eoc_armed = False
    regs.clear()
    # default_value / channel_values intentionally preserved across
    # reset: renode_hooks pushes fixture values BEFORE Renode's `start`,
    # so we must not wipe them when the reset vector runs.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_STR:
        if val & ADC_STR_STRT:
            # ADC_StartConvert: arm EOCAF for the next ISR poll.
            eoc_armed = True
        regs[off] = val
    elif off == ADC_ISR:
        # ADC_ClrEocFlag writes ISR with bit 0 cleared.
        if not (val & ADC_ISR_EOCAF):
            eoc_armed = False
        regs[off] = val
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 17 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_STR:
        # Single-shot conversions auto-clear STRT; the EOCAF check runs
        # before the "running" check anyway, so report not-running.
        request.Value = 0
    elif off == ADC_ISR:
        request.Value = ADC_ISR_EOCAF if eoc_armed else 0
    elif ADC_DR0 <= off < ADC_DR_LAST + 2 and (off - ADC_DR0) % 2 == 0:
        ch = (off - ADC_DR0) // 2
        request.Value = channel_values.get(ch, default_value) & 0xFFF
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 17 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
