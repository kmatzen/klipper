# Renode Python.PythonPeripheral script for SAM4S/SAM3X ADC. Replaces
# the upstream Analog.SAM4S_ADC peripheral with a stub klippy can
# drive via the renode_hooks.adc_default / adc_set magic-offset pokes
# (same protocol as test/emulator/afec_stub.py for SAM4E/SAME70).
# Without this stub, all ADC channels read 0 and adc_scaled's
# (vref - vssa) divisor is 0, raising ZeroDivisionError in klippy
# (extras/adc_scaled.py).
#
# Implements the slice of SAM4S ADC behaviour klipper actually touches
# (src/atsam/adc.c):
#   - ADC_CHER (offset 0x10) write enables the requested channel:
#     CHSR |= written bits.
#   - ADC_CHDR (offset 0x14) write disables: CHSR &= ~written bits.
#   - ADC_CHSR (offset 0x18) reads as currently-enabled mask.
#   - ADC_CR (offset 0x00) START bit arms ISR.DRDY for the very next
#     ISR read (firmware polls ISR until DRDY=1 then reads LCDR).
#   - ADC_ISR (offset 0x30) reads DRDY=1 if armed; bits 0..15 mirror
#     per-channel EOC. Clears DRDY after read (real silicon clears
#     on ISR read; firmware tolerates retain-after-read too).
#   - ADC_LCDR (offset 0x20) returns last converted value: per-channel
#     override if set, else default_value.
#
# Conversion values are configured externally by renode_hooks's
# adc_default / adc_set via writes to magic offsets in the range
# 0x100..0x140 (reserved on real silicon - ADC's last register is
# ADC_WPSR at 0xE8):
#   0x100 - default conversion value for every channel
#   0x104..0x140 - per-channel override (channels 0..15, 4 bytes each)
# Values are 12 bits (klipper's ADC_MAX=4095). The ADC_LCDR readback
# is masked to 0xFFF to mirror that.

if 'init_done' not in dir():
    init_done = False
    chsr = 0
    last_started_channel = 0
    drdy_armed = False
    default_value = 0
    channel_values = {}
    regs = {}

ADC_CR    = 0x00
ADC_CHER  = 0x10
ADC_CHDR  = 0x14
ADC_CHSR  = 0x18
ADC_LCDR  = 0x20
ADC_ISR   = 0x30

ADC_CR_SWRST = 0x1
ADC_CR_START = 0x2

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..15

if request.IsInit:
    init_done = True
    chsr = 0
    drdy_armed = False
    regs.clear()
    # default_value / channel_values intentionally preserved across
    # reset: renode_hooks pushes fixture values BEFORE the CPU starts
    # running, so we must not wipe them when the reset vector runs.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_CR:
        if val & ADC_CR_SWRST:
            drdy_armed = False
        if val & ADC_CR_START:
            drdy_armed = True
            # Remember the most-recently-enabled channel so LCDR returns
            # its value. Klipper enables exactly one channel at a time
            # via ADC_CHER, so chsr should be a single-bit mask; the
            # bit_length-1 finds it.
            if chsr:
                last_started_channel = chsr.bit_length() - 1
        regs[off] = val
    elif off == ADC_CHER:
        chsr |= val & 0xFFFF
    elif off == ADC_CHDR:
        chsr &= ~(val & 0xFFFF)
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 16 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_LCDR:
        v = channel_values.get(last_started_channel, default_value)
        request.Value = v & 0xFFF
    elif off == ADC_ISR:
        if drdy_armed:
            # DRDY (bit 24) | per-channel EOC mirroring active CHSR
            request.Value = (1 << 24) | chsr
        else:
            request.Value = 0
    elif off == ADC_CHSR:
        request.Value = chsr & 0xFFFF
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 16 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
