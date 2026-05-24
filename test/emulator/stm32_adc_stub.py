# Renode Python.PythonPeripheral script for the STM32F1 / STM32F4 ADC.
# Replaces the upstream Analog.STM32_ADC peripheral with a stub klippy
# can drive via the renode_hooks.adc_default / adc_set magic-offset pokes
# (same protocol as test/emulator/afec_stub.py and sam4s_adc_stub.py).
#
# F1 and F4 share src/stm32/adc.c and the same SR/CR2/SQR3/DR register
# slice, so a single stub covers both families. The only register-level
# difference the stub must absorb is the ADC_CR2 software-trigger bit:
# the F1 reference manual places SWSTART at bit 22, the F4 (RM0090) at
# bit 30. We accept either (see ADC_CR2_SWSTART below). F4 skips the F1
# RSTCAL/CAL self-calibration entirely, but masking those bits is inert
# there, so no family conditional is needed.
#
# Why a stub: klipper's STM32 F1/F4 ADC driver (src/stm32/adc.c) drives a
# specific software-trigger handshake -
#   gpio_adc_sample(): read ADC_SR; if STRT clear -> write ADC_SQR3 =
#     channel, write ADC_CR2 = SWSTART|... to start a conversion; on the
#     next call, the conversion is "ready" only when STRT && EOC are set
#     AND ADC_SQR3 still equals the requested channel.
#   gpio_adc_read(): write ADC_SR = ~STRT (clear STRT), then read ADC_DR.
# Upstream Renode's Analog.STM32_ADC does not complete this SWSTART ->
# STRT/EOC -> SQR3/DR cycle the way the F1 firmware polls it, so EOC is
# never observed, no conversion ever "completes", klippy receives no
# analog_in samples, and every heater reads temp=0.0 (with no range
# check, since a range check only runs on an actual sample). This stub
# models exactly the register slice the firmware touches so conversions
# complete deterministically and return the poked value.
#
# Register map (STM32F1 ADC, RM0008; offsets relative to the peripheral
# base 0x40012400 = ADC1):
#   ADC_SR   0x00  status  - bit1 EOC, bit4 STRT
#   ADC_CR2  0x08  control - bit0 ADON, bit2 CAL, bit3 RSTCAL, bit22 SWSTART
#   ADC_SQR3 0x34  regular sequence - channel of the 1st conversion
#   ADC_DR   0x4C  data register - last converted value (12 bit)
# The calibration busy-waits (`while (CR2 & RSTCAL)` / `& CAL`) are
# satisfied by always reading those two bits back as 0.
#
# Conversion values are configured externally by renode_hooks's
# adc_default / adc_set via writes to magic offsets (reserved on real
# silicon - the ADC's last real register, ADC_DR, ends at 0x4C):
#   0x100        - default conversion value for every channel
#   0x104..0x140 - per-channel override (channels 0..15, 4 bytes each)
# Values are 12 bits (klipper's ADC_MAX=4095); ADC_DR readback is masked
# to 0xFFF to mirror that.

if 'init_done' not in dir():
    init_done = False
    sqr3 = 0
    strt = False
    eoc = False
    default_value = 0
    channel_values = {}
    regs = {}

ADC_SR   = 0x00
ADC_CR2  = 0x08
ADC_SQR3 = 0x34
ADC_DR   = 0x4C

ADC_SR_EOC      = 0x02         # bit1
ADC_SR_STRT     = 0x10         # bit4
ADC_CR2_CAL     = 0x04         # bit2
ADC_CR2_RSTCAL  = 0x08         # bit3
# SWSTART is bit22 on STM32F1 (RM0008) and bit30 on STM32F4 (RM0090).
# Accept either so the one stub serves both families; no other CR2 write
# klipper issues (ADON | EXTSEL | EXTTRIG | TSVREFE on F1, ADON on F4)
# sets bit22 or bit30, so this never false-triggers a conversion.
ADC_CR2_SWSTART = 0x00400000 | 0x40000000   # F1 bit22 | F4 bit30

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..15

if request.IsInit:
    init_done = True
    sqr3 = 0
    strt = False
    eoc = False
    regs.clear()
    # default_value / channel_values intentionally preserved across
    # reset: renode_hooks pushes fixture values BEFORE the CPU starts
    # running, so we must not wipe them when the reset vector runs.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_SR:
        # gpio_adc_read() writes ADC_SR = ~STRT to clear STRT; treat any
        # SR write as "end of conversion" and reset for the next one.
        strt = False
        eoc = False
    elif off == ADC_CR2:
        regs[off] = val
        if val & ADC_CR2_SWSTART:
            # Software-triggered conversion: it completes immediately in
            # the model, so the firmware's next ADC_SR poll sees STRT &
            # EOC and reads the value for the channel set in ADC_SQR3.
            strt = True
            eoc = True
    elif off == ADC_SQR3:
        sqr3 = val & 0x1F
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 16 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_SR:
        v = 0
        if strt:
            v |= ADC_SR_STRT
        if eoc:
            v |= ADC_SR_EOC
        request.Value = v
    elif off == ADC_CR2:
        # Mask the self-clearing calibration bits so the firmware's
        # `while (CR2 & RSTCAL)` / `while (CR2 & CAL)` loops exit.
        request.Value = regs.get(off, 0) & ~(ADC_CR2_RSTCAL | ADC_CR2_CAL)
    elif off == ADC_SQR3:
        request.Value = sqr3
    elif off == ADC_DR:
        request.Value = channel_values.get(sqr3, default_value) & 0xFFF
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 16 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
