# Renode Python.PythonPeripheral script for the STM32H7 ADC. Replaces
# the upstream `adcM1S2: Analog.STM32F0_ADC @ 0x40022000` peripheral
# (declared in platforms/cpus/stm32h743.repl, omitted by the vendored
# test/emulator/repl/stm32h723.repl) with a stub klippy can drive via the
# renode_hooks.adc_default / adc_set magic-offset pokes - the same
# protocol as the other ADC stubs, but for the H7 ADC register layout
# (ISR/CR/SQR1/DR with ADSTART), which differs from both the F1/F4 ADC
# (SR/CR2/SQR3/DR) and the F0/G0 ADC (ISR/CR/CHSELR/DR).
#
# Why a stub: upstream models the H7 ADC1/ADC2 pair at 0x40022000 with
# the F0-family Analog.STM32F0_ADC, whose register semantics do not match
# klipper's H7 driver (src/stm32/stm32h7_adc.c). In particular the F0
# model self-clears calibration differently and addresses channels via
# CHSELR (0x28) rather than the H7's SQR1 (0x30), so klipper's
# `while (CR & ADCAL)` / SQR1-gated EOC handshake either spins or never
# returns a controllable conversion value. This stub models exactly the
# register slice the H7 firmware touches so conversions complete
# deterministically and return the poked value.
#
# H7 ADC driver handshake (src/stm32/stm32h7_adc.c):
#   gpio_adc_setup():  CR = ADVREGEN; (regulator delay);
#                      CR = cr | ADCAL; while (CR & ADCAL) ;
#                      ISR = ADRDY;
#                      while (!(CR & ADEN)) CR |= ADEN;
#                      while (!(ISR & ADRDY)) ;
#                      PCSEL |= 1 << chan;
#   gpio_adc_sample(): cr = CR; if (cr & ADSTART) wait;
#                      if (ISR & EOC) { if (SQR1 == chan<<6) ready; }
#                      SQR1 = chan << 6; CR = cr | ADSTART;
#   gpio_adc_read():   return DR;
#
# Register map (STM32H7 ADC, RM0468; offsets relative to ADC1 base
# 0x40022000):
#   ADC_ISR   0x00  status  - bit0 ADRDY, bit2 EOC
#   ADC_CR    0x08  control - bit0 ADEN, bit2 ADSTART, bit28 ADVREGEN,
#                             bit31 ADCAL (+ ADCALLIN/BOOST during cal)
#   ADC_PCSEL 0x1c  channel preselect - read/modify/write, storeback
#   ADC_SQR1  0x30  regular sequence  - channel in bits 6..10 (SQ1)
#   ADC_DR    0x40  data register     - last converted value (12 bit)
# This stub is registered with a 0x400-byte window (via the launcher's
# _EXTRA_PERIPHERAL_STUBS_FOR_CHIP, not the 0x200 AFEC block) so it also
# covers the ADC12 common block at base+0x300: omitting upstream's
# adcM1S2 unmaps the common registers, and klipper's setup does
# MODIFY_REG(adc_common->CCR=base+0x308, CKMODE, ...). That access is
# served as plain storeback (no busy-wait depends on it). ADC_CR's ADCAL
# (bit31) and ADSTART (bit2) read back 0 (instantaneous self-clear);
# ADEN (bit0) is reflected so the enable busy-wait exits; EOC is raised on
# ADSTART and cleared when DR is read.
#
# Conversion values are configured by renode_hooks adc_default / adc_set
# via writes to magic offsets (reserved relative to ADC1 - its real
# registers end below 0x100; the 0x100..0x1ff slice is ADC2's window,
# which the H7 firmware in scope never touches because every thermistor
# pin resolves to ADC1):
#   0x100        - default conversion value for every channel
#   0x104..0x184 - per-channel override (channel index 0..31, 4 bytes ea)
# Values are 12 bits (klipper's ADC_MAX=4095); DR readback is masked to
# 0xFFF to mirror that.

if 'init_done' not in dir():
    init_done = False
    sqr1 = 0
    adrdy = False
    eoc = False
    cr_persist = 0
    default_value = 0
    channel_values = {}
    regs = {}

ADC_ISR   = 0x00
ADC_CR    = 0x08
ADC_PCSEL = 0x1c
ADC_SQR1  = 0x30
ADC_DR    = 0x40

ADC_ISR_ADRDY = 0x0001        # bit0
ADC_ISR_EOC   = 0x0004        # bit2
ADC_CR_ADEN    = 0x00000001   # bit0
ADC_CR_ADSTART = 0x00000004   # bit2
ADC_CR_ADCAL   = 0x80000000   # bit31

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channel indexes 0..31

if request.IsInit:
    init_done = True
    sqr1 = 0
    adrdy = False
    eoc = False
    cr_persist = 0
    regs.clear()
    # default_value / channel_values preserved across reset: renode_hooks
    # pushes fixture values before the CPU starts running.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_CR:
        # Persist everything except the self-clearing ADCAL / ADSTART so
        # those read back 0 and ADEN (sticky) reads back set.
        cr_persist = val & ~(ADC_CR_ADCAL | ADC_CR_ADSTART)
        if val & ADC_CR_ADEN:
            adrdy = True
        if val & ADC_CR_ADSTART:
            # Software-triggered conversion completes immediately: the
            # next ISR poll sees EOC and DR holds the SQR1 channel value.
            eoc = True
    elif off == ADC_ISR:
        # Write-1-to-clear status bits (klipper clears ADRDY before the
        # enable wait).
        if val & ADC_ISR_ADRDY:
            adrdy = False
        if val & ADC_ISR_EOC:
            eoc = False
    elif off == ADC_SQR1:
        sqr1 = val & 0xFFFFFFFF
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 32 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        # PCSEL, CFGR, SMPRx, the ADC12 common CCR at 0x308, etc.
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_ISR:
        v = 0
        if adrdy:
            v |= ADC_ISR_ADRDY
        if eoc:
            v |= ADC_ISR_EOC
        request.Value = v
    elif off == ADC_CR:
        request.Value = cr_persist & ~(ADC_CR_ADCAL | ADC_CR_ADSTART)
    elif off == ADC_SQR1:
        request.Value = sqr1
    elif off == ADC_DR:
        ch = (sqr1 >> 6) & 0x1F          # SQ1 field
        request.Value = channel_values.get(ch, default_value) & 0xFFF
        eoc = False  # hardware clears EOC on DR read
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 32 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
