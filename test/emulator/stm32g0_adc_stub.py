# Renode Python.PythonPeripheral script for the STM32G0 ADC. Replaces
# the upstream Analog.STM32G0_ADC peripheral (declared in
# platforms/cpus/stm32g0.repl, omitted by test/emulator/repl/stm32g0b1.repl)
# with a stub klippy can drive via the renode_hooks.adc_default / adc_set
# magic-offset pokes - the same protocol as afec_stub.py /
# sam4s_adc_stub.py / stm32_adc_stub.py, but for the G0/F0-family ADC
# register layout (which is completely different from the F1/F4 ADC).
#
# Why a stub: klipper's G0 ADC driver (src/stm32/stm32f0_adc.c, shared
# with F0) drives this handshake -
#   gpio_adc_setup():  CR = ADVREGEN; (regulator delay);
#                      CR = ADVREGEN | ADCAL; while (CR & ADCAL) ;
#                      ISR = ADRDY; CR = ADVREGEN | ADEN;
#                      while (!(ISR & ADRDY)) ;
#   gpio_adc_sample(): if (CR & ADSTART) wait;
#                      if (ISR & EOC) { if (CHSELR == chan) ready; }
#                      if (CHSELR != chan) { ISR = CCRDY; CHSELR = chan;
#                                            while (!(ISR & CCRDY)) ; }
#                      CR = ADVREGEN | ADSTART;
#   gpio_adc_read():   return DR;
# Upstream Renode's Analog.STM32G0_ADC neither self-clears ADCAL the way
# this loop needs nor returns klippy-controllable conversion values, so
# without this stub the boot either spins in the calibration busy-wait or
# every heater reads a fixed reference-derived value that may be outside
# the thermistor range. This stub models exactly the register slice the
# firmware touches and returns the poked value so conversions complete
# deterministically.
#
# Register map (STM32G0 ADC, RM0444; offsets relative to ADC1 base
# 0x40012400):
#   ADC_ISR    0x00  status   - bit0 ADRDY, bit2 EOC, bit13 CCRDY
#   ADC_CR     0x08  control  - bit0 ADEN, bit2 ADSTART, bit31 ADCAL
#   ADC_CHSELR 0x28  channel selection - a one-hot MASK (1 << chan), not
#                    a channel index: klipper sets g.chan = 1 << chan and
#                    writes it here, so DR returns the value for the bit
#                    position set in CHSELR.
#   ADC_DR     0x40  data register - last converted value (12 bit)
# The ADCAL calibration busy-wait is satisfied by reading CR's bit31 back
# as 0; ADSTART (bit2) likewise reads back 0 so the model's instantaneous
# conversion is seen as "not in progress". CCRDY (channel-config-ready)
# is raised on every CHSELR write and cleared by the write-1-to-clear ISR
# access that precedes it. EOC is raised on ADSTART and cleared when DR is
# read (hardware clears EOC on DR read).
#
# Conversion values are configured by renode_hooks adc_default / adc_set
# via writes to magic offsets (reserved on real silicon - the G0 ADC's
# real registers end well below 0x100, and the ADC common block is at a
# separate 0x40012708 base outside this 0x200 window):
#   0x100        - default conversion value for every channel
#   0x104..0x184 - per-channel override (channel index 0..31, 4 bytes each)
# Values are 12 bits (klipper's ADC_MAX=4095); DR readback is masked to
# 0xFFF to mirror that.

if 'init_done' not in dir():
    init_done = False
    chselr = 0
    adrdy = False
    eoc = False
    ccrdy = False
    cr_persist = 0
    default_value = 0
    channel_values = {}
    regs = {}

ADC_ISR    = 0x00
ADC_CR     = 0x08
ADC_CHSELR = 0x28
ADC_DR     = 0x40

ADC_ISR_ADRDY = 0x0001        # bit0
ADC_ISR_EOC   = 0x0004        # bit2
ADC_ISR_CCRDY = 0x2000        # bit13
ADC_CR_ADEN    = 0x00000001   # bit0
ADC_CR_ADSTART = 0x00000004   # bit2
ADC_CR_ADCAL   = 0x80000000   # bit31

MAGIC_DEFAULT = 0x100
# 0x104 + ch * 4. The G0 ADC exposes up to 19 channels (PC5 = index 18 in
# src/stm32/stm32f0_adc.c adc_pins[]), so the per-channel poke range must
# reach past channel 15 (e.g. the SKR mini E3 v3.0 bed thermistor PC4 is
# channel 17). 32 channels (0x104..0x184) stays inside the 0x200 stub
# window and well clear of the real G0 ADC registers (all below 0xB8).
MAGIC_CH_BASE = 0x104


def _chselr_to_index(mask):
    # CHSELR holds a one-hot 1<<chan; recover chan (lowest set bit).
    if not mask:
        return 0
    idx = 0
    m = mask & 0xFFFFFFFF
    while not (m & 1):
        m >>= 1
        idx += 1
    return idx


if request.IsInit:
    init_done = True
    chselr = 0
    adrdy = False
    eoc = False
    ccrdy = False
    cr_persist = 0
    regs.clear()
    # default_value / channel_values preserved across reset: renode_hooks
    # pushes fixture values before the CPU starts running.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == ADC_CR:
        # Persist everything except the self-clearing ADCAL / ADSTART.
        cr_persist = val & ~(ADC_CR_ADCAL | ADC_CR_ADSTART)
        if val & ADC_CR_ADEN:
            adrdy = True
        if val & ADC_CR_ADSTART:
            # Software-triggered conversion completes immediately in the
            # model: the next ISR poll sees EOC and DR holds the value for
            # the channel selected in CHSELR.
            eoc = True
    elif off == ADC_ISR:
        # Write-1-to-clear status bits (klipper clears ADRDY / CCRDY).
        if val & ADC_ISR_ADRDY:
            adrdy = False
        if val & ADC_ISR_CCRDY:
            ccrdy = False
        if val & ADC_ISR_EOC:
            eoc = False
    elif off == ADC_CHSELR:
        chselr = val & 0xFFFFFFFF
        ccrdy = True
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 32 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == ADC_ISR:
        v = 0
        if adrdy:
            v |= ADC_ISR_ADRDY
        if eoc:
            v |= ADC_ISR_EOC
        if ccrdy:
            v |= ADC_ISR_CCRDY
        request.Value = v
    elif off == ADC_CR:
        # ADCAL / ADSTART always read back 0 (instantaneous self-clear).
        request.Value = cr_persist & ~(ADC_CR_ADCAL | ADC_CR_ADSTART)
    elif off == ADC_CHSELR:
        request.Value = chselr
    elif off == ADC_DR:
        ch = _chselr_to_index(chselr)
        request.Value = channel_values.get(ch, default_value) & 0xFFF
        eoc = False  # hardware clears EOC on DR read
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 32 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
