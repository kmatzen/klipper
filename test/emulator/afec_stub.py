# Renode Python.PythonPeripheral script for ATSAME70 AFEC (Analog
# Front-End Converter). Renode does not ship a SAM_AFEC peripheral
# model, so without this stub klipper firmware on SAME70 reads
# AFE_LCDR / AFE_CDR as zero, decodes as below-min-temp on every
# thermistor in the printer config, and shuts down before the test
# can drive any gcode. printers.test runs M105 against the empty
# fixture which makes the issue immediate.
#
# Implements the slice of AFEC behaviour klipper actually uses
# (src/atsam/sam4e_afec.c, MACH_SAM4E and MACH_SAME70 share this
# file - both chips' boot init runs gpio_afec_init):
#
#   - AFE_ISR (offset 0x30) reads as 0 until the first AFE_CR_START
#     write, then as DRDY|<all-channel-EOC>. The pre-START zero is
#     what makes init_afec()'s `if (ISR & DRDY) return -1` busy-check
#     fall through to the AFEC configuration block instead of returning
#     -1 forever (and thereby spinning gpio_afec_init's
#     `while(init_afec(AFEC0) != 0)` loop indefinitely - that loop is
#     not bounded). After CR_START the synthesised DRDY|EOC satisfies
#     gpio_adc_sample's "(DRDY) && (1<<chan)" check for whichever
#     channel just got CR_START written. AFE_CR_SWRST reverts the
#     synthesis to the pre-START state, matching real silicon's reset
#     behaviour.
#   - AFE_LCDR (offset 0x20) reads as the last converted value for
#     the channel previously selected via AFE_CSELR (offset 0x64).
#   - AFE_CDR (offset 0x68) reads as the per-channel data for the
#     channel previously selected via AFE_CSELR.
#   - AFE_CHSR (offset 0x1C) reads with all 12 channels enabled
#     (klipper's gpio_adc_sample treats CHSR as enabled-mask).
#
# Conversion values are configured externally by the renode_hooks
# adc_default / adc_set path via writes to magic offsets in the
# 0x100..0x140 range (reserved on real hardware - AFEC's last real
# register is AFE_WPSR at 0xE8). The renode_hooks function poking
# these offsets uses a 12-bit raw value (klipper's ADC_MAX=4095),
# matching the LCDR/CDR width.
#
# Magic offsets:
#   0x100 - write sets / read returns the default conversion value
#           (used when no per-channel override is set)
#   0x104..0x130 - per-channel overrides for channels 0..11 (4 bytes
#                   each); 0x104 is channel 0, 0x108 is channel 1, ...
#
# Real AFEC writes (AFE_CR / AFE_MR / AFE_CHER / etc.) are stored
# without side effects.

if 'init_done' not in dir():
    init_done = False
    selected_channel = 0
    default_value = 0
    channel_values = {}
    regs = {}
    # ISR.DRDY synthesis is gated on AFE_CR_START having been written
    # at least once since the last reset. See header comment.
    drdy_armed = False

AFE_CR    = 0x00
AFE_CHSR  = 0x1C
AFE_LCDR  = 0x20
AFE_ISR   = 0x30
AFE_CSELR = 0x64
AFE_CDR   = 0x68

# AFE_CR bit definitions (lib/sam4e/include/component/afec.h /
# lib/same70b/include/component/afec.h - SWRST and START are bits 0
# and 1 in both chip families).
AFE_CR_SWRST = 0x1
AFE_CR_START = 0x2

MAGIC_DEFAULT = 0x100
MAGIC_CH_BASE = 0x104  # 0x104 + ch * 4, channels 0..11

if request.IsInit:
    init_done = True
    selected_channel = 0
    drdy_armed = False
    regs.clear()
    # default_value / channel_values intentionally preserved across
    # reset: renode_hooks pushes fixture values BEFORE Renode's `start`
    # is issued, so we must not wipe them when the CPU's reset vector
    # runs.
elif request.IsWrite:
    val = int(request.Value)
    off = request.Offset
    if off == AFE_CR:
        # gpio_afec_init writes SWRST then later gpio_adc_sample writes
        # START. SWRST returns the synthesised ISR state to "no
        # conversion has happened" so init_afec()'s busy-check passes;
        # START arms ISR.DRDY so the very next ISR poll observes the
        # synthetic conversion-complete.
        if val & AFE_CR_SWRST:
            drdy_armed = False
        if val & AFE_CR_START:
            drdy_armed = True
        regs[off] = val
    elif off == AFE_CSELR:
        selected_channel = val & 0x1F
    elif off == MAGIC_DEFAULT:
        default_value = val & 0xFFF
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 12 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        channel_values[ch] = val & 0xFFF
    else:
        regs[off] = val
elif request.IsRead:
    off = request.Offset
    if off == AFE_LCDR or off == AFE_CDR:
        v = channel_values.get(selected_channel, default_value)
        request.Value = v & 0xFFF
    elif off == AFE_ISR:
        if drdy_armed:
            # DRDY (bit 24) | per-channel EOC bits 11:0. Returning all
            # channels ready is harmless - klipper only checks the bit
            # for the channel it's currently sampling.
            request.Value = (1 << 24) | 0xFFF
        else:
            # Pre-START / post-SWRST. init_afec()'s busy-check requires
            # DRDY=0 here to fall through to the AFEC configuration
            # block; without this gate gpio_afec_init spins forever.
            request.Value = 0
    elif off == AFE_CHSR:
        # All 12 channels enabled. klipper's gpio_adc_sample early-
        # exits if the requested channel isn't in CHSR, but it also
        # writes CHER first, so any nonzero answer that includes the
        # active channel is sufficient.
        request.Value = 0xFFF
    elif off == MAGIC_DEFAULT:
        request.Value = default_value
    elif MAGIC_CH_BASE <= off < MAGIC_CH_BASE + 12 * 4:
        ch = (off - MAGIC_CH_BASE) // 4
        request.Value = channel_values.get(ch, default_value)
    else:
        request.Value = regs.get(off, 0)
