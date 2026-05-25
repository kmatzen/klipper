#!/usr/bin/env python3
# Renode-backed bridge for klipper firmware testing.
#
# Spawns `renode` with a per-chip platform script that loads the
# klipper firmware ELF over USART1, exposes a host pty for klippy to
# connect to, and accepts the same line-oriented fixture control
# protocol as test/emulator/simavr_bridge.c. Fixture commands sent to
# --control-socket are translated into Renode Monitor commands sent
# over Renode's TCP Monitor port (-P), which in turn dispatch to the
# Python hook functions defined in test/emulator/renode_hooks.py
# (loaded into Renode at startup).
#
# Argument shape mirrors simavr_bridge so scripts/test_klippy.py can
# spawn either backend with the same kwargs - --sim-time-file and
# --tick-socket are accepted-and-ignored on first cut (Renode is
# real-time only until lockstep stepping is wired up; klippy in pure
# real-time mode falls back to live USART-based clock estimation,
# which is the production code path).

import argparse
import json
import os
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

# Per-chip Renode platform .repl path (relative to the Renode install).
# Most chips use upstream platforms verbatim; chips Renode does not
# ship a .repl for (currently just stm32h723) live under
# test/emulator/repl/ in this repo and are referenced by absolute path
# at .resc render time.
_LOCAL_REPL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'repl')
def _local(name):
    return '@' + os.path.join(_LOCAL_REPL_DIR, name)


# Platform map. Local overrides (test/emulator/repl/) extend the
# upstream Renode platforms with peripherals upstream doesn't model
# but klipper firmware initialises - notably ADC (missing on
# stm32f1.repl + stm32f4.repl + stm32f429.repl in Renode upstream)
# and SPI (missing on stm32f103.repl). For chips whose upstream
# platform already covers everything klipper needs (F0 / G0 / H7
# variants in scope today) we point at upstream verbatim. SAM4S8C
# memory layout (512KB flash + 128KB sram) matches sam4s8b, so we
# point at upstream sam4s8b.repl rather than carrying a duplicate
# locally.
_PLATFORM_FOR_CHIP = {
    'stm32f070': '@platforms/cpus/stm32f072.repl',
    'stm32f103': _local('stm32f103.repl'),
    'stm32f401': _local('stm32f4.repl'),
    'stm32f405': _local('stm32f4.repl'),
    'stm32f407': _local('stm32f4.repl'),
    'stm32f429': _local('stm32f429.repl'),
    'stm32f446': _local('stm32f4.repl'),
    'stm32g0b1': _local('stm32g0b1.repl'),
    'stm32h723': _local('stm32h723.repl'),
    'stm32h743': '@platforms/cpus/stm32h743.repl',
    'sam3x8c': _local('sam3x8e.repl'),
    'sam3x8e': _local('sam3x8e.repl'),
    # Local copy of upstream sam4s.repl that OMITS the upstream
    # `adc: Analog.SAM4S_ADC` so test/emulator/sam4s_adc_stub.py can
    # claim 0x40038000. Memory layout matches sam4s8b (512KB flash +
    # 128KB sram) which is the same as sam4s8c.
    'sam4s8c': _local('sam4s8c.repl'),
    'sam4e8e': _local('sam4e8e.repl'),
    'same70q20b': _local('same70q20b.repl'),
    'same70q20b-usb': _local('same70q20b.repl'),
    'samd21g18': _local('samd21g18.repl'),
    'samd51p20': _local('samd51p20.repl'),
    'lpc176x': _local('lpc176x.repl'),
    'hc32f460': _local('hc32f460.repl'),
    'rp2040': _local('rp2040.repl'),
}

# Renode peripheral name for the UART/USART that klipper uses as the
# host link in -serial.config mode. STM32 -serial.config selects
# USART1 family-wide. Atmel chips don't share that convention -
# klipper's src/atsam/serial.c picks a chip-specific Atmel UART
# peripheral (UART1 on SAM4S, UART2 on SAME70) which the platform
# .repl exposes under different names; klipper's src/atsamd/serial.c
# always picks SERCOM0 on the SAMx5 family. Per-chip overrides keyed
# by the same chip basename as _PLATFORM_FOR_CHIP; chips not in the
# override map fall back to the STM32 default.
_DEFAULT_HOST_LINK_PERIPHERAL = 'usart1'
_HOST_LINK_FOR_CHIP = {
    'sam3x8c': 'uart',
    'sam3x8e': 'uart',
    'sam4s8c': 'uart1',
    'sam4e8e': 'uart0',
    'same70q20b': 'uart2',
    # The USB-CDC SAME70 build wires host I/O through USBHS instead of
    # UART2; renode_launcher's UartPtyTerminal connector binds to whatever
    # peripheral name lives here, and SAM_USBHS exposes UARTBase on its
    # bulk endpoints (skip-enum cheat in same70_usbhs.cs).
    'same70q20b-usb': 'usbhs',
    'samd21g18': 'sercom0',
    'samd51p20': 'sercom0',
    'lpc176x': 'uart0',
    'hc32f460': 'usart1',
    'rp2040': 'uart0',
}


def _host_link_peripheral(chip):
    return _HOST_LINK_FOR_CHIP.get(chip, _DEFAULT_HOST_LINK_PERIPHERAL)

# Where renode_hooks.py lives, to be loaded into Renode at startup via
# `include @<path>` so all the hook functions (step_trigger, bltouch,
# adc_default, i2c_register_response, ...) are in scope before any
# fixture command arrives.
_HOOKS_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'renode_hooks.py')

_RCC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'rcc_stub.py')

_AFEC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'afec_stub.py')

# SAM4S has an `ADC` peripheral (not AFEC) at 0x40038000 with a
# different register layout. Renode upstream's Analog.SAM4S_ADC model
# accepts the register writes but never feeds samples, so all channels
# read 0 - which trips adc_scaled's (vref - vssa) divisor in klippy.
# This stub mimics the AFEC stub but with the ADC register map.
_SAM4S_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'sam4s_adc_stub.py')
# STM32F1 ADC stub. Upstream Renode's Analog.STM32_ADC doesn't complete
# klipper's SWSTART -> STRT/EOC -> SQR3/DR software-trigger handshake
# (src/stm32/adc.c), so no analog_in samples reach klippy and every
# heater reads temp=0.0. This stub models that register slice.
_STM32_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'stm32_adc_stub.py')
# RP2040 ADC stub. The local rp2040.repl models no ADC at all (the
# 0x4004C000 region is unmapped), so klipper's thermistor reads return
# 0. This stub drives klipper's RP2040 ADC handshake (src/rp2040/adc.c).
_RP2040_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'rp2040_adc_stub.py')
# STM32G0 ADC stub. The local stm32g0b1.repl OMITS upstream's
# `adc: Analog.STM32G0_ADC` so this stub can claim 0x40012400 and
# complete klipper's G0 ISR/CR/CHSELR/DR handshake (src/stm32/stm32f0_adc.c)
# with fixture-poked values - the F0/G0 ADC register layout differs from
# the F1/F4 ADC, so it needs its own stub.
_STM32G0_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    'stm32g0_adc_stub.py')
# STM32H7 ADC stub. The vendored stm32h723.repl OMITS upstream's
# `adcM1S2: Analog.STM32F0_ADC @ 0x40022000` (the F0 model spins
# klipper's H7 ADCAL loop and returns no controllable value); this stub
# completes the H7 ISR/CR/SQR1/DR handshake (src/stm32/stm32h7_adc.c). It
# is registered via _EXTRA_PERIPHERAL_STUBS_FOR_CHIP (not the 0x200 AFEC
# block) because its window must be 0x400 bytes to also cover the ADC12
# common CCR at base+0x308.
_STM32H7_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    'stm32h7_adc_stub.py')
# HC32F460 ADC stub. The hc32f460.repl maps no ADC peripheral, so this
# Python stub is the only thing at M4_ADC1's base 0x40040000. It completes
# klipper's HC32 STR/ISR/DR handshake (src/hc32f460/adc.c) with fixture-
# poked values; the HDSC ADC register layout matches no upstream model.
_HC32F460_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'hc32f460_adc_stub.py')
# SAMD51 / SAME54 ADC stub. The samd51p20.repl maps no ADC, so this stub
# claims both ADC0 (0x43001C00) and ADC1 (0x43002000). It completes
# klipper's SAMX5 INPUTCTRL/SWTRIG/INTFLAG/RESULT handshake
# (src/atsamd/adc.c) and returns SYNCBUSY=0 so adc_init's busy-waits
# fall through. The SAMX5 ADC register layout differs from SAMD21's, so
# it needs its own stub.
_SAMD51_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'samd51_adc_stub.py')
# SAMD21 ADC stub. The samd21g18.repl maps no ADC, so this stub claims
# the single ADC @ 0x42004000. The SAMD21 ADC register layout differs
# from the SAMX5 one (SWTRIG@0x0C, INPUTCTRL@0x10, INTFLAG@0x18,
# RESULT@0x1A, and no SYNCBUSY), so it needs its own stub.
_SAMD21_ADC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'samd21_adc_stub.py')

# SAME70 EFC stub. klipper same70_sysinit.c reads EFC->EEFC_FRR to
# check GPNVM TCM bits 7+8; Renode's SVD-tagged EFC returns 0, so
# the firmware enters the "configure GPNVM and request reset" branch
# and spins in `for(;;)` waiting for an RSTC reset Renode has no
# model for. The stub overrides FRR to return GPNVM_TCM_MASK so the
# firmware sees TCM as already-set and skips the spin.
_SAME70_EFC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'same70_efc_stub.py')

# SAME70 XDMAC channel-0 busy-wait synthesis. klipper SystemInit
# polls XDMAC->XDMAC_CIS0 for BIS after kicking off a flash->ITCM
# DMA copy; Renode's SVD-tagged XDMAC returns 0, so BIS never latches
# and the firmware spins. The stub overrides CIS0 to BIS=1 (no DMA
# actually needed - useVirtualAddress=true loads the ELF at VMA
# directly). See same70_xdmac_stub.py for the full reasoning.
_SAME70_XDMAC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'same70_xdmac_stub.py')

_SAMD_OSCCTRL_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'samd_oscctrl_stub.py')
_SAMD_GCLK_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'samd_gclk_stub.py')
_SAMD_OSC32KCTRL_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'samd_osc32kctrl_stub.py')
_SAMD_STOREBACK_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'samd_storeback.py')

_SAMD21_GCLK_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'samd21_gclk_stub.py')
_SAMD21_SYSCTRL_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'samd21_sysctrl_stub.py')

_LPC_SC_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'lpc_sc_stub.py')

# STM32H7 PWR stub. Upstream stm32h743.repl models RCC + the flash
# controller but leaves PWR (0x58024800) as a bare Tag, so klipper's
# clock_setup() spins on PWR->CSR1.ACTVOSRDY / PWR->D3CR.VOSRDY before
# it reaches RCC. The stub synthesizes those two ready bits.
_STM32H7_PWR_STUB_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    'stm32h7_pwr_stub.py')

# Path to the HC32F460 USART C# peripheral. Loaded into the running
# Renode runtime via `i @<path>` (Roslyn-compiled into the live
# process by IncludeFileCommand) BEFORE any LoadPlatformDescription
# call so the .repl's `UART.HC32F460_USART` reference resolves at
# platform-load time. HDSC has no upstream Renode UART model whose
# register layout matches its USART, so we ship one as a small .cs
# under test/emulator/repl/.
_HC32F460_UART_CS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'repl', 'hc32f460_uart.cs')

# Path to the RP2040 64-bit timer C# peripheral. Renode upstream has no
# RP2040-family timer model; we ship one under test/emulator/repl/ and
# load it the same way as hc32f460_uart.cs (Roslyn-compiled into the
# running runtime via `i @<path>` before LoadPlatformDescription).
_RP2040_TIMER_CS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'repl', 'rp2040_timer.cs')

# Path to the SAME70 USBHS C# peripheral. Loaded into the running
# Renode runtime via `i @<path>` (Roslyn-compiled into the live process
# by IncludeFileCommand) BEFORE the platform LoadPlatformDescription so
# the .repl's `USB.SAM_USBHS` reference resolves at platform-load
# time. Renode upstream has no SAM USB-OTG peripheral model.
_SAME70_USBHS_CS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'repl', 'same70_usbhs.cs')

# Path to the LPC176x ADC C# peripheral. klipper's LPC ADC driver
# (src/lpc176x/adc.c) is interrupt-driven - gpio_adc_sample() arms a
# burst conversion and only completes once ADC_IRQHandler has run five
# times - so a polled register-storeback Python stub (like the AFEC /
# SAM4S / STM32 / RP2040 ADC stubs) can't drive it: nothing raises the
# ADC IRQ, the ISR never runs, and thermistors read temp=0.0. This C#
# model raises the ADC line (GPIO IRQ -> nvic@22). Loaded via `i @<path>`
# before the .repl parses (the .repl references Analog.LPC176x_ADC), same
# path as rp2040_timer.cs.
_LPC_ADC_CS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'repl', 'lpc176x_adc.cs')

# Per-stub-region paths for the RP2040 clock-controller surface. Each
# script implements one clock peripheral's busy-wait status synthesis
# (RESET_DONE = ~RESET, XOSC.STATUS.STABLE from CTRL.ENABLE, PLL.CS.LOCK
# from PLL.PWR.PD, CLK_<x>_SELECTED from CLK_<x>_CTRL.SRC) - splitting
# them out keeps each script's `regs` dict per-instance rather than
# trying to multiplex across base addresses.
_RP2040_CLOCKS_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'rp2040_clocks_stub.py')
_RP2040_RESETS_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'rp2040_resets_stub.py')
_RP2040_XOSC_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'rp2040_xosc_stub.py')
_RP2040_PLL_STUB_PY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'rp2040_pll_stub.py')

# Per-chip RCC peripheral base address (RM cross-reference per
# family). Most upstream Renode STM32 platforms ship some kind of
# RCC model: F4 has Miscellaneous.STM32F4_RCC, H7 has
# Miscellaneous.STM32H7_RCC, F0/G0 have their own
# Python.PythonPeripheral stubs at 0x40021000. STM32F1 is the
# exception - upstream stm32f103.repl has no rcc entry at all and
# the firmware spins on HSERDY/PLLRDY. We register our local
# rcc_stub.py only for F1.
#
# Adding a stub at an address that already has an RCC peripheral
# defined causes the platform load to fail silently (Renode
# discards the .resc remainder), so it's important the entries below
# stay restricted to chips whose upstream platform truly lacks RCC.
_RCC_BASE_FOR_CHIP = {
    'stm32f103': 0x40021000,
}

# Per-chip AFEC peripheral base addresses. SAME70 has AFEC0 / AFEC1
# (12 channels each); klipper's src/atsam/sam4e_afec.c initialises
# both. Renode does not ship a SAM_AFEC peripheral model, so we
# register Python.PythonPeripheral stubs (test/emulator/afec_stub.py)
# at each base. The stub serves AFE_LCDR / AFE_CDR reads with values
# poked through magic offsets by the renode_hooks adc_default /
# adc_set path.
_AFEC_BASES_FOR_CHIP = {
    'sam4e8e': (0x400B0000, 0x400B4000),
    'same70q20b': (0x4003C000, 0x40064000),
    'same70q20b-usb': (0x4003C000, 0x40064000),
    # SAM4S has an ADC (not AFEC) at 0x40038000. Local sam4s8c.repl
    # OMITS the upstream `adc: Analog.SAM4S_ADC` so this Python stub
    # can claim the address.
    'sam4s8c': (0x40038000,),
    # SAM3X ADC at 0x400C0000. src/atsam/adc.c is shared SAM3X/SAM4S
    # and the register layout is identical (CR/CHER/CHDR/CHSR/LCDR/ISR),
    # so the SAM4S ADC stub serves SAM3X verbatim at the SAM3X base. The
    # sam3x8e.repl maps no ADC there, so the stub owns the address.
    'sam3x8e': (0x400C0000,),
    'sam3x8c': (0x400C0000,),
    # STM32F1 ADC1 at 0x40012400. Local stm32f103.repl OMITS the
    # upstream `adc1: Analog.STM32_ADC` so this Python stub can claim
    # the address (klipper's F1 configs read every thermistor on ADC1).
    'stm32f103': (0x40012400,),
    # STM32F4 ADC1 at 0x40012000. The local stm32f4.repl maps no Analog
    # peripheral (upstream omits them too), so the stub is the only thing
    # at 0x40012000; its 0x200 window also covers the unused ADC2
    # (0x40012100). klipper's F4 driver reads thermistor channels 0..15
    # on ADC1 (src/stm32/adc.c).
    'stm32f446': (0x40012000,),
    # STM32G0 ADC1 at 0x40012400 (same base as F1, different register
    # layout). The local stm32g0b1.repl OMITS upstream's
    # `adc: Analog.STM32G0_ADC` so this stub can claim the address.
    'stm32g0b1': (0x40012400,),
    # RP2040 ADC at 0x4004C000. The local rp2040.repl models no ADC, so
    # this Python stub is the only thing mapped there.
    'rp2040': (0x4004C000,),
    # HC32F460 M4_ADC1 at 0x40040000. The hc32f460.repl maps no ADC, so
    # this stub claims the address (0x200 window covers DR0..DR16 at
    # 0x50..0x72 plus the magic-offset poke region at 0x100+).
    'hc32f460': (0x40040000,),
    # SAMD51 / SAME54 ADC0 + ADC1. The samd51p20.repl maps no ADC; the
    # stub claims both (0x200 windows don't overlap: ADC0 ends at
    # 0x43001DFF, ADC1 starts at 0x43002000). Each base gets its own
    # stub instance; klipper routes chan<16 -> ADC0, chan>=16 -> ADC1.
    'samd51p20': (0x43001C00, 0x43002000),
    # SAMD21 single ADC at 0x42004000. The samd21g18.repl maps no ADC.
    'samd21g18': (0x42004000,),
    # NB: STM32H7 ADC1 (0x40022000) is NOT here - it is registered via
    # _EXTRA_PERIPHERAL_STUBS_FOR_CHIP['stm32h723'] with a 0x400 window
    # (the AFEC block hardcodes 0x200, too small to reach the ADC12
    # common CCR at base+0x308). The renode_hooks _AFEC_BASES poke list
    # still includes 0x40022000 so adc_set/adc_default reach it.
}

# Override the default afec_stub.py per chip. The SAM4S ADC uses
# different registers (ADC_CHER/CHDR/CHSR/CR/ISR/LCDR) from AFEC
# (AFE_CR/CHSR/LCDR/ISR/CSELR/CDR), so it needs its own stub.
_ADC_STUB_FOR_CHIP = {
    'sam4s8c': _SAM4S_ADC_STUB_PY,
    # SAM3X shares src/atsam/adc.c with SAM4S - same register map, so the
    # same stub (at the SAM3X base 0x400C0000 above).
    'sam3x8e': _SAM4S_ADC_STUB_PY,
    'sam3x8c': _SAM4S_ADC_STUB_PY,
    'stm32f103': _STM32_ADC_STUB_PY,
    # F4 shares the F1 SR/CR2/SQR3/DR stub (one register slice, one
    # SWSTART-bit difference the stub absorbs).
    'stm32f446': _STM32_ADC_STUB_PY,
    'stm32g0b1': _STM32G0_ADC_STUB_PY,
    'rp2040': _RP2040_ADC_STUB_PY,
    'hc32f460': _HC32F460_ADC_STUB_PY,
    'samd51p20': _SAMD51_ADC_STUB_PY,
    'samd21g18': _SAMD21_ADC_STUB_PY,
}

# Per-chip extra Python.PythonPeripheral stubs to inject after the
# platform load. Each entry is a list of (name, base, size, stub_path)
# tuples. The samd51p20 entries cover the chip's clock controllers
# (OSCCTRL / GCLK / OSC32KCTRL) and the simple store/return blocks
# (MCLK / CMCC) that klipper firmware touches during SystemInit() and
# enable_pclock() - none of which Renode upstream models. Without
# these the firmware spins forever in samd51_clock.c.
_SAME70_EXTRA_STUBS = [
    # EFC at 0x400E0C00 with 0x10-byte register window (FMR/FCR/FSR/FRR).
    # See same70_efc_stub.py for the GPNVM-bypass synthesis.
    ('efc', 0x400E0C00, 0x10, _SAME70_EFC_STUB_PY),
    # XDMAC channel-0 register block at 0x40078050..0x4007808F covers
    # CIE0/CID0/CIM0/CIS0/CSA0/CDA0/CNDA0/CNDC0/CUBC0/CBC0/CC0/...
    # Stub synthesises CIS0.BIS = 1 to short-circuit the flash->ITCM
    # DMA-copy busy-wait in same70_sysinit.c (the actual copy is
    # unnecessary because useVirtualAddress=true loads the ELF at VMA
    # 0x0 directly, same place the DMA would put it).
    ('xdmac_ch0', 0x40078050, 0x40, _SAME70_XDMAC_STUB_PY),
]


_EXTRA_PERIPHERAL_STUBS_FOR_CHIP = {
    # SAMD21G18: standalone .repl declares CPU/NVIC/SRAM/flash/SERCOM0
    # /TC4/PORT and the auxiliary fuse rows; this list adds the clock
    # peripherals (GCLK / SYSCTRL with custom synthesis stubs) and the
    # plain store/return regions klipper firmware writes during
    # SystemInit + enable_pclock (PM APBxMASK, NVMCTRL CTRLB,
    # WDT CONFIG/CTRL/CLEAR). None of the storeback regions are read
    # back to gate boot progress, so samd_storeback.py covers them.
    #   - gclk    (0x40000C00, 0x20)  CTRL.SWRST self-clear +
    #                                 STATUS.SYNCBUSY=0 synthesis.
    #   - sysctrl (0x40000800, 0x80)  PCLKSR.{XOSC32KRDY,DFLLRDY} +
    #                                 DPLLSTATUS.{LOCK,CLKRDY}
    #                                 synthesised from the matching
    #                                 ENABLE writes; otherwise storeback.
    #   - pm      (0x40000400, 0x80)  APB{A,B,C}MASK clock-enable bits.
    #   - nvmctrl (0x41004000, 0x80)  CTRLB wait-state cfg.
    #   - wdt     (0x40001000, 0x10)  watchdog_init writes
    #                                 CONFIG/CTRL; watchdog_reset
    #                                 polls STATUS.SYNCBUSY (default
    #                                 0 from storeback) and writes
    #                                 CLEAR.
    'samd21g18': [
        ('gclk', 0x40000C00, 0x20, _SAMD21_GCLK_STUB_PY),
        ('sysctrl', 0x40000800, 0x80, _SAMD21_SYSCTRL_STUB_PY),
        ('pm', 0x40000400, 0x80, _SAMD_STOREBACK_PY),
        ('nvmctrl', 0x41004000, 0x80, _SAMD_STOREBACK_PY),
        ('wdt', 0x40001000, 0x10, _SAMD_STOREBACK_PY),
    ],
    'samd51p20': [
        ('oscctrl', 0x40001000, 0x80, _SAMD_OSCCTRL_STUB_PY),
        ('osc32kctrl', 0x40001400, 0x40, _SAMD_OSC32KCTRL_STUB_PY),
        ('gclk', 0x40001C00, 0x200, _SAMD_GCLK_STUB_PY),
        ('mclk', 0x40000800, 0x40, _SAMD_STOREBACK_PY),
        ('cmcc', 0x41006000, 0x40, _SAMD_STOREBACK_PY),
    ],
    # LPC176x: the LPC_SC clock-controller stub (SystemInit busy-wait
    # synthesis) plus the fast-GPIO block as plain storeback. klipper's
    # LPC GPIO (src/lpc176x/gpio.c) lives at LPC_GPIO_BASE 0x2009C000 with
    # five 0x20-spaced port blocks (FIODIR 0x00, FIOMASK 0x10, FIOPIN
    # 0x14, FIOSET 0x18, FIOCLR 0x1C). Without a mapped region the
    # firmware's GPIO writes hit unmapped space (harmless for the boot
    # smoke test) but gpio_in_read (FIOPIN & bit) can't be driven, which
    # the TMC2208/2209 single-wire UART responder needs: renode_hooks
    # _drive_rx writes the FIOPIN bit for the firmware-side RX pin (same
    # idea as the RP2040 SIO GPIO_IN write, but at the LPC FIOPIN offset).
    # Plain dict storeback suffices - the firmware never reads a GPIO
    # register back to gate boot progress, and FIOSET/FIOCLR don't need
    # to mirror into FIOPIN because the only readback we care about (the
    # RX pin) is driven directly by _drive_rx. 0xA0 covers all five
    # ports.
    'lpc176x': [
        ('lpc_sc', 0x400FC000, 0x200, _LPC_SC_STUB_PY),
        ('gpio', 0x2009C000, 0xA0, _SAMD_STOREBACK_PY),
    ],
    # STM32H7: PWR power-control block. Upstream stm32h743.repl leaves
    # this address as a logging Tag; the stub serves CSR1.ACTVOSRDY and
    # D3CR.VOSRDY so clock_setup()'s pre-RCC busy-waits complete. Both
    # h723 (local repl) and h743 (upstream repl) need it. The 0x400-byte
    # window matches the Tag <0x58024800, 0x58024BFF> "PWR" range.
    'stm32h723': [
        ('pwr', 0x58024800, 0x400, _STM32H7_PWR_STUB_PY),
        # ADC1 at 0x40022000 with a 0x400 window (covers ADC1 registers,
        # the magic-offset poke region at +0x100, and the ADC12 common
        # CCR at +0x308). The vendored stm32h723.repl omits upstream's
        # adcM1S2 so this stub owns the address. renode_hooks _AFEC_BASES
        # includes 0x40022000 so adc_set/adc_default poke it.
        ('adc1', 0x40022000, 0x400, _STM32H7_ADC_STUB_PY),
    ],
    'stm32h743': [
        ('pwr', 0x58024800, 0x400, _STM32H7_PWR_STUB_PY),
    ],
    # HC32F460: every region the firmware writes during init except
    # USART1 (which has a real model in the .repl). None of these
    # peripherals are read back to gate boot progress, so plain
    # storeback (samd_storeback.py: dict-backed read-after-write,
    # zero default for unread offsets) is sufficient. See
    # test/emulator/repl/hc32f460.repl for the reasoning per region.
    #   - efm  (0x40010400, 0x400) covers HRC_FREQ_MON @ 0x40010684
    #     read by SystemCoreClockUpdate; default 0 -> firmware picks
    #     the HRC_20MHz_VALUE branch and SystemCoreClock = HRC_VALUE.
    #   - mstp (0x40048000, 0x20) clock-enable bits written by
    #     PWC_Fcg1PeriphClockCmd via M4_MSTP->FCG{0,1,2,3}.
    #   - intc (0x40051000, 0x800) IrqRegistration's source -> NVIC
    #     line remap writes; static .repl wiring of usart1 IRQs to
    #     nvic@0/1/2/3 supersedes the dynamic mapping.
    #   - port (0x40053800, 0x80) PORT_Unlock/PSPCR/PCCR/PFSR writes
    #     from PORT_DebugPortSetting + PORT_SetFunc.
    #   - sysreg (0x40054000, 0x1400) PWR_FPRC unlock and CMU_CKSWR
    #     reads. Default 0 on CMU_CKSWR.CKSW maps to "internal HRC"
    #     in SystemCoreClockUpdate, which avoids the PLL setup path
    #     entirely.
    #   - aos (0x40010800, 0x100) trigger-source select writes; not
    #     read back by any current consumer.
    'hc32f460': [
        ('efm', 0x40010400, 0x400, _SAMD_STOREBACK_PY),
        ('aos', 0x40010800, 0x100, _SAMD_STOREBACK_PY),
        ('mstp', 0x40048000, 0x20, _SAMD_STOREBACK_PY),
        ('intc', 0x40051000, 0x800, _SAMD_STOREBACK_PY),
        ('port', 0x40053800, 0x80, _SAMD_STOREBACK_PY),
        ('sysreg', 0x40054000, 0x1400, _SAMD_STOREBACK_PY),
    ],
    # RP2040: every region klipper firmware writes during init except
    # UART0 / UART1 (UART.PL011 in the .repl) and TIMER (Timers.
    # RP2040_Timer C# model loaded via _CSHARP_INCLUDES_FOR_CHIP).
    # Four regions need bit-set synthesis to clear busy-wait loops in
    # main.c:clock_setup; the rest are plain dict-backed storeback
    # because the firmware writes once and never reads back to gate
    # progress.
    #   - clocks (0x40008000, 0xD0)  CLK_<x>_SELECTED = 1<<CTRL.SRC.
    #   - resets (0x4000C000, 0x10)  RESET_DONE = ~RESET.
    #   - xosc   (0x40024000, 0x20)  STATUS.STABLE when
    #                                CTRL.ENABLE field == 0xFAB.
    #   - pll_sys (0x40028000, 0x10) CS.LOCK when PWR.PD bit cleared.
    #   - pll_usb (0x4002C000, 0x10) same script as pll_sys; per-
    #                                instance regs keep state separate.
    #   - psm (0x40010000, 0x10)     watchdog source-select; klipper
    #                                writes WDSEL but never reads.
    #   - io_bank0 (0x40014000, 0x180) PORT_SetFunc per-pin CTRL writes.
    #   - pads_bank0 (0x4001C000, 0x100) PADS_SetPad drive/pull writes.
    #   - watchdog (0x40058000, 0x40) watchdog_init writes load/ctrl;
    #                                clock_setup writes tick (RP2040
    #                                only - RP2350 watchdog tick lives
    #                                under the ticks block, which
    #                                klipper RP2040 firmware doesn't
    #                                touch).
    #   - vreg (0x40064000, 0x10)    set_vsel writes VREG.VSEL field.
    #   - rosc (0x40060000, 0x20)    not driven by klipper directly,
    #                                but covered for safety - the
    #                                cmsis startup may probe it.
    'rp2040': [
        ('psm', 0x40010000, 0x10, _SAMD_STOREBACK_PY),
        ('clocks', 0x40008000, 0xD0, _RP2040_CLOCKS_STUB_PY),
        ('resets', 0x4000C000, 0x10, _RP2040_RESETS_STUB_PY),
        ('io_bank0', 0x40014000, 0x180, _SAMD_STOREBACK_PY),
        ('pads_bank0', 0x4001C000, 0x100, _SAMD_STOREBACK_PY),
        ('xosc', 0x40024000, 0x20, _RP2040_XOSC_STUB_PY),
        ('pll_sys', 0x40028000, 0x10, _RP2040_PLL_STUB_PY),
        ('pll_usb', 0x4002C000, 0x10, _RP2040_PLL_STUB_PY),
        ('watchdog', 0x40058000, 0x40, _SAMD_STOREBACK_PY),
        ('rosc', 0x40060000, 0x20, _SAMD_STOREBACK_PY),
        ('vreg', 0x40064000, 0x10, _SAMD_STOREBACK_PY),
    ],
    # SAME70 - both the serial-mode and USB-CDC firmware variants need
    # the EFC GPNVM-bypass synthesis (klipper same70_sysinit.c reads
    # EEFC_FRR for TCM bits before any peripheral-specific code runs).
    'same70q20b': _SAME70_EXTRA_STUBS,
    'same70q20b-usb': _SAME70_EXTRA_STUBS,
}


def _adc_poke_bases(chip):
    # The ADC base(s) the renode_hooks adc_default / adc_set pokes should
    # write for THIS chip. Combines the AFEC-block bases
    # (_AFEC_BASES_FOR_CHIP) with any ADC stub registered through the
    # extra-peripheral path (the STM32H7 ADC needs a 0x400 window the
    # 0x200 AFEC block can't give, so it lives in
    # _EXTRA_PERIPHERAL_STUBS_FOR_CHIP under a name containing "adc").
    # The launcher hands this list to renode_hooks.set_adc_bases so the
    # poke never scribbles a base that aliases a live non-ADC peripheral
    # on a different chip (e.g. 0x40022000 = H7 ADC1 but G0 flash ctrl).
    bases = list(_AFEC_BASES_FOR_CHIP.get(chip, ()))
    for name, base, _size, _stub in _EXTRA_PERIPHERAL_STUBS_FOR_CHIP.get(
            chip, ()):
        if 'adc' in name and base not in bases:
            bases.append(base)
    return bases

# Per-chip override controlling whether `sysbus LoadELF` uses the
# segment's virtual address (VMA, the address the firmware code
# actually executes from) or its physical address (LMA, where the
# bytes get flashed on real hardware).
#
# SAME70 firmware compiled by `src/atsam/same70_link.lds.S` has
# .text VMA=0x0..N (rom origin = CONFIG_FLASH_APPLICATION_ADDRESS = 0x0)
# but LMA=0x400000..0x400000+N (the AT() clause sets LMA to
# CONFIG_ARMCM_ITCM_FLASH_MIRROR_START). On real silicon, the
# matrix-controller mirrors 0x400000 onto 0x0 at reset, so the CPU's
# initial SP/PC read at 0x0/0x4 hits the vector table. Renode does
# not model the matrix-controller mirror, so loading at LMA leaves
# 0x0..0x100 zeroed and the CPU halts on its first fetch with
# `PC does not lay in memory or PC and SP are equal to zero`.
# Loading at VMA puts the bytes where the CPU expects them, which is
# functionally equivalent to running on real silicon post-mirror-setup.
#
# All other Atmel chips (sam3x, sam4s, sam4e) use the generic
# armcm linker script with VMA == LMA, so loading at LMA (the
# default) works without override.
_USE_VIRTUAL_ELF_LOAD = {
    'same70q20b': True,
    'same70q20b-usb': True,
}


# Per-chip C# peripherals to load (Roslyn-compiled into the running
# Renode runtime via `i @<path>` / IncludeFileCommand) before the
# .repl is parsed. Used when an upstream Renode peripheral model
# either doesn't exist for the chip or has a register layout that
# doesn't match the chip's vendor library. Today only HC32F460 needs
# this (its USART register set isn't a match for any of the existing
# UART/USART models in renode-infrastructure); the .cs file lives at
# test/emulator/repl/hc32f460_uart.cs and exposes
# UART.HC32F460_USART for reference from the .repl.
_CSHARP_INCLUDES_FOR_CHIP = {
    'hc32f460': [_HC32F460_UART_CS],
    'rp2040': [_RP2040_TIMER_CS],
    'lpc176x': [_LPC_ADC_CS],
    # Both same70q20b chip keys load the USBHS .cs even though only
    # the USB-mode firmware exercises the peripheral - the .repl
    # references USB.SAM_USBHS unconditionally so the type must
    # resolve at LoadPlatformDescription time. The serial-mode
    # firmware never writes to USBHS so the model stays idle.
    'same70q20b': [_SAME70_USBHS_CS],
    'same70q20b-usb': [_SAME70_USBHS_CS],
}


def _chip_for_elf(elf_path):
    # Klipper builds yield ci_build/elf/<chip>.elf; the chip basename is
    # the lookup key for _PLATFORM_FOR_CHIP. For oddball test paths
    # (e.g. a one-off ELF passed by hand) we accept any name that
    # matches a known chip prefix.
    base = os.path.basename(elf_path)
    if base.endswith('.elf'):
        base = base[:-4]
    if base in _PLATFORM_FOR_CHIP:
        return base
    for chip in _PLATFORM_FOR_CHIP:
        if base.startswith(chip):
            return chip
    raise RuntimeError(
        "renode_launcher: no Renode platform mapping for ELF %r "
        "(expected basename matching one of %s)"
        % (elf_path, sorted(_PLATFORM_FOR_CHIP)))


def _allocate_tcp_port():
    # Bind ephemeral, read back the port, close. The kernel keeps the
    # port in TIME_WAIT briefly but Renode's listen will succeed
    # because we set SO_REUSEADDR (and Mono's TcpListener uses it by
    # default). For multi-instance parallel test runs each launcher
    # gets a distinct port; SO_REUSEADDR isn't sufficient if two
    # launchers race for the same port, so we wrap with a retry loop
    # in the caller.
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _render_resc(chip, elf_path, pty_path, monitor_port, log_path,
                 tick_mode=False):
    # The .resc Renode runs on startup. mach create + LoadPlatform +
    # the RCC PythonPeripheral stub (so clock setup completes) +
    # LoadELF + UartPtyTerminal connected to USART1 + include of
    # renode_hooks.py + set_monitor(monitor) so the hook funcs can
    # resolve `monitor.Machine` (which their module namespace
    # otherwise can't see). We do NOT call `start` here - the
    # launcher sends `start` over the TCP Monitor after it has
    # finished pushing all fixture-driven hook registrations, so
    # peripheral state is configured before the CPU begins
    # executing.
    platform = _PLATFORM_FOR_CHIP[chip]
    rcc_base = _RCC_BASE_FOR_CHIP.get(chip)
    rcc_block = ''
    if rcc_base is not None:
        rcc_block = (
            'machine LoadPlatformDescriptionFromString '
            '"rcc: Python.PythonPeripheral @ sysbus 0x{rcc:08X} '
            '{{ size: 0x400; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(rcc=rcc_base, stub=_RCC_STUB_PY)
    afec_bases = _AFEC_BASES_FOR_CHIP.get(chip, ())
    afec_stub = _ADC_STUB_FOR_CHIP.get(chip, _AFEC_STUB_PY)
    afec_block = ''
    for idx, base in enumerate(afec_bases):
        afec_block += (
            'machine LoadPlatformDescriptionFromString '
            '"afec{idx}: Python.PythonPeripheral @ sysbus 0x{base:08X} '
            '{{ size: 0x200; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(idx=idx, base=base, stub=afec_stub)
    extra_stubs = _EXTRA_PERIPHERAL_STUBS_FOR_CHIP.get(chip, ())
    extra_block = ''
    for name, base, size, stub in extra_stubs:
        extra_block += (
            'machine LoadPlatformDescriptionFromString '
            '"{name}: Python.PythonPeripheral @ sysbus 0x{base:08X} '
            '{{ size: 0x{size:X}; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(name=name, base=base, size=size, stub=stub)
    # C# peripherals must be Roslyn-compiled into the runtime BEFORE
    # any LoadPlatformDescription that references them, otherwise the
    # platform parser fails to resolve the type. See
    # tests/unit-tests/bus_isolation.resc for the upstream pattern
    # (mach create -> include @x.cs -> LoadPlatformDescription).
    cs_includes = _CSHARP_INCLUDES_FOR_CHIP.get(chip, ())
    cs_include_block = ''.join('i @%s\n' % p for p in cs_includes)
    # SAME70 needs the ELF loaded TWICE - once at VMA so the vector
    # table sits at 0x0 where the CPU's reset fetch reads SP/PC (the
    # ITCM mirror real silicon would set up via the matrix-controller
    # remap is not modelled by Renode), and again at LMA so the .data
    # segment's flash-source bytes (referenced by `_data_flash` in
    # armcm_link.lds.S) are present at 0x400000+_text_size for
    # reset_handler_stage_two's boot_memcpy(&_data_start, &_data_flash,
    # ...) to copy into RAM. With only the VMA load the LMA region is
    # zero and boot_memcpy clobbers initialized variables (periodic_timer
    # / sentinel_timer / etc.) with garbage, which then trips
    # sched_add_timer's "timer too close" guard during ctr_run_initfuncs
    # and longjmps against an uninitialized shutdown_jmp.
    if _USE_VIRTUAL_ELF_LOAD.get(chip, False):
        elf_load_block = ('sysbus LoadELF @{elf} useVirtualAddress=true\n'
                          'sysbus LoadELF @{elf}\n').format(elf=elf_path)
    else:
        elf_load_block = 'sysbus LoadELF @{elf}\n'.format(elf=elf_path)
    # EXPERIMENT knobs (env-driven so they can be toggled without a
    # rebuild): RENODE_EXP_QUANTUM sets the emulation global sync
    # quantum in seconds (smaller = finer host/virtual interleaving =
    # less bursty clock, at the cost of speed); RENODE_EXP_MIPS pins
    # the CPU's reported MIPS. Both are no-ops when unset.
    timing_block = ''
    _exp_quantum = os.environ.get('RENODE_EXP_QUANTUM')
    if _exp_quantum:
        timing_block += 'emulation SetGlobalQuantum "%s"\n' % _exp_quantum
    _exp_mips = os.environ.get('RENODE_EXP_MIPS')
    if _exp_mips:
        timing_block += 'cpu PerformanceInMips %s\n' % _exp_mips
    # RENODE_EXP_BLOCKSIZE caps the CPU's translation-block size so the
    # CPU services a pending IRQ (e.g. SysTick) after fewer instructions.
    # The hypothesis was that a large block lets DWT->CYCCNT run past a
    # due timer deadline before SysTick_Handler fires, tripping
    # armcm_timer.c's ">1ms in the past" shutdown (reason 56). Empirically
    # this is NOT the cause for the SAME70 USB build: a sweep from 1..1024
    # left the reason-56 shutdown unchanged (the overshoot is not
    # IRQ-servicing-granularity-bound; it comes from the wall-clock vs
    # virtual-time drift that only deterministic tick-mode removes). Kept
    # as a diagnostic knob alongside MIPS/QUANTUM. No-op when unset.
    _exp_blocksize = os.environ.get('RENODE_EXP_BLOCKSIZE')
    if _exp_blocksize:
        timing_block += 'cpu MaximumBlockSize %s\n' % _exp_blocksize
    usart = _host_link_peripheral(chip)
    if tick_mode:
        # Tick mode: skip Renode's pty terminal entirely. The launcher
        # creates an openpty() pair itself and shuttles bytes through
        # renode_hooks.serial_init / serial_write_hex / serial_drain_hex
        # synchronously with each emulation RunFor.
        uart_block = ''
    else:
        uart_block = (
            'emulation CreateUartPtyTerminal "uartTerm" "%s"\n'
            'connector Connect sysbus.%s uartTerm\n'
        ) % (pty_path, usart)
    return (
        'using sysbus\n'
        'mach create "klipper-{chip}"\n'
        '{cs_include_block}'
        'machine LoadPlatformDescription {platform}\n'
        '{timing_block}'
        '{rcc_block}'
        '{afec_block}'
        '{extra_block}'
        '{elf_load_block}'
        'logFile @{log}\n'
        'logLevel 1\n'
        '{uart_block}'
        # Add the hooks directory to sys.path so `import renode_hooks`
        # resolves it as a module, then import and call set_monitor.
        # Earlier launcher iterations used `i @file.py` which evaluates
        # the file's contents in the Monitor's Python ScriptScope and
        # copies top-level names into the Monitor scope - which makes
        # bare-name step_trigger() calls work but leaves the *module*
        # namespace's `_M = None`, so cpu-PC-hook callbacks dispatched
        # by Renode (which run in the renode_hooks module scope, not
        # the Monitor scope) crash with 'NoneType has no attribute
        # Machine'. Importing as a real module and using the qualified
        # `renode_hooks.X(...)` form on the call side keeps a single
        # consistent namespace.
        'python "import sys; sys.path.append(\\"{hooks_dir}\\")"\n'
        'python "import renode_hooks; renode_hooks.set_monitor(monitor)"\n'
        # Restrict the ADC magic-offset poke to THIS chip's real ADC
        # base(s) so adc_set/adc_default can't scribble a base that
        # aliases a live non-ADC peripheral on another chip.
        'python "import renode_hooks; renode_hooks.set_adc_bases({adc_bases})"\n'
    ).format(chip=chip, platform=platform,
             log=log_path, rcc_block=rcc_block,
             afec_block=afec_block, extra_block=extra_block,
             cs_include_block=cs_include_block,
             elf_load_block=elf_load_block,
             uart_block=uart_block, timing_block=timing_block,
             adc_bases=repr(_adc_poke_bases(chip)),
             hooks_dir=os.path.dirname(_HOOKS_PY))


def _wait_for_path(path, deadline, poll=0.05):
    while time.monotonic() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(poll)
    return False


def _connect_monitor(host, port, deadline):
    # Renode's TCP monitor takes a moment to come up after process
    # spawn. Retry until either it accepts a connection or we miss the
    # deadline.
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection((host, port), timeout=1.0)
            s.settimeout(None)
            return s
        except (ConnectionRefusedError, OSError):
            time.sleep(0.05)
    raise RuntimeError(
        "renode_launcher: Renode TCP Monitor on %s:%d did not "
        "accept connection within deadline" % (host, port))


_monitor_nonce = [0]


def _drain_monitor_until_marker(sock, marker, timeout=10.0):
    # Drain until the unique per-call `marker` appears in the stream.
    # The marker is emitted by a sentinel `python` command appended
    # after the real command (see _send_monitor); it is constructed at
    # runtime from string fragments so the marker text does NOT appear
    # in the sentinel command's own echo - only in its evaluated output.
    # That makes it an unambiguous response delimiter, unlike matching
    # the `(<context>)` prompt: Renode echoes each command verbatim, and
    # an echoed command containing a parenthesised identifier (e.g.
    # `GetAllSymbolAddresses(n)`) can satisfy a `(\w+)`-style prompt
    # match when a recv() chunk happens to end on it, desyncing every
    # subsequent round-trip.
    deadline = time.monotonic() + timeout
    buf = b''
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(
                "renode_launcher: monitor read timed out after %fs; "
                "buffer=%r" % (timeout, buf[-200:]))
        r, _, _ = select.select([sock], [], [], min(remaining, 0.5))
        if not r:
            continue
        try:
            chunk = sock.recv(4096)
        except OSError:
            return buf
        if not chunk:
            return buf
        buf += chunk
        if marker in buf:
            return buf


def _send_monitor(sock, line, timeout=10.0):
    # Trailing \n is required; carriage return optional but harmless.
    # Append a sentinel command that prints a unique marker so the
    # response delimiter is robust against echoed-command false matches.
    _monitor_nonce[0] += 1
    token = 'RLDONE%dX' % _monitor_nonce[0]
    marker = token.encode('ascii')
    # Build the printed token from two fragments so the literal `token`
    # never appears in the sentinel's command echo - only its output.
    sentinel = ('python "import sys; sys.stdout.write(\'%s\'+\'%s\')"'
                % (token[:5], token[5:]))
    sock.sendall((line + '\n' + sentinel + '\n').encode('utf-8'))
    return _drain_monitor_until_marker(sock, marker, timeout=timeout)


# ---------------------------------------------------------------------
# Fixture control protocol -> Renode hook function call translation.
#
# Mirrors the command vocabulary that
# scripts/test_klippy.py:_push_fixture_to_control_socket emits and
# that test/emulator/simavr_bridge.c parses. Each function returns the
# Python expression to evaluate inside Renode's MonitorPythonEngine
# (which has the renode_hooks.py module's symbols in scope).

def _xlat_step_trigger(parts):
    # step_trigger <step_port> <step_pin> <count> <trig_port> <trig_pin> <val>
    # eg "step_trigger A 5 100 B 7 1" -> "renode_hooks.step_trigger(...)"
    if len(parts) != 7:
        return None
    return ("renode_hooks.step_trigger('%s', %d, %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3]),
               parts[4], int(parts[5]), int(parts[6])))


def _xlat_probe_step(parts):
    # probe_step <step_port> <step_pin> <reset_us> <force_per_step>
    #            <trig_port> <trig_pin> <val>
    if len(parts) != 8:
        return None
    return ("renode_hooks.probe_step('%s', %d, %d, %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3]), int(parts[4]),
               parts[5], int(parts[6]), int(parts[7])))


def _xlat_bltouch(parts):
    # bltouch <ctrl_port> <ctrl_pin> <sensor_port> <sensor_pin> <invert>
    if len(parts) != 6:
        return None
    return ("renode_hooks.bltouch('%s', %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), parts[3], int(parts[4]),
               int(parts[5])))


def _xlat_gpio(parts):
    # gpio <port> <pin> <val>
    if len(parts) != 4:
        return None
    return ("renode_hooks.gpio_set('%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3])))


def _xlat_sw_uart(parts):
    # The fixture runner emits two forms of `sw_uart`:
    #   simavr (4 args): sw_uart <port> <pin> <bit_time> <addr>
    #     - AVR firmware uses single-wire mode (rx_pin == tx_pin); the
    #       simavr bridge takes one pin and registers an IRQ hook that
    #       both decodes RX and drives TX from the same wire.
    #   renode (6 args): sw_uart <rx_port> <rx_pin> <tx_port> <tx_pin>
    #                            <bit_time> <addr>
    #     - ARM firmware (e.g. duet2-maestro on SAM4S) uses separate
    #       uart_pin/tx_pin. Renode's hook implementation
    #       (renode_hooks.sw_uart) drives RX bit-by-bit via CPU PC hooks
    #       on tmcuart_read_event etc., so it needs both pins explicitly.
    #
    # AVR runs through simavr, never through this launcher, so the 4-arg
    # form would only land here if someone misroutes a fixture. Reject
    # it cleanly rather than silently registering a half-broken hook.
    if len(parts) == 7:
        return ("renode_hooks.sw_uart('%s', %d, '%s', %d, %d, %d)"
                % (parts[1], int(parts[2]), parts[3], int(parts[4]),
                   int(parts[5]), int(parts[6])))
    return None


def _xlat_passthrough(parts):
    # Catch-all: forward as a function call with the same name. Lets
    # us add new fixture commands without re-touching the launcher as
    # long as renode_hooks.py grows the matching function.
    name = parts[0]
    args = ', '.join(repr(p) for p in parts[1:])
    return "renode_hooks.%s(%s)" % (name, args)


_XLAT = {
    'step_trigger': _xlat_step_trigger,
    'probe_step': _xlat_probe_step,
    'bltouch': _xlat_bltouch,
    'gpio': _xlat_gpio,
    'sw_uart': _xlat_sw_uart,
}


# Firmware symbols renode_hooks.sw_uart needs to attach CPU PC hooks
# to. The launcher resolves these via sysbus.GetAllSymbolAddresses
# right after LoadELF and pushes the dict to renode_hooks via
# apply_sw_uart_symbols(). Symbols absent from the firmware (e.g.
# configs without [tmc2208 ...] sections that don't link tmcuart.o)
# are silently skipped - the hook installer no-ops if any required
# symbol is missing.
_SW_UART_SYMBOLS = (
    'command_tmcuart_send',
    'tmcuart_send_finish_event',
    'tmcuart_read_sync_event',
    'tmcuart_read_event',
)


def _resolve_sched_status_addr(monitor_sock):
    # Resolve the firmware's SchedStatus struct address so
    # renode_hooks.peek_shutdown_reason() can read byte +11
    # (shutdown_reason) directly. Used to surface the firmware's
    # shutdown reason BEFORE klippy decodes it - klippy can't decode
    # the shutdown response message until the dict is loaded, so
    # firmware shutdowns during identify are otherwise opaque.
    cmd = (
        'python "import sys, renode_hooks; '
        'sb = monitor.Machine[\\"sysbus\\"]; '
        'a = list(sb.GetAllSymbolAddresses(\\"SchedStatus\\")); '
        'renode_hooks.apply_sched_status_addr(int(a[0]) if a else 0); '
        'sys.stdout.write(\\"SCHEDSTATUS=%s\\" % '
        '(int(a[0]) if a else 0))"')
    try:
        resp = _send_monitor(monitor_sock, cmd)
        m = re.search(rb'SCHEDSTATUS=(\d+)', resp)
        if m:
            sys.stderr.write("renode_launcher: SchedStatus @ 0x%X\n"
                             % int(m.group(1)))
        else:
            sys.stderr.write("renode_launcher: SchedStatus resolve "
                             "no match in resp=%r\n" % resp[-200:])
    except Exception as e:
        sys.stderr.write("renode_launcher: SchedStatus resolve err %s\n" % e)


def _resolve_sw_uart_symbols(monitor_sock):
    # Resolve the firmware tmcuart_* symbols and push the dict to
    # renode_hooks via apply_sw_uart_symbols. Configs without
    # [tmc2208 ...] sections don't link tmcuart.o; absent symbols
    # come back as empty lists from GetAllSymbolAddresses and are
    # omitted by the dict comprehension's `if` guard. The hook
    # installer no-ops if any required symbol is missing.
    sym_list = ', '.join("'%s'" % s for s in _SW_UART_SYMBOLS)
    cmd = (
        'python "import renode_hooks; '
        'sb = monitor.Machine[\\"sysbus\\"]; '
        'renode_hooks.apply_sw_uart_symbols(dict('
        '(n, int(list(sb.GetAllSymbolAddresses(n))[0])) '
        'for n in [%s] if list(sb.GetAllSymbolAddresses(n))))"'
        % sym_list)
    try:
        _send_monitor(monitor_sock, cmd)
    except Exception as e:
        sys.stderr.write("renode_launcher: sw_uart symbol resolve "
                         "send err %s\n" % e)


def _resolve_spi_tmc_symbol(monitor_sock):
    # Resolve the firmware `spidev_transfer` symbol and push it to
    # renode_hooks.apply_spi_tmc_symbol so the TMC SPI chain hook can
    # attach. Configs without an SPI device (or built without spicmds.o)
    # come back empty and the hook stays uninstalled. Harmless to call
    # for every chip; renode_hooks.spi_tmc() (from the fixture) is what
    # actually enables the responder.
    cmd = (
        'python "import renode_hooks; '
        'sb = monitor.Machine[\\"sysbus\\"]; '
        'a = list(sb.GetAllSymbolAddresses(\\"spidev_transfer\\")); '
        'renode_hooks.apply_spi_tmc_symbol(int(a[0])) if a else None"')
    try:
        _send_monitor(monitor_sock, cmd)
    except Exception as e:
        sys.stderr.write("renode_launcher: spi_tmc symbol resolve "
                         "send err %s\n" % e)


# Default I2C addresses to register the empty fixture's
# i2c_default.register_responses against. Covers the LDC1612 default
# (0x2a) and its alternate (0x29) - the only I2C device klippy probes
# at startup with a fixed ID (and so the only one that hard-fails an
# entire printer config when the response is wrong / missing). Other
# I2C devices in printer configs get probed with their own register
# vocabularies; expand this list as concrete tests surface failures.
_DEFAULT_I2C_ADDRS = (0x2a, 0x29)


# --------------------------------------------------------------------
# Deterministic tick-mode driver.
#
# When --tick-socket is set we DO NOT issue Renode's `start` Monitor
# command (which kicks off wall-clock-paced execution). Instead the
# CPU only advances inside Monitor `python "...RunFor(...)"` calls
# the tick thread issues in response to klippy's `advance T\n` lines.
# All RunFor calls go through a single lock so the control-socket
# loop's barrier / fixture-push paths can also use the helper without
# racing.
#
# Probed and confirmed against Renode 1.16.1:
#   monitor.Machine.LocalTimeSource.RunFor(TimeInterval.FromMicroseconds(N))
#   monitor.Machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds
# RunFor is synchronous (the `python "..."` Monitor response only
# arrives after RunFor returns) so the launcher can treat the
# Monitor reply as the "advance complete" signal.

def _setup_tick_pty(slave_link):
    # Open a fresh pty pair; keep the master fd in this process for
    # synchronous byte shuttle between klippy and the firmware UART.
    # Klippy connects to the slave end via slave_link; we close the
    # slave fd here since klippy reopens it. Master is set raw so 8-bit
    # binary klipper protocol bytes pass through unchanged AND
    # non-blocking so _drain_pty_klippy_writes can poll without hanging
    # the tick loop.
    import pty as _pty
    import termios
    import fcntl
    master_fd, slave_fd = _pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    try:
        attrs = termios.tcgetattr(master_fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag = 0
        oflag = 0
        lflag = 0
        cflag |= termios.CS8
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(master_fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag,
                           ispeed, ospeed, cc])
    except (termios.error, OSError) as e:
        sys.stderr.write("renode_launcher: tick pty raw mode err %s\n"
                         % e)
    try:
        flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
        fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    except OSError as e:
        sys.stderr.write("renode_launcher: tick pty O_NONBLOCK err %s\n"
                         % e)
    try:
        os.unlink(slave_link)
    except OSError:
        pass
    os.symlink(slave_path, slave_link)
    return master_fd


def _make_tick_state(sim_time_file=None):
    import threading
    state = {
        'lock': threading.Lock(),
        # Cumulative microseconds we have requested via RunFor. Used
        # to compute the per-call delta for klippy's monotonic
        # `advance T` targets and to populate the `done T_actual`
        # reply.
        'virt_us': 0,
        'tx_bytes_total': 0,
        'rx_bytes_total': 0,
        # Optional mmap'd 8-byte double matching simavr_bridge.c's
        # --sim-time-file format. Klippy reads this via
        # KLIPPY_SIM_TIME_FILE so its monotonic clock advances in
        # lockstep with our RunFor calls. Without it klippy would
        # use wall time and immediately race ahead of virtual time.
        'sim_time_mmap': None,
    }
    if sim_time_file:
        try:
            import mmap as _mmap
            fd = os.open(sim_time_file,
                         os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o644)
            try:
                os.write(fd, b'\x00' * 8)
                state['sim_time_mmap'] = _mmap.mmap(
                    fd, 8, _mmap.MAP_SHARED,
                    _mmap.PROT_READ | _mmap.PROT_WRITE)
            finally:
                os.close(fd)
        except (OSError, ValueError) as e:
            sys.stderr.write(
                "renode_launcher: sim-time-file %s init err %s\n"
                % (sim_time_file, e))
    return state


def _tick_publish_sim_time(tick_state):
    m = tick_state['sim_time_mmap']
    if m is None:
        return
    try:
        import struct
        struct.pack_into('d', m, 0, tick_state['virt_us'] / 1e6)
    except Exception as e:
        sys.stderr.write("renode_launcher: sim-time write err %s\n"
                         % e)


_HEX_RE = re.compile(rb'\n\r([0-9a-fA-F]*)\([\w-]+\)\s')


def _drain_pty_klippy_writes(master_fd):
    # Non-blocking drain of bytes klippy wrote to the slave. Returns
    # bytes object (possibly empty). BlockingIOError is the normal
    # "nothing to read" exit; OSError (typically EIO when the slave
    # has been closed between klippy attempts) is also non-fatal -
    # the next slave open will reopen the pty.
    if master_fd is None:
        return b''
    chunks = []
    while True:
        try:
            d = os.read(master_fd, 4096)
        except (BlockingIOError, OSError):
            break
        if not d:
            break
        chunks.append(d)
    return b''.join(chunks)


def _tick_run_for(monitor_sock, tick_state, delta_us, master_fd=None):
    # Advance virtual time by `delta_us` microseconds via the
    # `emulation RunFor "<seconds>"` Monitor command (= EmulationManager
    # .Instance.CurrentEmulation.RunFor, the global master time source).
    # Routing through Machine.LocalTimeSource.RunFor instead is also
    # offered by the API but is wall-clock-paced (1100x slower than
    # real-time even on idle firmware) and not viable for CI. The
    # Monitor variant runs at near 1:1 virtual:wall on this host,
    # actually faster than real-time when firmware is idle.
    #
    # In tick mode (master_fd != None) we additionally shuttle UART
    # bytes synchronously: drain klippy writes BEFORE RunFor so the
    # firmware sees them this quantum, drain firmware-emitted bytes
    # AFTER RunFor and write them to the pty so klippy's serialqueue
    # poll() sees them on its next wake-up.
    if delta_us <= 0 and master_fd is None:
        return
    tx_bytes = b''
    with tick_state['lock']:
        if master_fd is not None:
            tx_bytes = _drain_pty_klippy_writes(master_fd)
            if tx_bytes:
                tick_state['tx_bytes_total'] += len(tx_bytes)
        if delta_us > 0 and master_fd is not None:
            # 3-step shuttle:
            #   1. serial_write_hex(klippy_tx_bytes)   - inject to UART
            #   2. emulation RunFor "<sec>"            - advance virt time
            #   3. serial_drain_hex()                  - capture firmware TX
            # The RunFor must go through the Monitor command form
            # (`emulation RunFor "X"`), NOT through the IronPython
            # call EmulationManager.Instance.CurrentEmulation.RunFor:
            # the latter is wall-clock-paced (1000x slower than
            # real-time even on idle firmware), the Monitor command
            # is near 1:1.
            if tx_bytes:
                tx_hex = ''.join('%02x' % b for b in tx_bytes)
                cmd = ('python "renode_hooks.serial_write_hex(\'%s\')"'
                       % tx_hex)
                try:
                    _send_monitor(monitor_sock, cmd, timeout=30.0)
                except Exception as e:
                    sys.stderr.write(
                        "renode_launcher: tick TX err %s\n" % e)
                    return
            cmd = 'emulation RunFor "%.6f"' % (delta_us / 1e6)
            try:
                _send_monitor(monitor_sock, cmd, timeout=120.0)
            except Exception as e:
                sys.stderr.write(
                    "renode_launcher: tick RunFor err %s\n" % e)
                return
            tick_state['virt_us'] += int(delta_us)
            _tick_publish_sim_time(tick_state)
            cmd = ('python "import sys; '
                   'sys.stdout.write(renode_hooks.serial_drain_hex() '
                   'or \\"\\")"')
            try:
                resp = _send_monitor(monitor_sock, cmd, timeout=30.0)
            except Exception as e:
                sys.stderr.write(
                    "renode_launcher: tick RX drain err %s\n" % e)
                return
            m = _HEX_RE.search(resp)
            if m:
                hex_b = m.group(1)
                if hex_b:
                    try:
                        rx_bytes = bytes.fromhex(
                            hex_b.decode('ascii'))
                    except ValueError as e:
                        sys.stderr.write(
                            "renode_launcher: tick RX bad hex %r: %s\n"
                            % (hex_b[:60], e))
                        return
                    tick_state['rx_bytes_total'] += len(rx_bytes)
                    try:
                        os.write(master_fd, rx_bytes)
                    except OSError:
                        pass
            return
        if delta_us > 0:
            # Adaptive RunFor with early-out:
            #
            # If klippy just sent TX bytes (it's expecting a response),
            # break the advance into small chunks and bail out as soon
            # as the firmware emits bytes. This keeps each request /
            # response round trip to ~1 ms of sim time so klippy's
            # 5 s connect window can fit hundreds of round trips
            # (identify of a 9.7 KB dictionary at 40 bytes/chunk
            # needs ~250 round trips).
            #
            # One RunFor per advance request; drain firmware-emitted
            # bytes afterwards.
            cmd = 'emulation RunFor "%.6f"' % (delta_us / 1e6)
            try:
                _send_monitor(monitor_sock, cmd, timeout=120.0)
            except Exception as e:
                sys.stderr.write(
                    "renode_launcher: tick RunFor err %s\n" % e)
                return
            tick_state['virt_us'] += delta_us
            _tick_publish_sim_time(tick_state)
            if master_fd is not None:
                _drain_firmware_to_pty(
                    monitor_sock, tick_state, master_fd)
        elif master_fd is not None:
            # delta_us == 0: still drain any pending firmware emission
            # in case bytes arrived during a previous RunFor that we
            # bailed out of early.
            _drain_firmware_to_pty(monitor_sock, tick_state, master_fd)


def _drain_firmware_to_pty(monitor_sock, tick_state, master_fd):
    cmd = ('python "import sys; '
           'sys.stdout.write(renode_hooks.serial_drain_hex() '
           'or \\"\\")"')
    try:
        resp = _send_monitor(monitor_sock, cmd, timeout=30.0)
    except Exception as e:
        sys.stderr.write(
            "renode_launcher: tick RX drain err %s\n" % e)
        return b''
    m = _HEX_RE.search(resp)
    if not m:
        return b''
    hex_b = m.group(1)
    if not hex_b:
        return b''
    try:
        rx_bytes = bytes.fromhex(hex_b.decode('ascii'))
    except ValueError as e:
        sys.stderr.write(
            "renode_launcher: tick RX bad hex %r: %s\n"
            % (hex_b[:60], e))
        return b''
    tick_state['rx_bytes_total'] += len(rx_bytes)
    sys.stderr.write(
        "renode_launcher: tick RX %d bytes (head=%r)\n"
        % (len(rx_bytes), rx_bytes[:32]))
    try:
        os.write(master_fd, rx_bytes)
    except OSError as e:
        sys.stderr.write(
            "renode_launcher: tick RX pty write err %s\n" % e)
    return rx_bytes


def _bind_control_socket(sock_path):
    # Bind+listen the AF_UNIX control socket synchronously at launcher
    # startup. The runner's _push_fixture_to_control_socket connect
    # retry is short (~2s); Renode boot is ~30s. Without early bind
    # the runner would silently miss publishing the fixture and the
    # sw_uart / step_trigger / gpio hooks would never register. The
    # ctl_thread launched after Renode boots accepts on this pre-bound
    # srv and reads any commands the runner queued during boot.
    try:
        os.unlink(sock_path)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        srv.bind(sock_path)
    except OSError as e:
        sys.stderr.write("renode_launcher: ctl bind(%s) err %s\n"
                         % (sock_path, e))
        return None
    try:
        os.chmod(sock_path, 0o666)
    except OSError:
        pass
    srv.listen(1)
    srv.settimeout(0.5)
    return srv


def _bind_tick_socket(tick_path):
    # Bind+listen the AF_UNIX tick socket synchronously at launcher
    # startup. Klippy's reactor.py only retries _tick_connect for ~5 s
    # of wall clock, but Renode boot takes ~30 s before the tick
    # thread would otherwise come up - klippy would fail with
    # ECONNREFUSED. Binding here returns a listening socket so klippy
    # can connect immediately; the actual accept() happens later in
    # _tick_socket_loop.
    try:
        os.unlink(tick_path)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        srv.bind(tick_path)
    except OSError as e:
        sys.stderr.write("renode_launcher: tick bind(%s) err %s\n"
                         % (tick_path, e))
        return None
    try:
        os.chmod(tick_path, 0o666)
    except OSError:
        pass
    srv.listen(1)
    srv.settimeout(0.5)
    return srv


def _tick_socket_loop(srv, monitor_sock, tick_state, stop_evt,
                      master_fd=None):
    # AF_UNIX accept loop on the pre-bound `srv`. Klippy connects,
    # sends `advance T\n` (T in seconds, monotonic), launcher RunFor's
    # the delta, replies `done T_actual\n`. Same wire protocol as
    # simavr_bridge.c so klippy's reactor.py is unchanged.
    #
    # Klippy's main loop spawns multiple Printer instances if connect
    # fails the first time (klippy.py:main while 1), each of which
    # opens its own tick socket. Outer loop re-accepts after each
    # client EOF so subsequent klippy attempts can also drive the
    # tick.
    while not stop_evt[0]:
        cli = None
        while not stop_evt[0]:
            try:
                cli, _ = srv.accept()
                break
            except socket.timeout:
                continue
            except OSError:
                return
        if cli is None:
            return
        _tick_serve_client(cli, monitor_sock, tick_state, stop_evt,
                           master_fd=master_fd)
    try:
        srv.close()
    except OSError:
        pass


def _tick_serve_client(cli, monitor_sock, tick_state, stop_evt,
                       master_fd=None):
    cli.settimeout(0.5)
    buf = b''
    while not stop_evt[0]:
        try:
            chunk = cli.recv(256)
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            # Don't set stop_evt - klippy's main loop spawns a new
            # Printer on connect failure and reconnects to the tick
            # socket; we want to accept again, not unlink it.
            break
        buf += chunk
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode('ascii', 'replace').strip()
            parts = text.split()
            if len(parts) != 2 or parts[0] != 'advance':
                # Unknown line: ack with current virtual time so
                # klippy doesn't wedge. Mirrors simavr_bridge.c
                # tick-mode bad-line handling.
                _tick_reply_done(cli, tick_state)
                continue
            try:
                target_s = float(parts[1])
            except ValueError:
                _tick_reply_done(cli, tick_state)
                continue
            import math
            target_us = int(math.ceil(target_s * 1e6))
            delta_us = target_us - tick_state['virt_us']
            # Klippy can request advance with target == current virt
            # (delta_us=0) when its reactor's next-timer waketime ==
            # eventtime in float - the for-loop in _check_timers ought
            # to fire that timer, but float precision in the
            # monotonic/timer arithmetic plus async-callback chains
            # keep pushing _next_timer to "right now" so the reactor
            # asks us to "advance to where we already are" without
            # ever firing the pending timer. Force a 1 us advance so
            # the firmware has cycles to keep firing its own
            # interrupts and klippy's monotonic ticks past the
            # equality.
            if delta_us == 0:
                # 1 ms nudge: enough virtual time for any pending
                # firmware interrupt to fire (1 ms >> bit_time at
                # 250 kbaud), but small enough that 5 sim-seconds of
                # connect timeout fit in well under 5000 nudge
                # round-trips. Shorter nudges (1 us) bottlenecked on
                # Monitor TCP overhead.
                delta_us = 1000
            _tick_run_for(monitor_sock, tick_state, delta_us,
                          master_fd=master_fd)
            _tick_reply_done(cli, tick_state)
    try:
        cli.close()
    except OSError:
        pass


def _tick_reply_done(cli, tick_state):
    actual_s = tick_state['virt_us'] / 1e6
    msg = ('done %.9f\n' % actual_s).encode('ascii')
    try:
        cli.sendall(msg)
    except OSError:
        pass


def _apply_fixture_to_renode(fixture_path, monitor_sock):
    # Read the fixture file and emit the renode-hook calls that
    # simavr_bridge.c handles internally. step_trigger / bltouch /
    # probe_step come over the control socket from
    # _push_fixture_to_control_socket - we don't duplicate those
    # here. analog_in, analog_in_default, i2c_default are NOT pushed
    # over the control socket by the test runner today, so the
    # launcher is the right place to consume them for the renode
    # backend.
    if not fixture_path or not os.path.isfile(fixture_path):
        return
    try:
        with open(fixture_path) as f:
            fx = json.load(f)
    except (OSError, ValueError) as e:
        sys.stderr.write(
            "renode_launcher: ignoring unreadable fixture %r: %s\n"
            % (fixture_path, e))
        return

    def _send(py_call):
        _send_monitor(monitor_sock,
                      'python "%s"' % py_call.replace('"', r'\"'))

    # All-channels ADC default.
    default_block = fx.get('analog_in_default')
    if isinstance(default_block, dict):
        dv = default_block.get('default_value')
        if dv is not None:
            _send('renode_hooks.adc_default(%d)' % int(dv))

    # Per-channel overrides. The keys in `analog_in` are labels
    # (typically `_hot_extruder_N`) that the AVR fixture pusher maps
    # to the FIRST few configured analog_in OIDs. We don't have the
    # OID assignment here, so we default to dict iteration order as
    # the channel index. Tests that need to pin a specific override
    # to a specific MCU ADC channel (e.g. adc_scaled's vref_pin /
    # vssa_pin where channel mismatch causes division by zero in
    # klippy's adc_scaled callback) supply an explicit `channel`
    # field in the spec; we honour that when present.
    analog_in = fx.get('analog_in')
    if isinstance(analog_in, dict):
        for ch_idx, (_label, spec) in enumerate(analog_in.items()):
            if not isinstance(spec, dict):
                continue
            v = spec.get('default_value')
            if v is None:
                continue
            target_ch = spec.get('channel', ch_idx)
            _send('renode_hooks.adc_set(%d, %d)'
                  % (int(target_ch), int(v)))

    # I2C ID-probe responses.
    i2c_default = fx.get('i2c_default')
    if isinstance(i2c_default, dict):
        reg_resp = i2c_default.get('register_responses')
        if isinstance(reg_resp, dict) and reg_resp:
            # Normalise the dict to a JSON-safe form for embedding in
            # a python string (bytes lists -> int lists, hex keys
            # passed through as strings - the hook re-parses with
            # int(k, 0)).
            normalised = {str(k): list(v) for k, v in reg_resp.items()}
            payload_json = json.dumps(normalised)
            for addr in _DEFAULT_I2C_ADDRS:
                _send('renode_hooks.i2c_register_response(1, %d, %s)'
                      % (addr, payload_json))


def _translate_fixture_command(line):
    parts = line.strip().split()
    if not parts:
        return None
    fn = _XLAT.get(parts[0], _xlat_passthrough)
    return fn(parts)


# ---------------------------------------------------------------------

def _control_socket_loop(srv, monitor_sock, stop_evt,
                         tick_state=None, master_fd=None):
    # Accept connections from _push_fixture_to_control_socket. Each
    # newline-terminated command is either a hook setter (translated
    # to a Renode python call) or one of the bridge-protocol verbs
    # the simavr bridge implements internally:
    #
    #   start            - kick off CPU execution. simavr's bridge
    #                      starts running as soon as it spawns; Renode
    #                      starts paused, so we have to issue `start`
    #                      to the monitor explicitly. The runner does
    #                      NOT send `start` today (the simavr bridge
    #                      does not expose a `start` verb), so we also
    #                      auto-trigger on the first `barrier`.
    #                      In tick mode `start` is a no-op: the CPU
    #                      only runs inside klippy-driven RunFor calls.
    #   barrier <usec>   - wait simulated time and ACK. simavr blocks
    #                      until its cycle counter advances <usec>;
    #                      Renode in real-time mode does a wall-clock
    #                      sleep of usec microseconds (firmware activity
    #                      scales with wall time after `start`). In tick
    #                      mode, we RunFor <usec> of virtual time so the
    #                      firmware has cycles to apply queued GPIO
    #                      drives before klippy's first advance arrives.
    #
    # Hook commands (analog_in_default, step_trigger, ...) translate
    # to `python "<call>"` against renode_hooks.py.
    #
    # `srv` is bound+listening before this thread starts (see
    # _bind_control_socket called from main()); we just accept here.
    started = [False]

    def _ensure_started():
        if started[0]:
            return
        if tick_state is not None:
            # Tick-driven: the CPU only runs inside RunFor calls
            # initiated by klippy's tick socket. Do NOT issue `start`
            # to the monitor or the wall-clock executor will steal
            # cycles concurrently with our deterministic advances.
            started[0] = True
            return
        _send_monitor(monitor_sock, 'start')
        started[0] = True

    while not stop_evt[0]:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        try:
            buf = b''
            while not stop_evt[0]:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, _, buf = buf.partition(b'\n')
                    text = line.decode('utf-8', 'replace').strip()
                    if not text:
                        continue
                    if text == 'start':
                        _ensure_started()
                        try:
                            conn.sendall(b'OK\n')
                        except OSError:
                            pass
                        continue
                    if (text == 'barrier'
                            or text.startswith('barrier ')):
                        _ensure_started()
                        usec = 1000
                        parts = text.split()
                        if len(parts) > 1:
                            try:
                                usec = int(parts[1])
                            except ValueError:
                                pass
                        # Cap the wait so a malformed fixture cannot
                        # hang the launcher; simavr bridge's barrier
                        # is bounded by the runner's 5 s ack timeout
                        # anyway.
                        usec = min(usec, 5000000)
                        if tick_state is not None:
                            # Tick mode: advance virtual time by the
                            # requested microseconds so any GPIO drives
                            # / IRQ schedules pushed in fixture setup
                            # take effect before klippy's first advance
                            # arrives. Serialised with the monitor lock
                            # so this doesn't race a klippy-driven
                            # RunFor.
                            _tick_run_for(monitor_sock, tick_state,
                                          usec, master_fd=master_fd)
                        else:
                            time.sleep(usec / 1e6)
                        try:
                            conn.sendall(b'OK\n')
                        except OSError:
                            pass
                        continue
                    py_call = _translate_fixture_command(text)
                    if py_call is None:
                        sys.stderr.write(
                            "renode_launcher: unrecognized fixture "
                            "command %r\n" % text)
                        continue
                    full_cmd = ('python "%s"'
                                % py_call.replace('"', r'\"'))
                    try:
                        _send_monitor(monitor_sock, full_cmd)
                    except Exception as e:
                        sys.stderr.write(
                            "renode_launcher: ctl send err %s\n" % e)
                    try:
                        conn.sendall(b'OK\n')
                    except OSError:
                        pass
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
    if not started[0]:
        # No `start`/`barrier` was issued (test runner gave up early
        # or never connected the control socket - happens in
        # single-shot diagnostic runs). Start the CPU so the firmware
        # at least boots and we can observe the failure mode.
        try:
            _send_monitor(monitor_sock, 'start')
        except (OSError, RuntimeError):
            pass
    try:
        srv.close()
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', required=True)
    ap.add_argument('--slave-link', required=True,
                    help='symlink published to the host pty Renode opens '
                         'for USART1; klippy connects to this path.')
    ap.add_argument('--control-socket', required=True,
                    help='unix socket that receives newline-terminated '
                         'fixture commands (same protocol as '
                         'simavr_bridge.c).')
    ap.add_argument('--duration', type=float, default=120.0,
                    help='hard wall-clock cap on this run (seconds).')
    # Accepted for arg-shape parity with simavr_bridge; ignored on
    # first cut (Renode runs real-time, klippy uses live clock
    # estimation over USART when KLIPPY_SIM_TIME_FILE is unset).
    ap.add_argument('--sim-time-file', default=None)
    ap.add_argument('--tick-socket', default=None)
    # The fixture file the test runner is about to push commands
    # from. simavr_bridge.c handles analog_in / i2c_default /
    # spi_response by reading the fixture itself (out-of-band of the
    # control socket); we mirror that here so the renode hooks for
    # ADC defaults and LDC1612 ID probes get applied before klippy
    # starts running. Optional - if omitted, only the control-socket
    # commands take effect (sufficient for tests with empty or
    # GPIO-only fixtures).
    ap.add_argument('--fixture-file', default=None)
    args = ap.parse_args()

    chip = _chip_for_elf(args.elf)

    workdir = tempfile.mkdtemp(prefix='renode_launcher_')
    pty_path = os.path.join(workdir, 'uart.pty')
    resc_path = os.path.join(workdir, 'launch.resc')
    log_path = os.path.join(workdir, 'renode.log')

    monitor_port = _allocate_tcp_port()
    tick_mode = bool(args.tick_socket)
    resc = _render_resc(chip, os.path.abspath(args.elf), pty_path,
                        monitor_port, log_path, tick_mode=tick_mode)
    with open(resc_path, 'w') as f:
        f.write(resc)

    renode = shutil.which('renode')
    if renode is None:
        sys.stderr.write("renode_launcher: `renode` not on PATH\n")
        return 2

    # --plain disables ANSI colors (cleaner log capture); --disable-xwt
    # avoids any GUI initialization (works headless on Linux Docker
    # without X). -P binds the Monitor to a TCP port so we can drive
    # it from this process; -e includes our generated .resc.
    cmd = [
        renode, '--plain', '--disable-xwt',
        '-P', str(monitor_port),
        '-e', 'include @' + resc_path,
    ]

    proc = subprocess.Popen(cmd, stdout=sys.stderr, stderr=sys.stderr)

    stop_evt = [False]

    def _shutdown(*_a):
        stop_evt[0] = True

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    # Renode startup can take 30+ seconds on a cold cache - the
    # ATSAME70Q21.svd download alone is ~6MB and depending on network
    # latency (macOS Docker in particular routes through vpnkit which
    # caps throughput on first contact), the full mach create + LoadELF +
    # CreateUartPtyTerminal sequence runs 25-50s before the pty link
    # appears. The runner's --duration is sized for klippy work time
    # (EMULATOR_KLIPPY_DEADLINE = 20s + 5s slack) and is too tight to
    # also cover Renode startup. Use a startup deadline decoupled from
    # --duration; once Renode is up and the pty is published, restart
    # the duration clock so klippy gets its full work window regardless
    # of how long startup took.
    startup_deadline = time.monotonic() + 90.0
    deadline = None

    serial_master_fd = None
    early_tick_state = None
    early_tick_srv = None
    # Pre-bind the control socket at launcher startup so the runner's
    # _push_fixture_to_control_socket call (which fires while Renode is
    # still booting) succeeds immediately. The ctl_thread accepts on
    # this srv after Renode is up and processes queued commands.
    early_ctl_srv = _bind_control_socket(args.control_socket)
    if early_ctl_srv is None:
        sys.stderr.write("renode_launcher: control socket bind failed\n")
        return 4
    if tick_mode:
        # Pre-create the tick_state (lock, sim_time mmap, counters)
        # IMMEDIATELY at launcher startup. Klippy spawns shortly after
        # the slave_link symlink appears (which we publish below in
        # _setup_tick_pty), well before Renode finishes booting and
        # we get to the post-prompt setup. If sim_time_file doesn't
        # exist when klippy first calls get_monotonic(), klippy falls
        # back to wall-clock and immediately requests a huge advance,
        # blowing the test out of the water. By creating the file
        # here with an initial 0.0 we guarantee klippy sees a sane
        # starting time.
        early_tick_state = _make_tick_state(
            sim_time_file=args.sim_time_file)
        # Same problem for the tick socket itself: klippy's
        # _tick_connect retries only ~5 s of wall clock, so it would
        # ECONNREFUSED before Renode finishes booting. Bind+listen
        # here so the socket exists from launcher start; accept()
        # happens later in _tick_socket_loop.
        early_tick_srv = _bind_tick_socket(args.tick_socket)
    try:
        if tick_mode:
            # Launcher-managed pty: we open the master end and hand
            # the slave to klippy via the symlink. Renode never sees
            # the pty - we shuttle bytes through renode_hooks instead
            # of CreateUartPtyTerminal.
            serial_master_fd = _setup_tick_pty(args.slave_link)
        else:
            # Wait for the pty Renode creates, then publish the
            # symlink so klippy's _wait_for_slave_link sees the slave
            # path.
            if not _wait_for_path(pty_path, startup_deadline):
                sys.stderr.write(
                    "renode_launcher: Renode did not create UART pty"
                    " at %s within 90s; aborting\n" % pty_path)
                return 3
            try:
                os.unlink(args.slave_link)
            except OSError:
                pass
            os.symlink(pty_path, args.slave_link)

        monitor_sock = _connect_monitor(
            '127.0.0.1', monitor_port, startup_deadline)
        # Consume Renode's startup banner and confirm the monitor is
        # live by round-tripping a no-op through the sentinel-delimited
        # _send_monitor (an empty command line just re-prints the
        # prompt; the appended sentinel proves the monitor executes).
        _send_monitor(monitor_sock, '', timeout=30.0)

        if tick_mode:
            # Subscribe to UART CharReceived in renode_hooks so
            # firmware-emitted bytes accumulate in a Python buffer we
            # can drain after each RunFor.
            usart = _host_link_peripheral(chip)
            cmd = (
                'python "import renode_hooks; '
                'renode_hooks.serial_init(\\"sysbus.%s\\")"' % usart)
            try:
                _send_monitor(monitor_sock, cmd)
            except Exception as e:
                sys.stderr.write(
                    "renode_launcher: serial_init err %s\n" % e)

        # Resolve firmware symbols renode_hooks.sw_uart attaches CPU
        # PC hooks to (tmcuart_read_event etc). Done here so the
        # symbols are in place before the first sw_uart fixture
        # command arrives over the control socket. Configs that don't
        # link tmcuart.o report no addresses and the hook installer
        # no-ops cleanly.
        _resolve_sw_uart_symbols(monitor_sock)
        _resolve_spi_tmc_symbol(monitor_sock)
        _resolve_sched_status_addr(monitor_sock)

        # Apply the fixture-resident hooks (ADC defaults, I2C ID
        # responses) BEFORE the control loop accepts the runner's
        # `start` command - those hook calls have to be in place
        # before the CPU starts executing or klippy may probe a
        # peripheral and shutdown before the response arrives.
        _apply_fixture_to_renode(args.fixture_file, monitor_sock)

        # Tick-mode lockstep. With --tick-socket the launcher runs the
        # CPU exclusively via klippy-driven RunFor calls (deterministic
        # virtual time); without it we fall back to Renode's wall-clock
        # paced execution. The control loop and tick loop share the
        # monitor TCP, so they coordinate via tick_state['lock'] to
        # avoid interleaved Monitor responses.
        # Reuse the early_tick_state we created at launcher startup so
        # the sim_time file is in place before klippy's first call to
        # get_monotonic(). _make_tick_state again here would re-truncate
        # the file and lose any progress; just point at the existing
        # state.
        tick_state = early_tick_state

        import threading
        ctl_thread = threading.Thread(
            target=_control_socket_loop,
            args=(early_ctl_srv, monitor_sock, stop_evt),
            kwargs={'tick_state': tick_state,
                    'master_fd': serial_master_fd},
            daemon=True)
        ctl_thread.start()

        tick_thread = None
        if tick_state is not None and early_tick_srv is not None:
            tick_thread = threading.Thread(
                target=_tick_socket_loop,
                args=(early_tick_srv, monitor_sock, tick_state,
                      stop_evt),
                kwargs={'master_fd': serial_master_fd},
                daemon=True)
            tick_thread.start()

        # Renode is up and the pty / monitor are wired - start the
        # --duration clock now so klippy gets its full work window.
        deadline = time.monotonic() + args.duration

        # Optional firmware-shutdown surfacing (debug aid, opt-in via
        # RENODE_PEEK_SHUTDOWN). In real-time (non-tick) mode nothing
        # else polls the firmware's shutdown state, so a firmware
        # shutdown during the klippy handshake is otherwise invisible in
        # the launcher trace (klippy just sees is_shutdown=1 at
        # get_config and can't decode the reason if it fired before the
        # dict loaded). When enabled, peek SchedStatus.shutdown_reason
        # periodically and log the static_string_id the first time it
        # goes non-zero (map via the dict's enumerations.static_string_id;
        # this is how the SAME70 USB-CDC "Rescheduled timer in the past"
        # / id 56 blocker was diagnosed). It is OFF by default because
        # each peek issues a Monitor `python` round-trip, and under
        # real-time execution that can perturb the CPU / pty timing
        # enough to glitch klippy's clocksync (negative freq estimate).
        # Tick mode already peeks inside serial_drain_hex.
        peek_shutdown = (tick_state is None
                         and os.environ.get('RENODE_PEEK_SHUTDOWN'))
        peek_interval = 0.25
        next_peek = time.monotonic() + peek_interval
        last_reason_logged = [0]
        while not stop_evt[0]:
            if proc.poll() is not None:
                break
            if time.monotonic() >= deadline:
                break
            if peek_shutdown and time.monotonic() >= next_peek:
                next_peek = time.monotonic() + peek_interval
                try:
                    # Print the reason into the Monitor response so the
                    # launcher (whose stderr IS captured into the test's
                    # emu log) can surface it - renode_hooks._log writes
                    # to the embedded-IronPython sys.stderr, which goes
                    # to the discarded Monitor response, not the emu log.
                    resp = _send_monitor(
                        monitor_sock,
                        'python "import sys; '
                        'sys.stdout.write(\\"PEEKREASON=%s\\" % '
                        'renode_hooks.peek_shutdown_reason())"',
                        timeout=5.0)
                    m = re.search(rb'PEEKREASON=(-?\d+)', resp)
                    if m:
                        reason = int(m.group(1))
                        if os.environ.get('RENODE_PEEK_DEBUG'):
                            sys.stderr.write(
                                "renode_launcher: PEEK reason=%d t=%.2f\n"
                                % (reason, time.monotonic()))
                        if reason and reason != last_reason_logged[0]:
                            last_reason_logged[0] = reason
                            sys.stderr.write(
                                "renode_launcher: FIRMWARE SHUTDOWN "
                                "reason=%d (static_string_id; map via "
                                "dict enumerations.static_string_id)\n"
                                % reason)
                except Exception:
                    pass
            time.sleep(0.1)
    finally:
        stop_evt[0] = True
        try:
            proc.terminate()
        except OSError:
            pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                proc.kill()
            except OSError:
                pass
        cleanup_paths = [args.slave_link, args.control_socket]
        if args.tick_socket:
            cleanup_paths.append(args.tick_socket)
        for p in cleanup_paths:
            try:
                os.unlink(p)
            except OSError:
                pass
        if serial_master_fd is not None:
            try:
                os.close(serial_master_fd)
            except OSError:
                pass
        try:
            shutil.rmtree(workdir, ignore_errors=True)
        except OSError:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
