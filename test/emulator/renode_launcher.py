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
    'stm32g0b1': '@platforms/cpus/stm32g0.repl',
    'stm32h723': _local('stm32h723.repl'),
    'stm32h743': '@platforms/cpus/stm32h743.repl',
    'sam3x8c': _local('sam3x8e.repl'),
    'sam3x8e': _local('sam3x8e.repl'),
    'sam4s8c': '@platforms/cpus/sam4s8b.repl',
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
}

# Per-chip extra Python.PythonPeripheral stubs to inject after the
# platform load. Each entry is a list of (name, base, size, stub_path)
# tuples. The samd51p20 entries cover the chip's clock controllers
# (OSCCTRL / GCLK / OSC32KCTRL) and the simple store/return blocks
# (MCLK / CMCC) that klipper firmware touches during SystemInit() and
# enable_pclock() - none of which Renode upstream models. Without
# these the firmware spins forever in samd51_clock.c.
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
    'lpc176x': [
        ('lpc_sc', 0x400FC000, 0x200, _LPC_SC_STUB_PY),
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


def _render_resc(chip, elf_path, pty_path, monitor_port, log_path):
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
    afec_block = ''
    for idx, base in enumerate(afec_bases):
        afec_block += (
            'machine LoadPlatformDescriptionFromString '
            '"afec{idx}: Python.PythonPeripheral @ sysbus 0x{base:08X} '
            '{{ size: 0x200; initable: true; '
            'filename: \\"{stub}\\" }}"\n'
        ).format(idx=idx, base=base, stub=_AFEC_STUB_PY)
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
    return (
        'using sysbus\n'
        'mach create "klipper-{chip}"\n'
        '{cs_include_block}'
        'machine LoadPlatformDescription {platform}\n'
        '{rcc_block}'
        '{afec_block}'
        '{extra_block}'
        'sysbus LoadELF @{elf}\n'
        'logFile @{log}\n'
        'logLevel 1\n'
        'emulation CreateUartPtyTerminal "uartTerm" "{pty}"\n'
        'connector Connect sysbus.{usart} uartTerm\n'
        'i @{hooks}\n'
        'python "import renode_hooks; renode_hooks.set_monitor(monitor)"\n'
    ).format(chip=chip, platform=platform, elf=elf_path,
             log=log_path, pty=pty_path, rcc_block=rcc_block,
             afec_block=afec_block, extra_block=extra_block,
             cs_include_block=cs_include_block,
             usart=_host_link_peripheral(chip), hooks=_HOOKS_PY)


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


_PROMPT_RE = re.compile(rb'\([\w-]+\)\s*$')


def _drain_monitor_until_prompt(sock, timeout=10.0):
    # Renode's monitor echoes commands and prints `(<context>) ` as a
    # prompt. We treat the prompt as the response delimiter. Returns
    # the bytes received between the last command and the prompt.
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
        if _PROMPT_RE.search(buf):
            return buf


def _send_monitor(sock, line):
    # Trailing \n is required; carriage return optional but harmless.
    sock.sendall((line + '\n').encode('utf-8'))
    return _drain_monitor_until_prompt(sock)


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
    # eg "step_trigger A 5 100 B 7 1" -> "step_trigger('A', 5, 100, 'B', 7, 1)"
    if len(parts) != 7:
        return None
    return ("step_trigger('%s', %d, %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3]),
               parts[4], int(parts[5]), int(parts[6])))


def _xlat_probe_step(parts):
    # probe_step <step_port> <step_pin> <reset_us> <force_per_step>
    #            <trig_port> <trig_pin> <val>
    if len(parts) != 8:
        return None
    return ("probe_step('%s', %d, %d, %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3]), int(parts[4]),
               parts[5], int(parts[6]), int(parts[7])))


def _xlat_bltouch(parts):
    # bltouch <ctrl_port> <ctrl_pin> <sensor_port> <sensor_pin> <invert>
    if len(parts) != 6:
        return None
    return ("bltouch('%s', %d, '%s', %d, %d)"
            % (parts[1], int(parts[2]), parts[3], int(parts[4]),
               int(parts[5])))


def _xlat_gpio(parts):
    # gpio <port> <pin> <val>
    if len(parts) != 4:
        return None
    return ("gpio_set('%s', %d, %d)"
            % (parts[1], int(parts[2]), int(parts[3])))


def _xlat_passthrough(parts):
    # Catch-all: forward as a function call with the same name. Lets
    # us add new fixture commands without re-touching the launcher as
    # long as renode_hooks.py grows the matching function.
    name = parts[0]
    args = ', '.join(repr(p) for p in parts[1:])
    return "%s(%s)" % (name, args)


_XLAT = {
    'step_trigger': _xlat_step_trigger,
    'probe_step': _xlat_probe_step,
    'bltouch': _xlat_bltouch,
    'gpio': _xlat_gpio,
}


# Default I2C addresses to register the empty fixture's
# i2c_default.register_responses against. Covers the LDC1612 default
# (0x2a) and its alternate (0x29) - the only I2C device klippy probes
# at startup with a fixed ID (and so the only one that hard-fails an
# entire printer config when the response is wrong / missing). Other
# I2C devices in printer configs get probed with their own register
# vocabularies; expand this list as concrete tests surface failures.
_DEFAULT_I2C_ADDRS = (0x2a, 0x29)


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
            _send('adc_default(%d)' % int(dv))

    # Per-channel overrides. The keys in `analog_in` are labels
    # (typically `_hot_extruder_N`) that the AVR fixture pusher maps
    # to the FIRST few configured analog_in OIDs. We don't have the
    # OID assignment here, so we use the order of dict iteration as
    # the channel index - close enough for the empty-fixture case
    # where the goal is just to get extruder thermistors to decode
    # to plausible temperatures rather than min_temp shutdowns.
    analog_in = fx.get('analog_in')
    if isinstance(analog_in, dict):
        for ch_idx, (_label, spec) in enumerate(analog_in.items()):
            if not isinstance(spec, dict):
                continue
            v = spec.get('default_value')
            if v is None:
                continue
            _send('adc_set(%d, %d)' % (ch_idx, int(v)))

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
                _send('i2c_register_response(1, %d, %s)'
                      % (addr, payload_json))


def _translate_fixture_command(line):
    parts = line.strip().split()
    if not parts:
        return None
    fn = _XLAT.get(parts[0], _xlat_passthrough)
    return fn(parts)


# ---------------------------------------------------------------------

def _control_socket_loop(sock_path, monitor_sock, stop_evt):
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
    #   barrier <usec>   - wait simulated time and ACK. simavr blocks
    #                      until its cycle counter advances <usec>;
    #                      Renode runs in real-time, so a wall-clock
    #                      sleep of usec microseconds is the closest
    #                      equivalent (firmware activity scales with
    #                      wall time after `start`). Auto-issues
    #                      `start` first if the CPU has not begun -
    #                      that is what makes klippy's identify+
    #                      clocksync handshake actually see firmware
    #                      output on the host pty.
    #
    # Hook commands (analog_in_default, step_trigger, ...) translate
    # to `python "<call>"` against renode_hooks.py.
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.settimeout(0.5)
    started = [False]

    def _ensure_started():
        if not started[0]:
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
                        # Cap the wall-clock wait so a malformed
                        # fixture cannot hang the launcher; simavr
                        # bridge's barrier is bounded by the runner's
                        # 5 s timeout on the OK ack anyway.
                        time.sleep(min(usec / 1e6, 5.0))
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
                    _send_monitor(
                        monitor_sock,
                        'python "%s"' % py_call.replace('"', r'\"'))
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
    resc = _render_resc(chip, os.path.abspath(args.elf), pty_path,
                        monitor_port, log_path)
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

    deadline = time.monotonic() + args.duration
    proc = subprocess.Popen(cmd, stdout=sys.stderr, stderr=sys.stderr)

    stop_evt = [False]

    def _shutdown(*_a):
        stop_evt[0] = True

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        # Wait for the pty Renode creates, then publish the symlink so
        # klippy's _wait_for_slave_link sees the slave path.
        if not _wait_for_path(pty_path,
                              min(deadline, time.monotonic() + 30.0)):
            sys.stderr.write(
                "renode_launcher: Renode did not create UART pty at "
                "%s within 30s; aborting\n" % pty_path)
            return 3
        # The pty file Renode creates IS the slave end (Renode opens
        # /dev/ptmx, then symlinks the published name to the slave
        # pts/N). Mirror linuxprocess: publish slave_link as a
        # symlink to it.
        try:
            os.unlink(args.slave_link)
        except OSError:
            pass
        os.symlink(pty_path, args.slave_link)

        monitor_sock = _connect_monitor(
            '127.0.0.1', monitor_port,
            min(deadline, time.monotonic() + 30.0))
        _drain_monitor_until_prompt(monitor_sock)

        # Apply the fixture-resident hooks (ADC defaults, I2C ID
        # responses) BEFORE the control loop accepts the runner's
        # `start` command - those hook calls have to be in place
        # before the CPU starts executing or klippy may probe a
        # peripheral and shutdown before the response arrives.
        _apply_fixture_to_renode(args.fixture_file, monitor_sock)

        import threading
        ctl_thread = threading.Thread(
            target=_control_socket_loop,
            args=(args.control_socket, monitor_sock, stop_evt),
            daemon=True)
        ctl_thread.start()

        # Wait for Renode subprocess or duration cap.
        while not stop_evt[0]:
            if proc.poll() is not None:
                break
            if time.monotonic() >= deadline:
                break
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
        for p in (args.slave_link, args.control_socket):
            try:
                os.unlink(p)
            except OSError:
                pass
        try:
            shutil.rmtree(workdir, ignore_errors=True)
        except OSError:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
