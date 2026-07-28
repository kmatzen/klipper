# MCU emulator test framework

Klipper ships a real-firmware MCU emulator framework that runs the
actual `klipper.elf` binary under cycle-accurate emulation
([simavr](https://github.com/buserror/simavr) for AVR,
[Renode](https://renode.io) for Cortex-M, host-process for the
linuxprocess and TI PRU host builds) and drives [klippy](Code_Overview.md)
end-to-end against it. The framework is dev/CI infrastructure — it is
exercised by the [test/klippy](../test/klippy) regression suite rather
than by an end-user printer — and exists alongside the lighter-weight
fileoutput backend the regression suite has used historically.

This document covers the user-relevant pieces: what the framework
covers, when (and why) you might want to run it, and where to point a
new test or printer at it. Contributor-facing internals — fixture
protocol, register-stub rationale, per-chip wiring decisions, the
deterministic tick-mode protocol spec — live in
[test/emulator/README.md](../test/emulator/README.md) and
[test/emulator/TICK_PROTOCOL_DESIGN.md](../test/emulator/TICK_PROTOCOL_DESIGN.md).

## Why it exists

`scripts/test_klippy.py` has historically run klippy against a
**fileoutput** backend, which is symbolic — every MCU response is the
all-zeros `DummyResponse` from `klippy/mcu.py`. That is fast and
sufficient for config-loading checks, but does not reach klippy code
paths that branch on actual sensor values, CRC validation, endstop
trigger timing, software-PWM pulses, software-UART traffic, TMC
SPI/UART round trips, or `trsync`-driven homing.

The emulator framework is a higher-fidelity alternative: every endstop
sample loop, `trsync` timer, software-PWM pulse, and i2c/SPI bus
exchange executes identically to what runs on a real printer, so tests
catch protocol-level regressions that fileoutput cannot.

## What it covers

Boot, identify, and clocksync smoke for every supported MCU family is
exercised by per-chip `test/klippy/<chip>_smoke.test` files. The
backends today are:

- **AVR** (`atmega2560`, `atmega1280`, `atmega1284p`, `atmega644p`,
  `at90usb1286`) — `simavr` via `test/emulator/simavr_bridge.c`.
- **Cortex-M** — `Renode` via `test/emulator/renode_launcher.py`. Covers
  STM32 (F1/F4/G0/H7), Atmel SAM3X/SAM4S/SAM4E/SAME70,
  SAMD21/SAMD51/SAME54, NXP LPC176x, HDSC HC32F460, and RP2040.
- **linuxprocess** for the host-MCU build (`linuxtest.test`), with
  optional DS18B20 mocking via `KLIPPER_W1_DEVICES_PATH`.
- **TI PRU** via a host build of `src/pru/` (`linuxpru.test`): no
  third-party PRU CPU emulator exists, so `src/pru/host_pru.c`
  reimplements the IEP timer, PRU INTC, AM335x GPIO MMIO blocks, and
  `SHARED_MEM` against host gcc.

Beyond the smoke layer, the framework drives full-pin-map per-printer
configurations (BTT SKR Pico / Mini E3 V2/V3, FYSETC Spider, Duet2
Maestro / WiFi, Duet3 6HC/6XD/Mini, Octopus Pro H723, Archim2, SKR V1.4,
Anycubic Kobra Go, SAMD21G18) end-to-end through TMC2208/2209/2240/2660
UART or TMC2130/5160 SPI traffic and live thermistor ADC. Multi-MCU
fan-out triples cover every backend pairing — simavr+simavr,
simavr+Renode, Renode+Renode, simavr+linuxprocess — including G28
homing of a stepper hosted on a satellite MCU.

A subset of the standard `test/klippy` regression tests also runs
end-to-end against the emulator (rather than against fileoutput) so
their real firmware paths are exercised:

- `temperature` — thermistor, PT1000, AD595, PT100 ADC + MAX6675/31855/
  31856/31865 SPI thermocouples.
- `bltouch`, `screws_tilt_adjust` — BLTouch deploy/stow/touch servo
  state machine + probe sampling.
- `tmc_spi`, `tmc_spi_divergence` — tmc2130/5160/2240/2660 SPI register
  read-back.
- `tmc` — TMC stallguard sensorless homing (`virtual_endstop`) +
  `[endstop_phase]` MSCNT calibration across tmc2130/5160/2240/2208/
  2209/2660.
- `delta`, `delta_calibrate` — kinematic motion + DELTA_CALIBRATE /
  DELTA_ANALYZE.

## When to use it

The framework is **opt-in**: the stock `scripts/ci-build.sh` builds
dicts only (no ELF, simavr, or Renode), and `EMULATOR`-directive tests
degrade to fileoutput when no backend is built (tests whose assertions
need real MCU responses — the per-chip / per-printer / multi-MCU
suites plus a handful of standard regressions such as `temperature`,
`sht3x_crc_regression`, and `tmc_spi_divergence` — carry
`REQUIRES_EMULATOR` and skip instead). The full backend ships in a
Docker image built from `scripts/Dockerfile.emulator-test`.

Typical contributor flows:

- **Sanity-check a firmware change.** Mount your local `src/` over the
  baked image, rebuild the affected MCU's `.elf` in-container, and
  re-run the relevant per-chip smoke test (see
  [test/emulator/README.md](../test/emulator/README.md#running-the-suite)
  for the mount-over recipe).
- **Add a new per-printer test.** Drop a `<printer>.cfg` and
  `<printer>.fixture.json` into `test/klippy/`; reuse the chip's
  existing UART/SPI responder and ADC stub. The framework auto-skips
  pin conflicts the firmware reserves for the host-link UART.
- **Reproduce a protocol-level regression on real hardware in CI.**
  An emulator-backed `test/klippy/<repro>.test` will catch
  regressions in protocol framing, trsync timing, and bulk-sensor
  flow that fileoutput cannot.

## Deterministic tick mode

The framework's per-printer tests run klippy and the emulator
step-for-step over an opt-in `tick_mode` lockstep protocol so simulated
time is decoupled from wall-clock — for a fixed test, every run
produces a byte-identical sequence of `advance` / `done` round trips and
firmware output. The state machine and correctness conditions are
specified in
[test/emulator/TICK_PROTOCOL_DESIGN.md](../test/emulator/TICK_PROTOCOL_DESIGN.md);
the §5.2 byte-identical proof procedure is the empirical check, and is
validated for `temperature.test` and `load_cell.test`.

Tick mode is **gated entirely behind the `KLIPPY_TICK_SOCKET`
environment variable** — with it unset, klippy's serial transport,
reactor, and serialqueue paths are byte-identical to upstream `master`,
so the deterministic-time machinery does not affect real-hardware
behavior.
