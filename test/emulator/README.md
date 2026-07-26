# Klipper MCU emulator test framework

A test harness that runs the actual Klipper firmware ELF under cycle-accurate
emulation and drives klippy end-to-end against it, so tests exercise real
firmware code paths — endstop sampling, trsync timers, software-PWM, CRC
validation, sensor protocols, TMC UART/SPI, ADC — instead of the all-zeros
`DummyResponse` that fileoutput mode produces.

Backends:

- **AVR** via [simavr](https://github.com/buserror/simavr) — `simavr_bridge.c`.
- **Cortex-M** via [Renode](https://renode.io) — `renode_launcher.py`. Covers
  STM32 (F1/F4/G0/H7), Atmel SAM3X/SAM4S/SAM4E/SAME70, SAMD21/SAMD51/SAME54,
  NXP LPC176x, HDSC HC32F460, RP2040.
- **linuxprocess** for klipper's host MCU build (`linuxtest.test`).
- **PRU** via a host build of `src/pru/` (`linuxpru.test`) — no third-party
  PRU CPU emulator exists, so `src/pru/host_pru.c` reimplements the IEP timer,
  the PRU INTC, the AM335x GPIO MMIO blocks, and `SHARED_MEM` against host
  gcc, and runs through the linuxprocess backend.

Multi-MCU fan-out is exercised across every backend pairing (simavr+simavr,
simavr+renode, renode+renode, simavr+linuxprocess). An optional deterministic
**tick-mode** locks klippy and the emulator step-for-step so simulated time is
decoupled from wall-clock; under the stock backend-less `scripts/ci-build.sh`
CI, `EMULATOR` tests degrade to fileoutput and stay green.

## Why this exists

`scripts/test_klippy.py` originally exercised klippy in fileoutput mode — fast,
but every MCU response is empty (`DummyResponse` in `klippy/mcu.py`). That's
enough for config-loading checks but doesn't reach code that branches on
actual sensor values, CRC validation, endstop trigger timing, etc.

The emulator backend is a higher-fidelity alternative: it runs the same C code
that ships on real hardware. Every endstop sample loop, trsync timer, and
software-PWM pulse executes identically to a real printer, so tests catch
protocol-level regressions that wouldn't show up under fileoutput mode.

## Architecture

```
   .test file                              firmware emulator subprocess
       |                                            |
       v                                            v
 test_klippy.py  <-- pty / socket -->  klipper.elf running under
       |                            simavr / Renode / linuxprocess
       |   control socket
       +-----------------------> bridge applies fixture state:
                                   adc / gpio / spi / i2c queues,
                                   step_trigger, bltouch state machine
```

`scripts/test_klippy.py` is the runner. The `EMULATOR <fixture.json>` directive
in a `.test` file:

1. Picks the backend for the test's MCU dict (`_backend_for_dict()`).
2. Spawns the bridge for that backend (simavr, Renode, linuxprocess, or PRU
   host-build).
3. Translates the test's fixture JSON to newline-terminated bridge control
   commands and writes them to the control socket.
4. Materializes the test's `.cfg` with `__EMULATOR_PTY__` (or
   `__EMULATOR_PTY_<chip>__` per MCU) rewritten to the actual link path and
   launches `klippy.py`.

## Files

### Bridges

- `simavr_bridge.c` — AVR bridge. Loads the ELF, runs simavr, exposes the UART
  as a pty (free-run mode) or a synchronous AF_UNIX socket (tick-mode
  lockstep), applies fixture state over a control socket. In tick mode it
  advances the AVR only toward a deterministic fixture-setup barrier, runs
  each `advance` to its target bounded by an output cap, and never drops
  output (KEEP-TAIL).
- `renode_launcher.py` — non-AVR dispatcher. Spawns Renode per-chip with the
  appropriate `.repl` (upstream ± local override) and Python/C# peripheral
  stubs; exposes the host link as a pty in both free-run and tick-mode.
- `renode_hooks.py` — IronPython hooks loaded into Renode. Implements the
  fixture protocol on the Renode side: `step_trigger`, `bltouch`, `gpio`,
  `adc`, the `sw_uart` TMC UART bit driver, the TMC SPI daisy-chain
  responder, `i2c_register_response`.

### Per-chip platform overrides and peripherals

`repl/` holds the local Renode platform descriptions (`.repl`) and C#
peripherals (`.cs`) for chips where the upstream Renode coverage is absent or
needs adjustment:

- `repl/<chip>.repl` — local platform: `hc32f460`, `lpc176x`, `rp2040`,
  `sam3x8e`, `sam4e8e`, `sam4s8c`, `samd21g18`, `samd51p20`, `same70q20b`,
  `stm32f103`, `stm32f4`, `stm32f429`, `stm32g0b1`, `stm32h723`.
- `repl/hc32f460_uart.cs` — HDSC HC32F460 USART (no upstream Renode model).
- `repl/rp2040_timer.cs` — RP2040 64-bit timer.
- `repl/same70_usbhs.cs` — SAME70 USBHS device-mode (for the native USB-CDC
  host link).
- `repl/lpc176x_adc.cs` — IRQ-driven LPC176x ADC (klipper's `src/lpc176x/adc.c`
  collects burst samples in `ADC_IRQHandler`, so a polled stub can't drive
  it).

`*_stub.py` files are `Python.PythonPeripheral` register-storeback stubs that
fill gaps in upstream Renode models — typically a clock-tree readiness bit, an
ADC handshake, or a chip-specific PWR/RCC/EFC register block. Naming pattern
is `<chip>_<peripheral>_stub.py`.

### Documentation

- `README.md` — this file.
- `TICK_PROTOCOL_DESIGN.md` — full design + correctness argument for the
  deterministic tick-mode protocol (state machine spec, the conditions C1–C9
  the implementation satisfies, throughput analysis, and the §5.2 byte-
  identical empirical-proof procedure).

## Control socket commands

Each bridge listens on a unix-domain stream socket; each newline-terminated
line is one command. Most commands are common across backends; a few are
backend-specific (noted below). Lines that don't parse are silently dropped.

| Command | Backends | Effect |
|---|---|---|
| `adc <ch> <mv>` | all | Drive ADC channel `<ch>` to a constant millivolt level |
| `gpio <port> <pin> <0\|1>` | all | Drive `<port><pin>` high/low |
| `spi <hexbytes>` | all | Replace the SPI MISO response queue (round-robin) |
| `spi_tmc_chip <port> <pin> <variant>` | renode | Register a TMC SPI chip (CS pin, datagram variant: `tmc2130` / `tmc5160` / `tmc2240` / `tmc2660`) on the SPI daisy-chain responder |
| `i2c <hexbytes>` | all | Replace the I2C read response queue (auto-ACKs writes/addressing) |
| `i2c_register_response <addr> <reg> <hexbytes>` | all | Per-register I2C response (multi-byte reads against a chip-id register) |
| `step_trigger <step_p> <step_pin> <count> <trig_p> <trig_pin> <val>` | all | Drive `<trig_p><trig_pin>` to `<val>` after `<count>` rising edges on the step pin |
| `bltouch <ctrl_p> <ctrl_pin> <sensor_p> <sensor_pin> <invert>` | all | BLTouch state machine on the named pins (decodes PWM commands by pulse duration, drives the sensor pin to match) |
| `spi_ads1220_chip <cs_p> <cs_pin> <drdy_p> <drdy_pin> <rate_hz>` | simavr | Register an ADS1220 chip (CS + DRDY pins); bridge pulses DRDY active-low at `<rate_hz>` SPS via a simavr cycle timer and de-asserts on each 3-byte continuous-mode read. Multi-chip configs receive a staggered phase to keep both chips' DRDY assertions out of the same poll tick |
| `eddy_probe_ramp …` | simavr | LDC1612 frequency-count ramp tied to Z stepper position (drives the eddy virtual endstop) |
| `spi_adxl345_chip <cs_p> <cs_pin> <vib_hz> <amp> <base_z>` | simavr | ADXL345 streaming model: register file + DEVID, 32-deep FIFO paced at the BW_RATE klippy programs, synthetic x-axis tone at `<vib_hz>` (index-based time, deterministic) |
| `probe_step <step_p> <step_pin> <…>` | simavr | ADS1220 force ramp tied to step position (drives the load-cell `trigger_analog` detector) |
| `ldc1612_ramp …` | simavr + renode | LDC1612 register-aware I2C responder + ramped DATA0 count (renode: `renode_hooks.ldc1612_ramp`, a DummyI2CSlave with STATUS period gating in the 64 MHz timer domain) |
| `sw_uart <port> <pin> <…>` | renode | Software-UART responder on a GPIO RX pin (single-wire or multi-drop TMC2208/TMC2209) |
| `barrier <usec>` | all | Tick-mode setup: advance to a deterministic sim-cycle target, then pause until tick-connect (see `TICK_PROTOCOL_DESIGN.md §4`) |

The bridge also accepts a separate `--tick-socket <path>` that klippy connects
to in tick mode (see "Tick mode" below). Its protocol is `advance <T>\n` from
klippy and `done <T_actual>\n` from the bridge, exchanged once per reactor
iteration; full spec in `TICK_PROTOCOL_DESIGN.md`.

## Fixture JSON keys

Tests drop a `<test>.fixture.json` next to `<test>.test` to script per-test
peripheral state. The runner translates these keys into bridge commands at
emulator startup:

- `analog_in_default` — global ADC default in raw 13-bit units (~25°C for
  typical thermistors). Optional `by_pin` map sets per-pin ADC values for
  tests with mixed sensor types on the combined-sensor path.
- `auto_trsync_trigger_ticks` — presence of this key enables the
  endstop-after-N-steps homing helper for every `[stepper_<axis>]` with a
  plain GPIO endstop in the test's `.cfg`. Klippy's homing fires after the
  bridge has counted N stepper edges.
- `bltouch` — `{control_pin, sensor_pin, invert}`. Configures the bridge's
  BLTouch state machine on the named pins.
- `auto_trigger_after_steps` — paired with `bltouch`: drives the bltouch
  sensor pin to triggered after N stepper edges on the Z step pin.
- `adxl345` — `{vib_freq_hz, amp_raw, base_z_raw}`: register the bridge's
  ADXL345 streaming model on every `[adxl345*]` section's CS pin. Gives
  `ACCELEROMETER_MEASURE` real 13-bit samples and `TEST_RESONANCES` a
  clean spectral line to find (see `adxl345.fixture.json`).
- `config_overrides` — `{section: {option: value}}` rewrites applied to the
  emulator's materialized cfg copy only (regular sections and the `#*#`
  SAVE_CONFIG autosave block; an option absent from the cfg is inserted).
  Lets a test carry emulator-only tunings — e.g. `delta_calibrate`'s
  real-printer `rotation_distance`, or `load_cell` pinning a second bulk
  sensor below the AVR link ceiling — while the shared `test/klippy` cfg
  stays byte-identical to upstream for fileoutput CI. The runner's cfg
  parsers (ADS1220 sample rates, endstop pins, …) read the overridden copy.
- `spi_response` — hex byte stream the bridge round-robins back as SPI MISO
  data. Used by tests with thermocouples or other SPI-resident sensors that
  need plausible (in-range) read responses.
- `spi_tmc_chip` — register one or more TMC SPI chips on the renode SPI-chain
  responder (`{port, pin, variant}` per chip).
- `i2c.<label>.reads` — lists of read responses per i2c slave label.
  Concatenated into the bridge's I2C queue in fixture order.
- `i2c_default.register_responses` — chip-id register payloads klippy probes
  once at startup.
- `eddy_probe_ramp` — LDC1612 frequency-count ramp tied to Z (see commit
  `b0a456b5f`). On renode the same fixture key drives the
  `renode_hooks.ldc1612_ramp` model; `eddy_arm.fixture.json` documents the
  tap-specific contact-hysteresis geometry (near-plateau `depress_per_step`,
  knee below the G28 trigger crossing).
- `klippy_deadline` — per-test wall-clock deadline override (seconds,
  default 180). For tests whose firmware wakes at bulk-sensor rates under
  renode tick mode (each advance costs ~ms of RunFor overhead), e.g.
  `eddy_arm`. Pair with `EXPECT_LOG_CONTAINS` completion assertions - the
  emulator path treats deadline-without-crash as success, so only log
  assertions prove the gcode actually finished.
- `probe_step` — ADS1220 force ramp tied to Z step edges.
- `sim_time` — drive klippy on the MCU's clock (advisory free-run; the bridge
  publishes `cycle / freq` into the `KLIPPY_SIM_TIME_FILE` mmap so
  `reactor.monotonic()` returns sim-time).
- `tick_mode` — if `true`, drive klippy and the emulator in lockstep over a
  tick socket. Implies `sim_time: true`.

## Tick mode (deterministic-time lockstep)

The default `sim_time: true` path puts klippy on the MCU's clock but lets the
emulator free-run with a wall-clock ceiling — the two clocks stay *consistent*
but neither side drives the other's progress. Under host CPU contention the
emulator can fall behind and the relative ordering of klippy I/O against MCU
events becomes nondeterministic.

`tick_mode: true` upgrades the relationship to lockstep. The runner allocates
an extra unix socket and passes it as `--tick-socket` to the bridge plus
`KLIPPY_TICK_SOCKET=<path>` to klippy. Klippy's reactor connects on startup;
whenever it has no fd ready and no timer due it sends `advance <T>\n` over the
socket. The bridge runs `avr_run` (or steps Renode forward, or runs the host
process) until `cycle / frequency >= T`, updates the sim_time mmap, and
replies `done <T_actual>\n`. The emulator never advances past where klippy
asked, and klippy never processes events past where the emulator has advanced
— so timing is deterministic regardless of host load.

`tick_mode: true` implies `sim_time: true` (the reactor still reads its
monotonic clock from the mmap). The wall-clock throttle that caps the
emulator in non-tick `sim_time` runs is bypassed once klippy connects; before
that the bridge enters a barrier-pause setup (see `TICK_PROTOCOL_DESIGN.md
§4`) so fixture timer phases are run-to-run identical.

### Determinism

For a fixed test (config + gcode + fixtures + firmware ELF), every run
produces a bit-identical sequence of `advance` / `done` round trips and AVR
output, independent of host scheduling. The full state machine spec and
correctness conditions are in `TICK_PROTOCOL_DESIGN.md`; the §5.2 byte-
identical proof procedure is the empirical check. Determinism is
validated against `temperature.test` (174 advances/run) and
`load_cell.test` (490 advances/run) end-to-end in the default gate.

`KLIPPY_TICK_TRACE=<path>` emits a per-round-trip CSV on both sides;
`KLIPPY_TICK_STALL_LOG=1` and `BRIDGE_TICK_DIAG=1` trace the liveness guards
(see `TICK_PROTOCOL_DESIGN.md §5.1`). All three ship disabled.

### Real hardware

Every tick-mode change is gated behind `KLIPPY_TICK_SOCKET`. With the env
unset, klippy's serial transport, reactor, and serialqueue paths are
byte-identical to upstream `master`. The bundled klippy fixes shipped
alongside the framework are independently argued byte-identical on real
hardware (see the PR body's "Bundled upstream fixes" list).

### Environment knobs

The framework's full env-knob surface, in summary:

| Variable | Side | Purpose |
|---|---|---|
| `KLIPPY_TICK_SOCKET` | klippy | Tick-mode lockstep host link (path or `:`-separated list for multi-MCU); the master gate for every tick-mode code path. Unset on real hardware. |
| `KLIPPY_SIM_TIME_FILE` | klippy | Replaces `reactor.monotonic()`'s clock source with a memory-mapped `double` the bridge updates each tick. Implied by `sim_time: true` and `tick_mode: true`; unset on real hardware. |
| `KLIPPY_TICK_TRACE` | klippy + bridge | Per-round-trip CSV (`.klippy` / bridge B-trace) — input to the §5.2 byte-identical proof procedure. |
| `KLIPPY_TICK_STALL_LOG`, `KLIPPY_TICK_STALL_LIMIT` | klippy | Surface the reactor livelock guard (`§5.1` / `_tick_stall_log`). |
| `BRIDGE_TICK_DIAG` | bridge | Log every advance read and `done` written (simavr + renode). |
| `KLIPPY_LDC1612_RAMP_TRACE` | simavr bridge | Per-step-edge CSV `cycle dir_irq_value descend_level descending net_descent` for the `ldc1612_ramp` hook (used to derive/debug the eddy fixture geometry). Off by default. |
| `RENODE_PEEK_SHUTDOWN`, `RENODE_PEEK_DEBUG` | renode launcher | Optional firmware-shutdown reason surfacing (debug aid). |
| `KLIPPER_W1_DEVICES_PATH` | firmware (linux MCU) | Overrides the `/sys/bus/w1/devices` prefix the DS18B20 driver scans. Used by `linuxtest.test` to point at a tempdir mock; unset on real hardware. |

All ship disabled / inert by default.

## CI integration

The stock `scripts/ci-build.sh` builds dicts only (no ELF / simavr / Renode).
`EMULATOR`-directive tests detect the missing backend and **degrade to
fileoutput** (the pre-EMULATOR path); five tests whose assertions need real
MCU responses carry `REQUIRES_EMULATOR` and skip instead. The emulator-test
Docker image always has the backend, so its end-to-end coverage is unchanged.

`printers.test` sweeps every shipped printer config against the per-MCU
dicts. Under the emulator backend it cleanly skips configs it can't validate
against the image's firmware builds — those whose MCU dict isn't built, and
those that drive a pin the firmware reserves for the host-link serial UART
(e.g. a board with a thermistor on a SAM3X UART0 pin) — rather than aborting
the sweep.

## Running the suite

The Dockerfile at `scripts/Dockerfile.emulator-test` builds an image with
simavr, Renode, the bridge binaries, and `.elf` + `.dict` artifacts for every
supported MCU build. Once built:

```sh
docker build -t klipper-emu -f scripts/Dockerfile.emulator-test .
docker run --rm klipper-emu             # runs the default gate (~55 tests)
```

To iterate on a single test without rebuilding the image — mount the current
tree's `test/` and `scripts/test_klippy.py` over the baked image:

```sh
docker run --rm \
    -v $(pwd)/test:/klipper/test \
    -v $(pwd)/scripts/test_klippy.py:/klipper/scripts/test_klippy.py \
    klipper-emu \
    /venv/bin/python scripts/test_klippy.py \
    -d ci_build/dict --force-emulator test/klippy/<one_test>.test
```

`--force-emulator` runs every test under the bridge regardless of whether the
test file specifies an `EMULATOR <fixture>` line.

To iterate on `simavr_bridge.c` itself, mount it read-only and rebuild
in-container against the baked simavr library (same `gcc` line as the
Dockerfile uses — search for it there).

## Validating the Renode wiring

Because Renode is Linux-only in our setup (Mono runtime, packaged as a .deb),
the launcher and hook code is hard to iterate on locally outside Linux.
`validate_renode.py` is a standalone probe runner that exercises every Renode
API the launcher relies on (TCP Monitor connect, prompt parsing, `mach
create`, `LoadPlatformDescription`, `LoadELF`, `CreateUartPtyTerminal`,
`python` state persistence, `machine[...]` peripheral lookup,
`Connections[N]` indexing, `AddStateChangedHook` registration, firmware boot
bytes on the pty) and reports PASS/FAIL with the raw Monitor response per
probe.

```sh
docker run --rm klipper-emu \
    /venv/bin/python test/emulator/validate_renode.py
```

A failed probe pinpoints which assumption in `renode_launcher.py` /
`renode_hooks.py` needs adjusting before the full `scripts/test_klippy.py`
path is run end-to-end against an STM32 (or other ARM) target.

## Per-chip and per-printer coverage

Boot/identify/clocksync smoke for every chip family lives in
`test/klippy/<chip>_smoke.test`. Full-pin-map per-printer tests
(`duet2_maestro`, `skr_mini_e3_v2`, `skr_pico`, `duet3_6hc`, `duet3_6xd`,
`skr_v1_4`, `fysetc_spider`, `skr_mini_e3_v3`, `octopus_pro_h723`,
`archim2`, `duet2_wifi`, `duet3_mini`, `anycubic_kobra_go`,
`samd21_printer`) drive a single real printer config — steppers, endstops,
TMC, thermistors, heaters/fans — end-to-end in tick mode. Multi-MCU triples
(`multi_mcu_*`) cover every backend pairing.

Design rationale for each per-chip wiring decision (register stubs, repl
trade-offs, validation results) lives in the corresponding commit message;
`git log master..HEAD --oneline --grep test/emulator -- test/emulator/` is
the index.
