# Klipper MCU emulator (simavr bridge)

A small wrapper around [simavr](https://github.com/buserror/simavr) that
runs the actual klipper firmware ELF under cycle-accurate AVR simulation
and exposes its UART as a pty. Tests connect klippy to the pty the same
way it would connect to a serial MCU, and drive peripheral state
(sensor reads, endstop transitions, BLTouch state) via a control
socket.

## Why this exists

`scripts/test_klippy.py` originally exercised klippy in fileoutput
mode - fast, but every MCU response is empty (`DummyResponse` in
`klippy/mcu.py`). That's enough for config-loading checks but doesn't
reach code that branches on actual sensor values, CRC validation,
endstop trigger timing, etc.

The bridge is a higher-fidelity alternative: it runs the same C code
that ships on real hardware. Every endstop sample loop, trsync timer,
and software-PWM pulse executes identically to a real printer, so
tests catch protocol-level regressions that wouldn't show up under
fileoutput mode.

## Architecture

```
   .test file                              simavr_bridge subprocess
       |                                            |
       v                                            v
 test_klippy.py  <----- pty ----->  klipper.elf running under simavr
       |                                            ^
       |   control socket                           |
       +-----------------------> bridge applies fixture state:
                                   adc / gpio / spi / i2c queues,
                                   step_trigger, bltouch state machine
```

`scripts/test_klippy.py` does the wiring:

1. Spawns `ci_build/simavr_bridge` with `--elf <path>.elf`,
   `--slave-link <pty>`, `--control-socket <path>`.
2. Waits for the bridge to publish the pty's slave path.
3. Translates the test's fixture JSON into newline-terminated bridge
   control commands and writes them to the control socket.
4. Materializes the test's `.cfg` with `__EMULATOR_PTY__` rewritten
   to the actual pty path and launches `klippy.py`.

## Files

- `simavr_bridge.c` - the bridge program. Loads klipper.elf, runs
  simavr, exposes UART, listens on the control socket, hooks the
  AVR's peripheral IRQs to provide fixture-driven state.
- `empty_fixture.json` - default fixture (analog_in defaults, i2c
  chip-id reads, auto_trsync_trigger_ticks for homing) used when
  `--force-emulator` injects a fixture and a test doesn't bring its
  own `<test>.fixture.json`.
- `README.md` - this file.

## Control socket commands

The bridge listens on a unix-domain stream socket; each newline-
terminated line is one command. Lines that don't parse are silently
dropped.

| Command                                              | Effect |
|------------------------------------------------------|--------|
| `adc <ch> <mv>`                                      | Drive ADC channel 0-15 to a constant millivolt level |
| `gpio <port> <pin> <0\|1>`                           | Drive `PORT<port>` `pin<0-7>` high/low |
| `spi <hexbytes>`                                     | Replace the SPI MISO response queue (round-robin) |
| `i2c <hexbytes>`                                     | Replace the I2C read response queue (auto-ACKs writes/addressing) |
| `step_trigger <step_p> <step_pin> <count> <trig_p> <trig_pin> <val>` | Drive `<trig_p><trig_pin>` to `val` after `count` rising edges on the step pin |
| `bltouch <ctrl_p> <ctrl_pin> <sensor_p> <sensor_pin> <invert>` | Configure the BLTouch state machine (decodes PWM commands by pulse duration, drives the sensor pin to match) |

## Fixture JSON keys

Tests drop a `<test>.fixture.json` next to `<test>.test` to script
per-test peripheral state. The runner translates these keys into
bridge commands at simavr startup:

- `analog_in_default` - global ADC default in raw 13-bit units (~25C
  for typical thermistors). Optional `by_pin` map sets per-pin ADC
  values for tests with mixed sensor types on the combined-sensor
  path.
- `auto_trsync_trigger_ticks` - presence of this key enables the
  endstop-after-N-steps homing helper for every `[stepper_<axis>]`
  with a plain GPIO endstop in the test's `.cfg`. Klippy's homing
  fires after the bridge has counted N stepper edges.
- `bltouch` - `{control_pin, sensor_pin, invert}`. Configures the C
  bridge's BLTouch state machine on the named pins.
- `auto_trigger_after_steps` - paired with `bltouch`: drives the
  bltouch sensor pin to triggered after N stepper edges on the Z
  step pin (modeling a probe-touch after toolhead motion into the
  bed).
- `spi_response` - hex byte stream the bridge round-robins back as
  SPI MISO data. Used by tests with thermocouples or other SPI-
  resident sensors that need plausible (in-range) read responses.
- `i2c.<label>.reads` - lists of read responses per i2c slave label.
  Concatenated into the bridge's I2C queue in fixture order.
- `i2c_default.register_responses` - chip-id register payloads
  klippy probes once at startup.

## Running

The Dockerfile at `scripts/Dockerfile.emulator-test` builds an image
with simavr, the bridge, and `.elf` + `.dict` artifacts for every
supported AVR config. Once built, run:

```sh
docker run --rm -v $(pwd)/scripts:/klipper/scripts \
    -v $(pwd)/test:/klipper/test \
    klipper-emulator-test:latest \
    /venv/bin/python scripts/test_klippy.py \
    -d ci_build/dict --force-emulator test/klippy/*.test
```

`--force-emulator` runs every test under the bridge regardless of
whether the test file specifies an `EMULATOR <fixture>` line.
