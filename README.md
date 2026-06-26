# ESP32 Servo + Tic Stepper Door Control

Desktop control system for a motorized door driven by two RDS5180 digital
servos (via an ESP32 over USB serial) and one NEMA 23 stepper (via a Pololu
Tic T249 controller over its own USB cable). A Tkinter GUI provides live
control, sequenced macros (HOME / OPEN / CLOSE), and an editable step panel.

## Architecture

```
Mac ──USB──> ESP32 ──PWM──> 2x RDS5180 servos (GPIO18 / GPIO19, 0–270°)
 │                     └──> limit switch (GPIO17, active-low)
 └──USB──> Pololu Tic T249 ──> NEMA 23 stepper   (driven via ticcmd CLI)
```

The Tic is **not** wired through the ESP32 — it connects directly to the
computer and is driven by the host GUI through the `ticcmd` command-line tool.

## Layout

| Path | Purpose |
|------|---------|
| `servo_demo/servo_demo.ino` | ESP32 firmware: 2 servos + limit switch, line-based serial protocol |
| `host_gui/servo_demo_gui.py` | Tkinter GUI: servos + stepper + macros + sequence editor |
| `host_gui/sequences.json` | Saved HOME / OPEN / CLOSE step sequences (editable in the GUI) |
| `diagnose_tic/` | Standalone Tic diagnostic sketch |
| `tools/` | Serial diagnostic helper scripts |
| `flash.sh` | Compile + upload helper for the ESP32 |

## Firmware (ESP32)

Two RDS5180 position servos on GPIO18 / GPIO19 (0–270°). A limit switch on
GPIO17 (internal pull-up, active-low) is reported in the `STATUS` line.

Build & flash (arduino-cli, esp32 core):

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 servo_demo
arduino-cli upload -p /dev/cu.usbserial-0001 \
  --fqbn esp32:esp32:esp32:UploadSpeed=115200 servo_demo
```

> Upload at 115200 baud — 921600 causes serial noise on this CP2102 board.

### Serial protocol (115200 baud, one command per line)

| Host → board | Effect |
|---|---|
| `PING` | replies `PONG` |
| `MOVE <id> <deg>` | set servo target angle (eased toward) |
| `JOG <id> <ddeg>` | nudge target by a relative amount |
| `SPEED <id> <dps>` | max slew rate, deg/sec (1–120) |
| `CENTER [id]` | center one servo, or both |
| `STOP [id]` | freeze one servo, or both (stays powered) |
| `LIMITS <id> <lo> <hi>` | soft min/max angle |
| `STATUS` | one status line for both servos + `limit=` |

Board → host status line includes per-motor `pos/target/speed/min/max/moving`
plus `limit=0/1` for the GPIO17 switch.

## Host GUI

```sh
pip3 install pyserial
python3 host_gui/servo_demo_gui.py --port /dev/cu.usbserial-0001
```

Features:

- Independent control of both servos (sliders, jog buttons, typed-angle GO).
- Stepper control via `ticcmd` (jog, absolute angle, speed, zero, stop).
- Per-motor **E-STOP** (halt but stay energized / holding torque).
- **Limit-switch indicator** LED (red = triggered, green = open).
- **Macro buttons** HOME / OPEN / CLOSE — ordered multi-motor routines that
  wait for each step to finish before the next.
- **EDIT** panel to add/remove/edit each macro step; saved to
  `host_gui/sequences.json` (survives relaunch).

## Requirements

- macOS (Apple Silicon tested), Python 3, `pyserial`
- arduino-cli with the `esp32` core + `ESP32Servo` library
- Pololu Tic Software (provides `ticcmd`)

## Notes

- The Tic T249 is open-loop: it always reports the *commanded* position and
  cannot detect lost steps. For repeatable zeroing, add a limit switch and use
  the controller's homing feature.
- Servos require an external 6.0–8.4 V supply with common ground to the ESP32 —
  do not power them from the ESP32's 3V3/5V rail.
