# INA219 Servo Door Control (branch: `new-v2`)

An alternate implementation of the door controller on this branch: **two RDS5180
digital servos driven by an ESP32, with live current/voltage sensing via an
INA219**. No stepper / Tic controller is used here — current sensing replaces it
for closed-loop safety (snug detection, over-current e-stop).

This variant was built and verified end-to-end on hardware.

## Hardware

```
12V PSU ──> ZK/XK-12KX buck (CC/CV, set ~7.6V out) ──> INA219 (Vin+ -> Vin-) ──> servo red x2
                                                          │ I2C
ESP32 ── SDA=GPIO21  SCL=GPIO22 ── INA219 (addr 0x40, R100 shunt, ±3.2A range)
      ── GPIO18 ── servo #0 "door"   (RDS5180, 270°, 500–2500µs)
      ── GPIO19 ── servo #1 "handle" (RDS5180, 270°)
      ── GPIO17 ── door-closed microswitch (INPUT_PULLUP)
      ── common GND with buck OUT- and both servo black wires
      └─ 1000µF cap across the servo supply to absorb inrush
```

- Servo supply: 7.4–7.6V (RDS5180 rated DC 6–8.4V). Stall ~5A each — **exceeds
  the INA219's ±3.2A range**, so startup peaks read clipped at 3200mA.
- Door mechanical range: **55° (open) … 220° (close command)**; snugs shut around
  ~190–217° depending on the frame.
- Handle: **0° = unlocked, 125° = locked**.

## Files

| Path | What |
|------|------|
| `servo_current_monitor/servo_current_monitor.ino` | Main firmware: sequences, e-stop, current telemetry |
| `host_gui/servo_gui.py` | Tkinter GUI: Open/Close/Home/STOP/START, sliders, live current chart |
| `diagnostics/ina219_monitor/` | Minimal INA219 current/voltage reader (bring-up) |
| `diagnostics/ina219_diag/` | I2C scan + raw INA219 register dump |
| `tools/watch_ina219.command` | Double-click serial viewer (pyserial) |

## Firmware serial protocol (115200 baud)

| Command | Action |
|---------|--------|
| `open` | Sequence: unlock (handle→0), door→90, door→55 |
| `close` | Sequence: door→220 (stops on resistance = snug), wait for door-closed confirmation (physical D17 **or** virtual switch), handle→125 lock, relieve pressure |
| `home` | Sequence: unlock (handle→0), door→90 |
| `cycle <n>` | Endurance test: repeat open → 5s pause → close → 5s pause, n times, with live count |
| `stop` | Emergency stop: freeze all motion + abort sequence |
| `start` (`reset`) | Resume / engage servos |
| `<ch> <angle>` | Manual move (ch 0=door limited 55–220, ch 1=handle) |
| `speed <n>` | Motion speed, 5–400 deg/s (default 30) |
| `u <ch> <us>` / `min` / `max` / `cal` | Raw pulse + calibration |
| `trip` | Test the e-stop path |

Telemetry line (every 150 ms):
```
V=7.60V  I=72.6mA  P=534mW   [door=190  handle=0  sw=0  vsw=0  tc=119]  {servos:idle}
```
Status suffixes: `!!ESTOP:reason!!`, `{seq:name:step}`, `{servos:idle}`, `{cyc:count/target}`.
`sw` = physical D17 switch, `vsw` = virtual (current-signature) switch,
`tc` = lifetime cycle counter (persisted in NVS, survives power loss and reflash).

## Safety logic

- **E-stop rules** (while a servo is actively moving): instantaneous > 2000 mA,
  **or** > 1000 mA sustained for 5 s → freeze motion.
- **Snug detection**: during the close sequence's door-travel step, the same
  current rule means "door hit the frame" — the door stops there and the
  sequence continues (it does *not* e-stop).
- **Close steps after the door stops are exempt** from the current rule: the door
  servo holds pressure (high current is normal) while the handle turns to lock —
  a single shared current sensor must not freeze the handle here.
- **Door-closed confirmation before locking**: physical D17 switch **or** a virtual
  switch — a 3 s rolling average of current > 1600 mA while pressing (robust to
  current fluctuation). Either one advances to the lock step; 60 s timeout → e-stop.
- **Endurance testing**: `cycle <n>` (GUI: "Run Cycle Test") runs open/close cycles
  with 5 s pauses and a live counter; any e-stop aborts the run at the failing cycle.
- **Servos idle on boot/reset** (no PWM) so a reset never slams the door; the
  first command engages them.
- Over-current after an e-stop escalates to detaching PWM (power-off).

## Build & run

```bash
# firmware (arduino-cli, ESP32 core + Adafruit INA219 + ESP32Servo libs)
arduino-cli compile --fqbn esp32:esp32:esp32 servo_current_monitor
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 servo_current_monitor

# host GUI (needs pyserial + matplotlib)
python3 host_gui/servo_gui.py
```
