/*
 * ESP32 door controller — USB-serial control (NO Wi-Fi / NO hotspot).
 *
 * The computer talks to this board directly over the USB cable using a
 * simple line-based text protocol at 115200 baud. A desktop GUI
 * (host_gui/door_gui.py) sends commands and shows live status.
 *
 * Protocol (one command per line, '\n' terminated)
 * ------------------------------------------------
 *   Host -> board:
 *     PING                -> board replies "PONG"
 *     OPEN                -> move stepper to open angle
 *     CLOSE               -> move stepper to closed angle
 *     STOP                -> halt + hold stepper immediately
 *     STEP <deg>          -> move stepper output shaft to <deg> (-180..180)
 *     SERVO <deg>         -> move servo to <deg> (0..180)
 *     STATUS              -> board replies one STATUS line (see below)
 *
 *   Board -> host (always one line, prefixed by a tag):
 *     STATUS online=<0|1> state=<str> err=0x%04X vin_mv=<n> angle=<deg> target=<deg> servo=<deg>
 *     OK <echoed-command>
 *     ERR <reason>
 *
 * Wiring
 * ------
 * SERVO  -> signal GPIO13, V+ 5V, GND common with ESP32.
 * Tic T249 over I2C: SDA->GPIO21, SCL->GPIO22, common GND,
 *   Tic logic power (USB/5V), 12-24V on VIN, coils A1/A2/B1/B2,
 *   Tic control mode = "Serial / I2C / USB", current limit set for NEMA 23.
 *
 * Libraries: "Tic" (Pololu), "ESP32Servo".
 */

#include <Wire.h>
#include <Tic.h>
#include <ESP32Servo.h>

// ----- Pins -----
const int SERVO_PIN = 13;
// I2C defaults: SDA = GPIO21, SCL = GPIO22

// ----- Servo limits -----
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 180;

// ----- Stepper (Tic) math -----
const long  FULL_STEPS_PER_REV = 200;    // 1.8 deg NEMA 23
const long  MICROSTEPS         = 16;     // 1/16 microstepping
const float GEARBOX_RATIO      = 5.0f;   // 5:1 reduction (output:motor)
const float MICROSTEPS_PER_OUTPUT_DEG =
    (FULL_STEPS_PER_REV * MICROSTEPS * GEARBOX_RATIO) / 360.0f;   // ~44.44

const int STEP_MIN_DEG = -180;
const int STEP_MAX_DEG =  180;

// Door positions (output-shaft degrees). Tune to your mechanism.
const int DOOR_CLOSED_DEG = 0;
const int DOOR_OPEN_DEG   = 90;

// ----- State -----
int  currentServoDeg = 90;
int  targetStepDeg   = 0;

Servo servo;
TicI2C tic;          // default I2C address 14

// ---------------------------------------------------------------------------
void moveServo(int deg) {
  deg = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  currentServoDeg = deg;
  servo.write(deg);
}

void moveStepperToDeg(int deg) {
  deg = constrain(deg, STEP_MIN_DEG, STEP_MAX_DEG);
  targetStepDeg = deg;
  long target = lround(deg * MICROSTEPS_PER_OUTPUT_DEG);
  tic.exitSafeStart();
  tic.energize();
  tic.setTargetPosition(target);
}

bool ticOnline() {
  tic.getOperationState();          // any read
  return tic.getLastError() == 0;   // 0 == I2C transfer ok
}

const char* opStateStr(TicOperationState s) {
  switch (s) {
    case TicOperationState::Reset:            return "Reset";
    case TicOperationState::Deenergized:      return "Deenergized";
    case TicOperationState::SoftError:        return "SoftError";
    case TicOperationState::WaitingForErrLine:return "WaitingForErrLine";
    case TicOperationState::StartingUp:       return "StartingUp";
    case TicOperationState::Normal:           return "Normal";
    default:                                  return "Unknown";
  }
}

void sendStatus() {
  bool online = ticOnline();
  long posU   = online ? tic.getCurrentPosition() : 0;
  int  angle  = lround(posU / MICROSTEPS_PER_OUTPUT_DEG);
  uint16_t err = online ? tic.getErrorStatus() : 0;
  uint32_t vin = online ? tic.getVinVoltage() : 0;
  const char* st = online ? opStateStr(tic.getOperationState()) : "offline";

  char line[160];
  snprintf(line, sizeof(line),
    "STATUS online=%d state=%s err=0x%04X vin_mv=%lu angle=%d target=%d servo=%d",
    online ? 1 : 0, st, err, (unsigned long)vin, angle, targetStepDeg, currentServoDeg);
  Serial.println(line);
}

// ---------------------------------------------------------------------------
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Split command and optional argument.
  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  String arg = (sp < 0) ? ""   : line.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "PING") {
    Serial.println("PONG");
  } else if (cmd == "OPEN") {
    moveStepperToDeg(DOOR_OPEN_DEG);
    Serial.println("OK OPEN");
  } else if (cmd == "CLOSE") {
    moveStepperToDeg(DOOR_CLOSED_DEG);
    Serial.println("OK CLOSE");
  } else if (cmd == "STOP") {
    tic.haltAndHold();
    Serial.println("OK STOP");
  } else if (cmd == "STEP") {
    moveStepperToDeg(arg.toInt());
    Serial.println("OK STEP");
  } else if (cmd == "SERVO") {
    moveServo(arg.toInt());
    Serial.println("OK SERVO");
  } else if (cmd == "STATUS") {
    sendStatus();
  } else {
    Serial.print("ERR unknown_command ");
    Serial.println(cmd);
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // --- Servo ---
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);  // min/max pulse (us) — adjust per servo
  moveServo(currentServoDeg);

  // --- I2C / Tic ---
  Wire.begin();                        // SDA=21, SCL=22
  tic.setStepMode(TicStepMode::Microstep16);
  tic.setMaxSpeed(2000000);            // 200 microsteps/s
  tic.setMaxAccel(40000);
  tic.haltAndSetPosition(0);
  tic.energize();
  tic.exitSafeStart();

  Serial.println("READY ESP32 door controller (serial mode)");
}

String rx;

void loop() {
  // Read line-based commands from USB serial.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx.length()) { handleLine(rx); rx = ""; }
    } else {
      rx += c;
      if (rx.length() > 120) rx = "";   // overflow guard
    }
  }
  tic.resetCommandTimeout();            // keep Tic safety timeout from tripping
}
