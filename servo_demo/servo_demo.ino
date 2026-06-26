/*
 * ESP32 + two RDS5180 digital servos — slow / small-angle USB-serial demo.
 *
 * No Wi-Fi, no hotspot. The Mac talks to the board over USB (CP2102,
 * 115200 baud) with a simple line-based text protocol. A desktop GUI
 * (host_gui/servo_demo_gui.py) sends commands and shows live positions.
 *
 * Two servos are driven independently:
 *     motor 0 -> GPIO18
 *     motor 1 -> GPIO19
 *
 * The firmware does the SMOOTH motion: you give each motor a target angle
 * and a speed limit (deg/sec). Each loop it eases that motor's live position
 * toward its target by at most speed*dt, so the servo crawls instead of
 * snapping.
 *
 * Protocol (one command per line, '\n' terminated). <id> is 0 or 1.
 * -----------------------------------------------------------------
 *   Host -> board:
 *     PING                 -> "PONG"
 *     MOVE <id> <deg>      -> set that motor's target angle (eased toward)
 *     JOG  <id> <ddeg>     -> nudge that motor's target by a relative amount
 *     SPEED <id> <dps>     -> max speed deg/sec for that motor (1..120)
 *     CENTER [id]          -> center one motor, or both if id omitted
 *     STOP [id]            -> freeze one motor, or both if id omitted
 *     LIMITS <id> <lo> <hi>-> soft min/max angle for that motor (0..180)
 *     STATUS               -> one STATUS line covering both motors
 *
 *   Board -> host:
 *     STATUS online=1 \
 *       m0_pos=.. m0_target=.. m0_speed=.. m0_min=.. m0_max=.. m0_moving=.. \
 *       m1_pos=.. m1_target=.. m1_speed=.. m1_min=.. m1_max=.. m1_moving=..
 *     OK <echoed-command> | ERR <reason> | PONG | READY ...
 *
 * Wiring
 * ------
 *   Servo 0 signal -> GPIO18,  Servo 1 signal -> GPIO19
 *   Servo V+       -> external 6.0-8.4 V supply (NOT 3V3/5V from ESP32)
 *   Servo GND      -> common ground with the ESP32 (tie supply GND to ESP32 GND)
 *   ESP32 powered over USB.
 *
 * NOTE: the Tic T249 stepper is NOT wired to this board. It connects directly
 * to the computer over its own USB cable and is driven by the host GUI via the
 * ticcmd CLI — this firmware only handles the two servos.
 *
 * Library: "ESP32Servo".
 */

#include <ESP32Servo.h>

// ----- Pins (one per motor) -----
const int NUM_MOTORS = 2;
const int SERVO_PINS[NUM_MOTORS] = {18, 19};

// ----- Limit switch -----
// Wired to GPIO17 with the internal pull-up enabled: the switch shorts the
// pin to GND when pressed, so digitalRead == LOW means TRIGGERED.
const int LIMIT_SWITCH_PIN = 17;

// ----- Servo pulse calibration (microseconds) -----
// 500-2500us spans the full mechanical range. The RDS5180SG is a 0-270 deg
// position servo (it can be turned within 360 deg only when powered OFF), so
// the powered travel limit is 270 degrees.
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2500;
const float ANGLE_RANGE_DEG = 270.0f;   // full mechanical range, powered

// ----- Per-motor state -----
struct Motor {
  Servo servo;
  float limLo  = 0.0f;
  float limHi  = 270.0f;
  float curPos = 135.0f;    // live commanded position (deg) — mid of 0..270
  float target = 135.0f;    // where we ease toward (deg)
  float speed  = 8.0f;      // max slew rate (deg/sec) — start slow
};
Motor motors[NUM_MOTORS];

unsigned long lastUs = 0;

// ---------------------------------------------------------------------------
float clampAngle(const Motor& m, float d) {
  if (d < m.limLo) return m.limLo;
  if (d > m.limHi) return m.limHi;
  return d;
}

void writeAngle(Motor& m, float deg) {
  int us = (int)lround(PULSE_MIN_US +
                       (PULSE_MAX_US - PULSE_MIN_US) * (deg / ANGLE_RANGE_DEG));
  m.servo.writeMicroseconds(us);
}

bool validId(const String& s, int& id) {
  if (s.length() == 0) return false;
  id = s.toInt();
  return (id >= 0 && id < NUM_MOTORS);
}

void sendStatus() {
  char line[320];
  int n = snprintf(line, sizeof(line), "STATUS online=1");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Motor& m = motors[i];
    bool moving = fabs(m.target - m.curPos) > 0.05f;
    n += snprintf(line + n, sizeof(line) - n,
      " m%d_pos=%.2f m%d_target=%.2f m%d_speed=%.1f m%d_min=%.1f m%d_max=%.1f m%d_moving=%d",
      i, m.curPos, i, m.target, i, m.speed, i, m.limLo, i, m.limHi, i, moving ? 1 : 0);
  }
  n += snprintf(line + n, sizeof(line) - n, " limit=%d",
                (digitalRead(LIMIT_SWITCH_PIN) == LOW) ? 1 : 0);
  Serial.println(line);
}

// ---------------------------------------------------------------------------
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Tokenize up to three fields: CMD, arg1, arg2
  int s1 = line.indexOf(' ');
  String cmd  = (s1 < 0) ? line : line.substring(0, s1);
  String rest = (s1 < 0) ? ""   : line.substring(s1 + 1);
  rest.trim();
  int s2 = rest.indexOf(' ');
  String a1 = (s2 < 0) ? rest : rest.substring(0, s2);
  String a2 = (s2 < 0) ? ""   : rest.substring(s2 + 1);
  a1.trim(); a2.trim();
  cmd.toUpperCase();

  int id;
  if (cmd == "PING") {
    Serial.println("PONG");
  } else if (cmd == "MOVE") {
    if (!validId(a1, id)) { Serial.println("ERR bad_motor_id"); return; }
    motors[id].target = clampAngle(motors[id], a2.toFloat());
    Serial.print("OK MOVE "); Serial.print(id); Serial.print(" ");
    Serial.println(motors[id].target, 2);
  } else if (cmd == "JOG") {
    if (!validId(a1, id)) { Serial.println("ERR bad_motor_id"); return; }
    motors[id].target = clampAngle(motors[id], motors[id].target + a2.toFloat());
    Serial.print("OK JOG "); Serial.print(id); Serial.print(" ");
    Serial.println(motors[id].target, 2);
  } else if (cmd == "SPEED") {
    if (!validId(a1, id)) { Serial.println("ERR bad_motor_id"); return; }
    float v = a2.toFloat();
    if (v < 1.0f) v = 1.0f;
    if (v > 120.0f) v = 120.0f;
    motors[id].speed = v;
    Serial.print("OK SPEED "); Serial.print(id); Serial.print(" ");
    Serial.println(motors[id].speed, 1);
  } else if (cmd == "CENTER") {
    if (a1.length() == 0) {                 // both
      for (int i = 0; i < NUM_MOTORS; i++)
        motors[i].target = clampAngle(motors[i], (motors[i].limLo + motors[i].limHi) * 0.5f);
      Serial.println("OK CENTER all");
    } else if (validId(a1, id)) {
      motors[id].target = clampAngle(motors[id], (motors[id].limLo + motors[id].limHi) * 0.5f);
      Serial.print("OK CENTER "); Serial.println(id);
    } else { Serial.println("ERR bad_motor_id"); }
  } else if (cmd == "STOP") {
    if (a1.length() == 0) {                 // both
      for (int i = 0; i < NUM_MOTORS; i++) motors[i].target = motors[i].curPos;
      Serial.println("OK STOP all");
    } else if (validId(a1, id)) {
      motors[id].target = motors[id].curPos;
      Serial.print("OK STOP "); Serial.println(id);
    } else { Serial.println("ERR bad_motor_id"); }
  } else if (cmd == "LIMITS") {
    if (!validId(a1, id)) { Serial.println("ERR bad_motor_id"); return; }
    int s3 = a2.indexOf(' ');
    if (s3 < 0) { Serial.println("ERR limits_need_two_values"); return; }
    float lo = a2.substring(0, s3).toFloat();
    float hi = a2.substring(s3 + 1).toFloat();
    if (hi <= lo) { Serial.println("ERR limits_hi_le_lo"); return; }
    motors[id].limLo = constrain(lo, 0.0f, ANGLE_RANGE_DEG);
    motors[id].limHi = constrain(hi, 0.0f, ANGLE_RANGE_DEG);
    motors[id].target = clampAngle(motors[id], motors[id].target);
    motors[id].curPos = clampAngle(motors[id], motors[id].curPos);
    Serial.print("OK LIMITS "); Serial.print(id); Serial.print(" ");
    Serial.print(motors[id].limLo, 1); Serial.print(" ");
    Serial.println(motors[id].limHi, 1);
  } else if (cmd == "STATUS") {
    sendStatus();
  } else {
    Serial.print("ERR unknown_command "); Serial.println(cmd);
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // ESP32Servo needs PWM timers allocated for multiple servos.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);   // limit switch on GPIO17 (active-low)

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].servo.setPeriodHertz(50);     // standard 50 Hz servo frame
    int ch = motors[i].servo.attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    Serial.print("ATTACH m"); Serial.print(i);
    Serial.print(" pin="); Serial.print(SERVO_PINS[i]);
    Serial.print(" ch="); Serial.print(ch);
    Serial.println(ch ? " OK" : " FAIL");
    writeAngle(motors[i], motors[i].curPos);
  }

  lastUs = micros();
  Serial.println("READY ESP32 dual servo (serial mode)");
}

void loop() {
  // 1) read line-based commands from USB serial
  static String rx;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx.length()) { handleLine(rx); rx = ""; }
    } else {
      rx += c;
      if (rx.length() > 120) rx = "";       // overflow guard
    }
  }

  // 2) ease each motor's curPos toward its target at its speed limit
  unsigned long now = micros();
  float dt = (now - lastUs) / 1e6f;          // seconds since last update
  lastUs = now;
  if (dt > 0.1f) dt = 0.1f;                   // ignore long stalls

  for (int i = 0; i < NUM_MOTORS; i++) {
    Motor& m = motors[i];
    float maxStep = m.speed * dt;             // most this motor may move now
    float diff = m.target - m.curPos;
    if (fabs(diff) <= maxStep) m.curPos = m.target;
    else                       m.curPos += (diff > 0 ? maxStep : -maxStep);
    writeAngle(m, m.curPos);
  }

  delay(10);                                  // ~100 Hz motion update
}
