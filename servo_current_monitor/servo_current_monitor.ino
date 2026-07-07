/*
 * ESP32 + INA219 + dual-servo door control system
 *   #0 = door   D18 (range 55 deg open ~ 220 deg close-command, snugs ~215-217)
 *   #1 = handle D19 (0 = unlock, 125 = lock)
 *
 * Serial commands (115200):
 *   "<ch> <angle>"    manual move (door limited 55~220; rejected during seq/estop)
 *   open / close / home    sequences
 *   stop / start           emergency stop / resume (reset == start)
 *   speed <n>              deg per second (5~400)
 *   u <ch> <us>, min/max/cal   pulse calibration
 *   trip                   test emergency stop
 *
 * E-stop rules (monitored while attached): instant >2000mA, or >1000mA sustained 5s
 *   - during close sequence door motion => treated as "door snug", freeze & continue
 *   - otherwise => e-stop: freeze all + abort sequence, 'start' to resume
 *   - still high current after e-stop => escalate to detach (power-off PWM)
 *
 * Wiring: INA219 SDA=21 SCL=22 Vcc=3V3; PSU OUT+ -> Vin+, Vin- -> servo red; common GND
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ESP32Servo.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define SERVO_DOOR_PIN   18
#define SERVO_HANDLE_PIN 19
#define DOOR_SWITCH_PIN  17   // door-closed microswitch, INPUT_PULLUP
// 实测极性: 不按 = LOW, 按下 = HIGH (开关走的是常闭NC触点到GND)
// 注意: 若开关线脱落, 引脚被上拉读HIGH会误判"按下"; 建议以后改接 COM+NO (按下=LOW 更安全)
#define SWITCH_PRESSED_LEVEL HIGH

// 270-degree servo, pulse calibration
#define SERVO_MAX_DEG 270
int servoMinUs = 500;
int servoMaxUs = 2500;

// Door range and sequence positions
#define DOOR_MIN_DEG   55    // open limit
#define DOOR_MAX_DEG   220   // close command position
#define DOOR_OPEN_DEG  55
#define DOOR_HOME_DEG  90
#define DOOR_CLOSE_DEG 220
#define HANDLE_UNLOCK  0
#define HANDLE_LOCK    125

// E-stop rules
#define I_INSTANT_MA   2000.0f
#define I_SUSTAIN_MA   1000.0f
#define I_SUSTAIN_MS   5000UL
#define I_RELIEF_MA    800.0f    // pressure-relief target after snug
#define BOOT_GRACE_MS  2000UL    // protection grace after boot/resume

Adafruit_INA219 ina219(0x40);
Servo servoDoor, servoHandle;

// Motion state (boot: door=190 near closed, handle=0 unlocked)
float curDoor = 190, curHandle = 0;
int   tgtDoor = 190, tgtHandle = 0;
float servoSpeedDps = 30.0;   // slow default; 'speed <n>' to change

// E-stop state
bool stopped = false;
bool detachedFlag = false;
String stopReason = "";
unsigned long protArmedAt = 0;

// 舵机上电状态: 开机不驱动(防复位猛跳), 第一个命令/序列时才 attach
bool servosEngaged = false;

// Sequence state
enum SeqType { SEQ_NONE = 0, SEQ_HOME, SEQ_OPEN, SEQ_CLOSE };
SeqType seq = SEQ_NONE;
int seqStep = 0;

// Current monitoring
float lastI = 0;
unsigned long hiSince = 0;

// Door-closed switch (debounced) + close-sequence bookkeeping
bool swPressed = false;
unsigned long closeWaitStart = 0;
int closeReliefSteps = 0;
#define CLOSE_SWITCH_TIMEOUT_MS 60000UL   // waiting for switch longer than this -> e-stop

const char* seqName() {
  return seq == SEQ_HOME ? "home" : seq == SEQ_OPEN ? "open" :
         seq == SEQ_CLOSE ? "close" : "";
}

int angleToUs(float a) {
  return servoMinUs + (int)((a / SERVO_MAX_DEG) * (servoMaxUs - servoMinUs) + 0.5f);
}

void stepOne(Servo &sv, float &cur, int tgt, float dt) {
  if (fabs(cur - tgt) < 0.01f) return;
  float st = servoSpeedDps * dt;
  if (cur < tgt) cur = min((float)tgt, cur + st);
  else           cur = max((float)tgt, cur - st);
  sv.writeMicroseconds(angleToUs(cur));
}

bool doorDone()   { return fabs(curDoor - tgtDoor) < 0.5f; }
bool handleDone() { return fabs(curHandle - tgtHandle) < 0.5f; }

void freezeAll() {
  tgtDoor = (int)(curDoor + 0.5f);
  tgtHandle = (int)(curHandle + 0.5f);
}

void doStop(const char* why) {
  freezeAll();
  seq = SEQ_NONE;
  stopped = true;
  stopReason = why;
  hiSince = 0;
  Serial.printf("!!! ESTOP: %s (current %.0fmA). Send 'start' to resume\n", why, fabs(lastI));
}

void engageServos() {
  if (servosEngaged && !detachedFlag) return;
  servoDoor.attach(SERVO_DOOR_PIN, 500, 2500);
  servoHandle.attach(SERVO_HANDLE_PIN, 500, 2500);
  servoDoor.writeMicroseconds(angleToUs(curDoor));
  servoHandle.writeMicroseconds(angleToUs(curHandle));
  servosEngaged = true;
  detachedFlag = false;
  protArmedAt = millis() + BOOT_GRACE_MS;
  Serial.println(">> Servos engaged");
}

void moveServo(int ch, int angle) {
  if (stopped) { Serial.println("!! E-stopped, send 'start' first"); return; }
  if (seq != SEQ_NONE) { Serial.printf("!! Sequence [%s] running, stop it or wait\n", seqName()); return; }
  engageServos();
  if (ch == 0) {
    angle = constrain(angle, DOOR_MIN_DEG, DOOR_MAX_DEG);
    tgtDoor = angle;
    Serial.printf(">> Door -> %d deg (limits %d~%d, %.0f deg/s)\n",
                  angle, DOOR_MIN_DEG, DOOR_MAX_DEG, servoSpeedDps);
  } else if (ch == 1) {
    angle = constrain(angle, 0, SERVO_MAX_DEG);
    tgtHandle = angle;
    Serial.printf(">> Handle -> %d deg (%.0f deg/s)\n", angle, servoSpeedDps);
  } else {
    Serial.println(">> Invalid channel, use 0 or 1");
  }
}

void startSeq(SeqType t) {
  if (stopped) { Serial.println("!! E-stopped, send 'start' first"); return; }
  engageServos();
  seq = t;
  seqStep = 0;
  hiSince = 0;
  Serial.printf(">> Sequence start: %s\n", seqName());
}

// ---- sequence state machine ----
void tickSeq(unsigned long now) {
  if (seq == SEQ_NONE) return;
  switch (seq) {
    case SEQ_HOME:   // home: handle->0 unlock, then door->90
      if (seqStep == 0) { tgtHandle = HANDLE_UNLOCK; seqStep = 1; Serial.println(">> [home] handle -> 0 (unlock)"); }
      else if (seqStep == 1 && handleDone()) { tgtDoor = DOOR_HOME_DEG; seqStep = 2; Serial.println(">> [home] door -> 90"); }
      else if (seqStep == 2 && doorDone()) { seq = SEQ_NONE; Serial.println(">> Home complete (handle=0, door=90)"); }
      break;

    case SEQ_OPEN:   // open: home first (handle->0, door->90), then door->55
      if (seqStep == 0) { tgtHandle = HANDLE_UNLOCK; seqStep = 1; Serial.println(">> [open] handle -> 0 (unlock)"); }
      else if (seqStep == 1 && handleDone()) { tgtDoor = DOOR_HOME_DEG; seqStep = 2; Serial.println(">> [open] door -> 90"); }
      else if (seqStep == 2 && doorDone()) { tgtDoor = DOOR_OPEN_DEG; seqStep = 3; Serial.println(">> [open] door -> 55"); }
      else if (seqStep == 3 && doorDone()) { seq = SEQ_NONE; Serial.println(">> Open complete (door=55)"); }
      break;

    case SEQ_CLOSE:  // close: door->220 (stop on resistance), wait for D17 switch, handle->125 lock, relieve
      if (seqStep == 0) {
        tgtDoor = DOOR_CLOSE_DEG; seqStep = 1;
        Serial.println(">> [close] door -> 220 (stop on resistance = snug)");
      }
      else if (seqStep == 1) {
        // 第1步只管合门(电流规则会把它停住)，此阶段不看开关
        if (doorDone()) {                   // reached 220 without resistance
          Serial.println(">> [close] door reached 220 (no resistance detected)");
          seqStep = 2; closeWaitStart = now; hiSince = 0;
          Serial.println(">> [close] now monitoring door switch (D17)...");
        }
        // snug via current rule handled in checkCurrent()
      }
      else if (seqStep == 2) {              // hold pressure, wait for door switch
        if (swPressed) {
          tgtHandle = HANDLE_LOCK; seqStep = 3;
          Serial.printf(">> [close] switch pressed -> locking: handle -> %d\n", HANDLE_LOCK);
        } else if (now - closeWaitStart > CLOSE_SWITCH_TIMEOUT_MS) {
          doStop("close timeout: door switch not pressed");
        }
      }
      else if (seqStep == 3 && handleDone()) {
        seqStep = 4; closeReliefSteps = 0;
        Serial.println(">> [close] locked, relieving door servo pressure...");
      }
      else if (seqStep == 4) {              // post-lock pressure relief (lock now holds the door)
        static unsigned long lastRelief = 0;
        if (fabs(lastI) > I_RELIEF_MA && closeReliefSteps < 20 &&
            now - lastRelief >= 200 && curDoor > DOOR_MIN_DEG) {
          lastRelief = now;
          curDoor -= 1.0f;
          tgtDoor = (int)(curDoor + 0.5f);
          servoDoor.writeMicroseconds(angleToUs(curDoor));
          closeReliefSteps++;
        } else if (fabs(lastI) <= I_RELIEF_MA || closeReliefSteps >= 20) {
          seq = SEQ_NONE;
          Serial.printf(">> Close complete, locked (handle=%d, door held at %d)\n",
                        HANDLE_LOCK, (int)(curDoor + 0.5f));
        }
      }
      break;

    default: break;
  }
}

// ---- e-stop rules ----
void checkCurrent(unsigned long now) {
  if (!servosEngaged || detachedFlag || now < protArmedAt) { hiSince = 0; return; }
  bool closingDoor = (seq == SEQ_CLOSE && seqStep == 1);  // rule doubles as snug detection
  // close steps 2/3/4 (pressing+waiting / locking handle / relieving): door holding
  // pressure makes high current NORMAL here — never e-stop, let the handle move freely
  bool exemptPhase = (seq == SEQ_CLOSE && seqStep >= 2);
  if (exemptPhase) { hiSince = 0; return; }

  float a = fabs(lastI);
  bool instant = a > I_INSTANT_MA;
  if (a > I_SUSTAIN_MA) { if (!hiSince) hiSince = now; }
  else hiSince = 0;
  bool sustained = hiSince && (now - hiSince >= I_SUSTAIN_MS);
  if (!(instant || sustained)) return;
  hiSince = 0;

  const char* why = instant ? "instant>2000mA" : "sustained>1000mA/5s";

  if (stopped) {           // still high after e-stop -> escalate to detach
    servoDoor.detach();
    servoHandle.detach();
    detachedFlag = true;
    servosEngaged = false;
    Serial.printf("!!! Current still high after ESTOP (%s), servos detached. Send 'start' to re-power\n", why);
    return;
  }
  if (closingDoor) {       // resistance while closing = snug
    tgtDoor = (int)(curDoor + 0.5f);
    Serial.printf(">> [close] door snug (%s, %.0fmA), stopped at ~%d deg\n", why, a, tgtDoor);
    seqStep = 2;
    closeWaitStart = now;
    Serial.println(">> [close] waiting for door switch (D17)...");
    return;
  }
  doStop(why);
}

// ---- serial commands ----
void parseCommand(String cmd) {
  int sp = cmd.indexOf(' ');
  String tok  = (sp > 0) ? cmd.substring(0, sp) : cmd;
  String rest = (sp > 0) ? cmd.substring(sp + 1) : "";
  rest.trim();

  if (tok == "open")       startSeq(SEQ_OPEN);
  else if (tok == "close") startSeq(SEQ_CLOSE);
  else if (tok == "home")  startSeq(SEQ_HOME);
  else if (tok == "stop") {
    doStop("manual STOP");
  }
  else if (tok == "start" || tok == "reset") {
    stopped = false;
    stopReason = "";
    hiSince = 0;
    engageServos();
    protArmedAt = millis() + BOOT_GRACE_MS;
    Serial.println(">> Resumed (START). Servos hold current position");
  }
  else if (tok == "trip") {
    doStop("test trip");
  }
  else if (tok == "u") {            // raw pulse: u <ch> <us>
    int s2 = rest.indexOf(' ');
    if (s2 > 0) {
      if (stopped) { Serial.println("!! E-stopped, send 'start' first"); return; }
      if (seq != SEQ_NONE) { Serial.println("!! Sequence running"); return; }
      engageServos();
      int ch = rest.substring(0, s2).toInt();
      int us = constrain(rest.substring(s2 + 1).toInt(), 500, 2500);
      if (ch == 0) us = constrain(us, angleToUs(DOOR_MIN_DEG), angleToUs(DOOR_MAX_DEG));
      float ang = (float)(us - servoMinUs) * SERVO_MAX_DEG / (servoMaxUs - servoMinUs);
      if (ch == 0)      { servoDoor.writeMicroseconds(us);   curDoor = ang;   tgtDoor = (int)ang; }
      else if (ch == 1) { servoHandle.writeMicroseconds(us); curHandle = ang; tgtHandle = (int)ang; }
      Serial.printf(">> Raw pulse: ch%d = %d us\n", ch, us);
    }
  }
  else if (tok == "min") {
    servoMinUs = constrain(rest.toInt(), 500, 2500);
    Serial.printf(">> servoMinUs(0 deg) = %d us\n", servoMinUs);
  }
  else if (tok == "max") {
    servoMaxUs = constrain(rest.toInt(), 500, 2500);
    Serial.printf(">> servoMaxUs(270 deg) = %d us\n", servoMaxUs);
  }
  else if (tok == "speed") {
    servoSpeedDps = constrain(rest.toFloat(), 5.0f, 400.0f);
    Serial.printf(">> Speed = %.0f deg/s\n", servoSpeedDps);
  }
  else if (tok == "cal") {
    Serial.printf(">> Cal: 0deg=%dus 270deg=%dus speed=%.0fdeg/s door limits %d~%d lock=%d\n",
                  servoMinUs, servoMaxUs, servoSpeedDps, DOOR_MIN_DEG, DOOR_MAX_DEG, HANDLE_LOCK);
  }
  else {                            // default: <ch> <angle>
    if (sp > 0) moveServo(tok.toInt(), rest.toInt());
    else Serial.println(">> Commands: open/close/home/stop/start | '<ch> <angle>' | speed <n> | trip");
  }
}

void handleSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      if (buf.length() > 0) parseCommand(buf);
      buf = "";
    } else {
      buf += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(DOOR_SWITCH_PIN, INPUT_PULLUP);  // pressed = LOW

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ina219.begin()) {
    Serial.println("INA219 not found, check wiring!");
    while (1) { delay(10); }
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoDoor.setPeriodHertz(50);
  servoHandle.setPeriodHertz(50);

  // 开机不驱动舵机(防复位猛跳): 第一个命令/序列/start 时才 attach 并定位
  protArmedAt = millis() + BOOT_GRACE_MS;

  Serial.println("Door control ready (servos idle until first command).");
  Serial.println("Commands: open/close/home/stop/start | '<ch> <angle>' | speed <n>");
}

void loop() {
  handleSerial();
  unsigned long now = millis();

  // current sampling + e-stop rules: every 50ms
  static unsigned long lastSense = 0;
  if (now - lastSense >= 50) {
    lastSense = now;
    lastI = ina219.getCurrent_mA();
    // door switch debounce: two consecutive identical 50ms samples
    static bool lastRaw = false;
    bool raw = (digitalRead(DOOR_SWITCH_PIN) == SWITCH_PRESSED_LEVEL);
    if (raw == lastRaw) swPressed = raw;
    lastRaw = raw;
    checkCurrent(now);
  }

  // sequence state machine
  tickSeq(now);

  // smooth motion: every 20ms (frozen during e-stop/detach)
  static unsigned long lastStep = 0;
  if (now - lastStep >= 20) {
    float dt = (now - lastStep) / 1000.0f;
    lastStep = now;
    if (!stopped && !detachedFlag && servosEngaged) {
      stepOne(servoDoor,   curDoor,   tgtDoor,   dt);
      stepOne(servoHandle, curHandle, tgtHandle, dt);
    }
  }

  // telemetry: every 150ms
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 150) {
    lastPrint = now;
    float busV  = ina219.getBusVoltage_V();
    float power = ina219.getPower_mW();
    char suffix[64] = "";
    if (stopped) {
      snprintf(suffix, sizeof(suffix), "  !!ESTOP:%s%s!!",
               stopReason.c_str(), detachedFlag ? "(detached)" : "");
    } else if (seq != SEQ_NONE) {
      snprintf(suffix, sizeof(suffix), "  {seq:%s:%d}", seqName(), seqStep);
    } else if (!servosEngaged) {
      snprintf(suffix, sizeof(suffix), "  {servos:idle}");
    }
    Serial.printf("V=%.2fV  I=%.1fmA  P=%.0fmW   [door=%d  handle=%d  sw=%d]%s\n",
                  busV, lastI, power, (int)(curDoor + 0.5f), (int)(curHandle + 0.5f),
                  swPressed ? 1 : 0, suffix);
  }
}
