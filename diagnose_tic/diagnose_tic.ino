/*
 * Tic T249 diagnostic over I2C (ESP32).
 * Scans the I2C bus, prints Tic VIN, operation state, and error flags,
 * then attempts to energize + nudge the motor so we can see WHY it
 * is (or isn't) moving. Read the serial monitor at 115200 baud.
 */
#include <Wire.h>
#include <Tic.h>

TicI2C tic;  // default address 14 (0x0E)

const long FULL_STEPS_PER_REV = 200;
const long MICROSTEPS         = 16;
const float GEARBOX_RATIO     = 5.0f;
const float USTEPS_PER_OUT_DEG =
    (FULL_STEPS_PER_REV * MICROSTEPS * GEARBOX_RATIO) / 360.0f;

void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X (%d)\n", a, a);
      found++;
    }
  }
  if (!found) Serial.println("  NONE FOUND  -> check SDA/SCL/GND wiring & Tic power");
}

void printOpState(TicOperationState s) {
  Serial.print("Operation state: ");
  switch (s) {
    case TicOperationState::Reset:          Serial.println("Reset"); break;
    case TicOperationState::Deenergized:    Serial.println("Deenergized"); break;
    case TicOperationState::SoftError:      Serial.println("SoftError"); break;
    case TicOperationState::WaitingForErrLine: Serial.println("WaitingForErrLineDeassert"); break;
    case TicOperationState::StartingUp:     Serial.println("StartingUp"); break;
    case TicOperationState::Normal:         Serial.println("Normal"); break;
    default:                                Serial.println("Unknown"); break;
  }
}

void printErrors(uint16_t e) {
  Serial.printf("Error status bits: 0x%04X\n", e);
  if (e == 0) { Serial.println("  (no active errors)"); return; }
  if (e & (1<<0)) Serial.println("  - Intentionally de-energized");
  if (e & (1<<1)) Serial.println("  - Motor driver error (check VIN/current limit/wiring)");
  if (e & (1<<2)) Serial.println("  - Low VIN (motor supply too low/absent)");
  if (e & (1<<3)) Serial.println("  - Kill switch active");
  if (e & (1<<4)) Serial.println("  - Required input invalid");
  if (e & (1<<5)) Serial.println("  - Serial error");
  if (e & (1<<6)) Serial.println("  - Command timeout");
  if (e & (1<<7)) Serial.println("  - Safe start violation");
  if (e & (1<<8)) Serial.println("  - ERR line high");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== Tic T249 diagnostic ===");
  Wire.begin();           // SDA=21, SCL=22
  delay(100);
  scanI2C();

  Serial.printf("VIN: %u mV\n", tic.getVinVoltage());
  printOpState(tic.getOperationState());
  printErrors(tic.getErrorStatus());

  Serial.println("\nAttempting energize + exitSafeStart + small move...");
  tic.setStepMode(TicStepMode::Microstep16);
  tic.setMaxSpeed(2000000);
  tic.setMaxAccel(40000);
  tic.haltAndSetPosition(0);
  tic.energize();
  tic.exitSafeStart();
  long target = lround(10 * USTEPS_PER_OUT_DEG);  // ~10 deg output
  tic.setTargetPosition(target);
  Serial.printf("Commanded target = %ld microsteps (~10 deg output)\n", target);
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 1000) {
    t = millis();
    tic.resetCommandTimeout();
    Serial.printf("pos=%ld target=%ld  VIN=%umV  ",
                  tic.getCurrentPosition(), tic.getTargetPosition(),
                  tic.getVinVoltage());
    printOpState(tic.getOperationState());
    uint16_t e = tic.getErrorStatus();
    if (e) printErrors(e);
  }
}
