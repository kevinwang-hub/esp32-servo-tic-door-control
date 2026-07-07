/*
 * INA219 诊断程序：扫描 I2C + 读取原始寄存器，判断芯片状态
 * 不驱动舵机，安全运行
 */
#include <Wire.h>
#include <Adafruit_INA219.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_INA219 ina219(0x40);

void scanI2C() {
  Serial.println("=== I2C 扫描 ===");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  发现设备地址: 0x");
      Serial.println(a, HEX);
      n++;
    }
  }
  if (n == 0) Serial.println("  没找到任何 I2C 设备！");
  Serial.println();
}

// 直接读 INA219 的 16 位寄存器
uint16_t readReg(uint8_t reg) {
  Wire.beginTransmission(0x40);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
  uint16_t v = Wire.read() << 8;
  v |= Wire.read();
  return v;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);

  scanI2C();

  if (!ina219.begin()) {
    Serial.println("ina219.begin() 失败 —— 芯片没响应");
  } else {
    Serial.println("ina219.begin() 成功");
  }
  Serial.println();
}

void loop() {
  uint16_t cfg   = readReg(0x00);  // Config
  uint16_t shunt = readReg(0x01);  // Shunt voltage (raw)
  uint16_t bus   = readReg(0x02);  // Bus voltage (raw)
  uint16_t calib = readReg(0x05);  // Calibration

  Serial.print("Config=0x");   Serial.print(cfg, HEX);
  Serial.print("  ShuntRaw=0x"); Serial.print(shunt, HEX);
  Serial.print("  BusRaw=0x");   Serial.print(bus, HEX);
  Serial.print("  Calib=0x");    Serial.print(calib, HEX);

  // BusRaw 高 13 位是电压，每 LSB = 4mV
  float busV = (bus >> 3) * 0.004;
  Serial.print("   -> 解析总线电压=");
  Serial.print(busV, 3);
  Serial.println("V");

  // 库函数读数
  Serial.print("   库读数: Bus=");
  Serial.print(ina219.getBusVoltage_V(), 3);
  Serial.print("V  Shunt=");
  Serial.print(ina219.getShuntVoltage_mV(), 3);
  Serial.print("mV  Current=");
  Serial.print(ina219.getCurrent_mA(), 2);
  Serial.println("mA");
  Serial.println();

  delay(1000);
}
