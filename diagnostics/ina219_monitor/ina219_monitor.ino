/*
 * ESP32 + INA219 实时电流/电压监测
 * 库依赖：Adafruit INA219、Adafruit BusIO（Arduino 库管理器安装）
 *
 * 接线：
 *   INA219 Vcc -> ESP32 3V3
 *   INA219 GND -> ESP32 GND
 *   INA219 SDA -> ESP32 GPIO21
 *   INA219 SCL -> ESP32 GPIO22
 *   Vin+ -> 电源正极，Vin- -> 舵机电源正极（串联在供电回路中）
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

// 默认 I2C 地址 0x40（若改了 A0/A1 焊盘，这里改成对应地址）
Adafruit_INA219 ina219(0x40);

// ESP32 默认 I2C 引脚
#define SDA_PIN 21
#define SCL_PIN 22

// 电流方向修正：接线正确时保持 0（正值=耗电）。
// 若哪天把绿色端子两根线对调、读数变负了，改成 1 取回正值。
#define INVERT_CURRENT 0

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!ina219.begin()) {
    Serial.println("找不到 INA219，请检查接线和 I2C 地址！");
    while (1) { delay(10); }
  }

  // 可选量程设置：
  // 默认 32V / 2A，分辨率较低
  // ina219.setCalibration_32V_1A();   // 32V / 1A，分辨率更高
  // ina219.setCalibration_16V_400mA(); // 16V / 400mA，小电流最精确

  Serial.println("INA219 就绪，开始测量...");
  Serial.println("总线电压(V)\t分流电压(mV)\t负载电压(V)\t电流(mA)\t功率(mW)");
}

void loop() {
  float shuntVoltage_mV = ina219.getShuntVoltage_mV();  // 分流电阻两端压降
  float busVoltage_V    = ina219.getBusVoltage_V();     // Vin- 到 GND 的电压
  float current_mA      = ina219.getCurrent_mA();       // 流过负载的电流
  float power_mW        = ina219.getPower_mW();          // 功率（始终为正）

#if INVERT_CURRENT
  current_mA = -current_mA;                              // 修正方向，耗电显示为正
#endif

  // 负载实际电压 = 总线电压 + 分流压降（单位换算）
  float loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000.0);

  Serial.print(busVoltage_V, 3);      Serial.print("\t\t");
  Serial.print(shuntVoltage_mV, 3);   Serial.print("\t\t");
  Serial.print(loadVoltage_V, 3);     Serial.print("\t\t");
  Serial.print(current_mA, 2);        Serial.print("\t\t");
  Serial.print(power_mW, 1);          Serial.println();

  delay(500);  // 每 0.5 秒读一次
}
