#include <Wire.h>
const uint8_t BMI160_ADDR = 0x68; // change to 0x69 if SDO=3V3

uint8_t i2cRead8(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, (uint8_t)1);
  return Wire.read();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(100);

  uint8_t chip_id = i2cRead8(0x00); // CHIP_ID register
  Serial.print("BMI160 CHIP_ID: 0x"); Serial.println(chip_id, HEX);
  if (chip_id == 0xD1) Serial.println("✅ BMI160 detected!");
  else Serial.println("❌ Unexpected ID. Check wiring/address.");
}

void loop() {}
