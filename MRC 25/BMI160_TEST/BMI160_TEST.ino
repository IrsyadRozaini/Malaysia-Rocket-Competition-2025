#include <Wire.h>
#include "DFRobot_BMI160.h"
DFRobot_BMI160 bmi160;

void setup(){
  Serial.begin(115200);
  Wire.begin(21,22);

  if(bmi160.softReset()!=BMI160_OK) { 
    Serial.println("BMI160 reset failed"); 
    while(1); 
  }

  if(bmi160.I2cInit(0x68)!=BMI160_OK) { 
    Serial.println("BMI160 init failed"); 
    while(1); 
  }

  Serial.println("BMI160 init successful!");
}

void loop(){
  int16_t accel[3], gyro[3];
  bmi160.getAccelData(accel);
  bmi160.getGyroData(gyro);

  Serial.print("AX: "); Serial.print(accel[0]);
  Serial.print(" AY: "); Serial.print(accel[1]);
  Serial.print(" AZ: "); Serial.print(accel[2]);
  Serial.print(" | GX: "); Serial.print(gyro[0]);
  Serial.print(" GY: "); Serial.print(gyro[1]);
  Serial.print(" GZ: "); Serial.println(gyro[2]);

  delay(200);
}
