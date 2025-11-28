#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_BMI160.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SD.h>

File dataFile;

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
Adafruit_BME280 bme;
DFRobot_BMI160 bmi160;


#define SS 5
#define RST 14
#define DIO0 26
#define SD_CS 13   // Adjust depending on your wiring

#define SEALEVELPRESSURE_HPA (1013.25)
#define G_SI 9.80665           // m/s² per g
#define DEG_TO_RAD 0.0174533   

void setup(){
   Serial.begin(115200);

    LoRa.setPins(SS, RST, DIO0);
      if (!LoRa.begin(433E6)) {   // adjust frequency
      Serial.println("LoRa init failed!");
      while (1);
  }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(17); 
    Serial.println("LoRa init OK");

    Wire.begin(21, 22);

      if (!bme.begin(0x76)) {   // or 0x77 depending on module
      Serial.println("BME280 not found!");
      while (1);
  }

       if (bmi160.softReset() != BMI160_OK) {
       Serial.println("BMI160 reset failed!");
       while (1);
  }
       if (bmi160.I2cInit(0x68) != BMI160_OK) {
       Serial.println("BMI160 not found!");
       while (1);
  }

    Serial.println("Sensors ready");

    GPSSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17 (adjust if needed)
    Serial.println("CanSat GPS + LoRa starting...");

      // --- SD Init ---
    if (!SD.begin(SD_CS)) {
      Serial.println("SD Card init failed!");
      while (1);
  }
    Serial.println("SD Card ready.");  

    dataFile = SD.open("/data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time(ms),Temp(K),Pressure(Pa),Humidity(%),Alt_BME(m),"
                     "AccelX(m/s2),AccelY(m/s2),AccelZ(m/s2),"
                     "GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),"
                     "Lat(deg),Lng(deg),Alt(m),Sats");
    dataFile.close();
  }
}

void loop(){
  sendTelemetry();
    delay(500);
}

void sendTelemetry() {
     // --- Read BME280 ---
   float t = bme.readTemperature();
   float p = bme.readPressure() / 100.0; // hPa
   float rh = bme.readHumidity();
   float alt_bme = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // --- Read BMI160 ---
  int16_t accel[3], gyro[3];
  bmi160.getAccelData(accel);
  bmi160.getGyroData(gyro);

    // Convert accel (g → m/s²)
  float ax = accel[0] / 16384.0 * G_SI;
  float ay = accel[1] / 16384.0 * G_SI;
  float az = accel[2] / 16384.0 * G_SI;

  // Convert gyro (°/s → rad/s)
  float gx = gyro[0] / 131.2 * DEG_TO_RAD;
  float gy = gyro[1] / 131.2 * DEG_TO_RAD;
  float gz = gyro[2] / 131.2 * DEG_TO_RAD;


  // --- Update GPS ---
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

   // --- Prepare LoRa frame ---
    String packet = "NOVA, ";
  packet += millis(); packet += ",";
  packet += String(t, 2) + ",";
  packet += String(p, 2) + ",";
  packet += String(rh, 1) + ",";
  packet += String(alt_bme, 2) + ",";
  packet += String(accel[0]) + ",";
  packet += String(accel[1]) + ",";
  packet += String(accel[2]) + ",";
  packet += String(gyro[0]) + ",";
  packet += String(gyro[1]) + ",";
  packet += String(gyro[2]) + ",";

  // GPS data (use "nan" if unavailable)
  if (gps.location.isValid()) {
    packet += String(gps.location.lat(), 6) + ",";
    packet += String(gps.location.lng(), 6) + ",";
  } else {
    packet += "nan,nan,";
  }

  if (gps.altitude.isValid()) {
    packet += String(gps.altitude.meters(), 2) + ",";
  } else {
    packet += "nan,";
  }

  if (gps.satellites.isValid()) {
    packet += String(gps.satellites.value());
  } else {
    packet += "0";
  }
 
  // --- Send packet ---
  Serial.println("TX: " + packet);
  digitalWrite(SD_CS, HIGH);  
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

    // --- Prepare CSV line for SD (matches header) ---
String csvLine = "";
csvLine += millis(); csvLine += ",";
csvLine += String(t, 2) + ",";
csvLine += String(p * 100.0, 2) + ",";
csvLine += String(rh, 1) + ",";
csvLine += String(alt_bme, 2) + ",";
csvLine += String(ax, 2) + ",";
csvLine += String(ay, 2) + ",";
csvLine += String(az, 2) + ",";
csvLine += String(gx, 2) + ",";
csvLine += String(gy, 2) + ",";
csvLine += String(gz, 2) + ",";
csvLine += (gps.location.isValid() ? String(gps.location.lat(), 6) : "nan"); csvLine += ",";
csvLine += (gps.location.isValid() ? String(gps.location.lng(), 6) : "nan"); csvLine += ",";
csvLine += (gps.altitude.isValid() ? String(gps.altitude.meters(), 2) : "nan"); csvLine += ",";
csvLine += (gps.satellites.isValid() ? String(gps.satellites.value()) : "0");

   // --- Save to SD ---
  digitalWrite(SS, HIGH);   
  dataFile = SD.open("/data.csv", FILE_APPEND);
  if (dataFile) {
    dataFile.println(csvLine);
    dataFile.close();
    Serial.println("SD write: " + csvLine); 
  }
}
