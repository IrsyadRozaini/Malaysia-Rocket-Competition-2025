#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Create BME280 object
Adafruit_BME280 bme;

// I2C address can be 0x76 or 0x77 depending on module
#define SEALEVELPRESSURE_HPA (1013.25)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("BME280 test starting...");

  if (!bme.begin(0x76)) {   // try 0x76 first
    Serial.println("Could not find BME280 at 0x76, trying 0x77...");
    if (!bme.begin(0x77)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
    }
  }
}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
  delay(2000);
}
