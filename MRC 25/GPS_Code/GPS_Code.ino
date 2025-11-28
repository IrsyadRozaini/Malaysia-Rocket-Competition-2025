#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

HardwareSerial mySerial(1);
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.println("GPS Test - Raw NMEA Data:");
  Serial.println("Waiting for GPS fix...");
}

void loop() {
  while (mySerial.available()) {
    char c = mySerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude: ");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.print("Fix status: ");
    Serial.println(gps.location.isValid() ? "Fixed" : "No fix");

    float speedKmh = gps.speed.kmph();
    if (speedKmh > 0.1) {
      Serial.print("Speed (km/h): ");
      Serial.println(speedKmh);
    } else {
      Serial.println("Speed: 0.0 km/h (Stationary)");
    }
  } else {
    Serial.println("Waiting for valid GPS data...");
  }

  delay(2000);
} 
