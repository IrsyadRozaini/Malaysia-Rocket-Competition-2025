#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Create GPS object
TinyGPSPlus gps;

// Create Serial2 on GPIO16 (RX) and GPIO17 (TX)
HardwareSerial GPS_Serial(2);

void setup() {
  Serial.begin(115200);          // Serial Monitor
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // Baud rate, RX, TX
  Serial.println("GPS Module Test Starting...");
}

void loop() {
  // Read data from GPS module
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // Display GPS info every 1 second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();

    if (gps.location.isValid()) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude (m): ");
      Serial.println(gps.altitude.meters());
      Serial.print("Speed (km/h): ");
      Serial.println(gps.speed.kmph());
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
    } else {
      Serial.println("Waiting for GPS fix...");
    }
    Serial.println("--------------------------");
  }
}
