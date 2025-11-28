#include <SPI.h>
#include <LoRa.h>

#define SS   5    // adjust to your LoRa module wiring
#define RST  14
#define DIO0 26

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6)) {   // adjust frequency to match CanSat
    Serial.println("LoRa init failed. Check wiring.");
    while (1);
  }

  // optional: make sure both sides use same settings
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17); 

  Serial.println("LoRa receiver ready!");
}

void loop() {
  // check for packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String data = "";
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      data += c;
    }

    // --- Basic validation ---
    if (data.startsWith("NOVA, ")) {
     processTelemetry(data);  // clean telemetry line
    } else {
      Serial.println("BAD: " + data); // corrupted or noise
    }
  }
}

// --- Helper to split CSV string into fields ---
int splitCSV(String s, String *fields, int maxFields) {
  int idx = 0;
  int start = 0;
  for (int i = 0; i <= s.length(); i++) {
    if (s[i] == ',' || i == s.length()) {
      fields[idx++] = s.substring(start, i);
      start = i + 1;
      if (idx >= maxFields) break;
    }
  }
  return idx;
}

void processTelemetry(String packet) {
  String fields[16];
  int count = splitCSV(packet, fields, 16);
  if (count < 16) {
    Serial.println("Parse error, got only " + String(count) + " fields");
    return;
  }


  // Extract fields
  unsigned long timestamp = fields[1].toInt();
  float temp = fields[2].toFloat();
  float pres0 = fields[3].toFloat();
  float pres = pres0 * 100.0;
  float hum  = fields[4].toFloat();
  float alt_bme = fields[5].toFloat(); 

  int16_t ax_raw = fields[6].toInt();
  int16_t ay_raw = fields[7].toInt();
  int16_t az_raw = fields[8].toInt();
  int16_t gx_raw = fields[9].toInt();
  int16_t gy_raw = fields[10].toInt();
  int16_t gz_raw = fields[11].toInt();

  float lat = fields[12] == "nan" ? NAN : fields[12].toFloat();
  float lon = fields[13] == "nan" ? NAN : fields[13].toFloat();
  float alt = fields[14] == "nan" ? NAN : fields[14].toFloat();
  int sats  = fields[15].toInt();

  // Convert raw accel (m/s²)
  float ax = (ax_raw / 16384.0) * 9.80665;
  float ay = (ay_raw / 16384.0) * 9.80665;
  float az = (az_raw / 16384.0) * 9.80665;

  // Convert raw gyro (°/s)
  float gx = gx_raw / 131.2;
  float gy = gy_raw / 131.2;
  float gz = gz_raw / 131.2;

  // Print nicely
  Serial.println("----- NOVA -----");
  Serial.print("Time (ms): "); Serial.println(timestamp);
  Serial.print("Temp (C): "); Serial.println(temp);
  Serial.print("Pressure (Pa): "); Serial.println(pres);
  Serial.print("Humidity (%): "); Serial.println(hum);
   Serial.print("BME Alt (m): "); Serial.println(alt_bme);
  Serial.print("Accel (m/s^2): X="); Serial.print(ax); 
  Serial.print(" Y="); Serial.print(ay); 
  Serial.print(" Z="); Serial.println(az);
  Serial.print("Gyro (deg/s): X="); Serial.print(gx); 
  Serial.print(" Y="); Serial.print(gy); 
  Serial.print(" Z="); Serial.println(gz);
  Serial.print("GPS Lat: "); Serial.println(lat, 6);
  Serial.print("GPS Lon: "); Serial.println(lon, 6);
  Serial.print("GPS Alt (m): "); Serial.println(alt);
  Serial.print("Satellites: "); Serial.println(sats);
  Serial.println("---------------------\n");
}
