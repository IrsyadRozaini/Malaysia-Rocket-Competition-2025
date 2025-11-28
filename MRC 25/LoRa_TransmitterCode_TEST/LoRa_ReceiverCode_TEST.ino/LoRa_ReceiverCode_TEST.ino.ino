#include <SPI.h>
#include <LoRa.h>

#define SS 5
#define RST 14
#define DIO0 2

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Receiver");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
