#include <SPI.h>
#include <LoRa.h>

#define SS 5
#define RST 14
#define DIO0 2

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6)) { // Use 915E6 or 868E6 depending on module version
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Transmitter");
}

void loop() {
  Serial.println("Sending packet...");
  LoRa.beginPacket();
  LoRa.print("Hello CanSat");
  LoRa.endPacket();
  delay(2000);
}
