#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SD_CS 5  // CS pin from wiring

void setup() {
  Serial.begin(115200);
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Test file write
  File testFile = SD.open("/test.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("Hello from ESP32!");
    testFile.close();
    Serial.println("File written successfully");
  } else {
    Serial.println("File write failed");
  }

  // Read back the file
  testFile = SD.open("/test.txt");
  if (testFile) {
    Serial.println("Reading file:");
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
  } else {
    Serial.println("File open failed");
  }
}

void loop() {
}
