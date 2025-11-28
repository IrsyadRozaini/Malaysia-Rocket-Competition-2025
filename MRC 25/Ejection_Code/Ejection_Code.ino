#include <ESP32Servo.h>

// Pin assignments
const int tiltPin   = 32;   // Tilt sensor
const int servoPin  = 25;   // Servo pin

Servo latchServo;

void setup() {
  pinMode(tiltPin, INPUT);

  latchServo.attach(servoPin);
  latchServo.write(0); // Servo starts locked at 0°

  Serial.begin(115200);
  Serial.println("System ready. Waiting for tilt...");
}

void loop() {
  int tiltState = digitalRead(tiltPin);

  if (tiltState == HIGH) {
    // Confirm tilt stable for 2 sec
    unsigned long startTime = millis();
    bool stillTilted = true;

    while (millis() - startTime < 2000) { // 2s debounce
      if (digitalRead(tiltPin) == LOW) {
        stillTilted = false;
        break;
      }
    }

    if (stillTilted) {
      Serial.println("Tilt confirmed! Activating servo...");

      // Step 1: Rotate servo to unlatch
      latchServo.write(180);
      delay(1000); // allow servo to move fully

      // Step 2: Reset servo back to 0°
      latchServo.write(0);
      delay(500);

      Serial.println("Sequence finished. Servo reset. Waiting for next tilt...");
    }
  }
}
