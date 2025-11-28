#include <ESP32Servo.h>   // use this library for ESP32

Servo myservo;            // create servo object

int pos = 0;              // variable to store position

void setup() {
  myservo.attach(13);     // attach servo signal wire to GPIO13 (D13)
}

void loop() {
  // sweep from 0° to 180°
  for (pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);   // move servo to "pos" degrees
    delay(0);            // wait for servo to move
  }

  // sweep back from 180° to 0°
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(0);
  }
}
