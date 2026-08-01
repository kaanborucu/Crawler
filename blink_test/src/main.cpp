#include <Arduino.h>

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  Serial.println("ESP32 core RGB blink test started");
}

void loop() {
  neopixelWrite(RGB_BUILTIN, 32, 0, 0);
  Serial.println("LED ON");
  delay(500);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  Serial.println("LED OFF");
  delay(500);
}
