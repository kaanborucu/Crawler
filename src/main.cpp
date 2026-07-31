#include <Arduino.h>

#include "crawler.h"

SET_LOOP_TASK_STACK_SIZE(32 * 1024);

Crawler crawlerApp;

void setup() {
  Serial.begin(115200);
  delay(250);
  crawlerApp.begin();
}

void loop() { crawlerApp.update(); }
