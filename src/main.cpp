#include <Arduino.h>
#if defined(ESP_PLATFORM)
#include "esp_log.h"
#endif

#include "crawler.h"

SET_LOOP_TASK_STACK_SIZE(32 * 1024);

Crawler crawlerApp;

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
#if defined(ESP_PLATFORM)
  // analogRead() reconfigures the ADC GPIO and the GPIO driver logs each
  // change at INFO level. Keep those internal messages out of the serial
  // calibration stream.
  esp_log_level_set("gpio", ESP_LOG_WARN);
#endif
  delay(250);
  crawlerApp.begin();
}

void loop() { crawlerApp.update(); }
