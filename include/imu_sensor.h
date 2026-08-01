#pragma once

#include "crawler_config.h"
#include "crawler_types.h"

#if defined(ARDUINO)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#endif

class ImuSensor {
 public:
  ImuSensor();

  bool begin();
  crawler::ImuState read();

 private:
  crawler::ImuState readFifoSample();
  void publish(const crawler::ImuState& state);
#if defined(ARDUINO)
  static void taskEntry(void* argument);
  void taskLoop();
#endif

  uint8_t address_;
  bool present_;
  crawler::ImuState latest_;
#if defined(ARDUINO)
  portMUX_TYPE mutex_;
  TaskHandle_t taskHandle_;
#endif
};
