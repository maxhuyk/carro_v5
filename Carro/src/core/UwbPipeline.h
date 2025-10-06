// Event-driven UWB -> Control pipeline
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct UwbMeasurement {
  uint32_t seq = 0;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  float d[3] = {0,0,0};
  bool valid = false;
};

namespace UwbPipeline {
  bool init();
  bool publish(const UwbMeasurement& m); // overwrite latest
  bool wait(UwbMeasurement& m, uint32_t timeoutMs); // blocks
  void recordControlLatency(uint32_t latencyUs);
  void dumpStats();
  void resetStats();
}
