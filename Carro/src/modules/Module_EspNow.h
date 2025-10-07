#pragma once
#include "core/Module.h"
#include <esp_now.h>
#include <WiFi.h>
#include "core/Log.h"

// Exposición mínima para otros módulos (Control) que quieran leer estado manual
struct EspNowSharedState {
  volatile uint8_t  modo = 0;
  volatile uint16_t battery_mV = 0;
  volatile int16_t  manualVelL = 0;
  volatile int16_t  manualVelR = 0;
  volatile unsigned long lastRxMs = 0;
  volatile unsigned long totalPackets = 0;
  volatile bool valid = false; // datos recientes (<1s)
};

EspNowSharedState& EspNow_getState();

class Module_EspNow : public Module {
public:
  Module_EspNow(): Module("EspNow") {}
  bool setup() override;
  void loop() override;
};
