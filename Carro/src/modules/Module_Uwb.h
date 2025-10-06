#pragma once
#include "core/Module.h"
#include "core/UwbPipeline.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void UwbTask(void* arg){
  LOGI("UWB","TaskUwb start core=%d", xPortGetCoreID());
  const TickType_t period = pdMS_TO_TICKS(80); // ~12.5 Hz simulado
  TickType_t last = xTaskGetTickCount();
  static uint32_t seq=0;
  while(true){
    UwbMeasurement m; m.seq=++seq; m.t_ms=millis(); m.t_us=micros(); m.valid=true;
    m.d[0]=2.0f + 0.01f*(seq%50); m.d[1]=3.5f+0.02f*(seq%40); m.d[2]=5.1f+0.015f*(seq%60);
    UwbPipeline::publish(m);
    vTaskDelayUntil(&last, period);
  }
}

class Module_Uwb : public Module {
public:
  Module_Uwb(): Module("Uwb") {}
  void configure() override { /* anchors config stub */ }
  bool setup() override {
    LOGI("UWB","Init pipeline y task (stub)");
    if(!UwbPipeline::init()) return false;
    if(xTaskCreatePinnedToCore(UwbTask, "UwbTask", 4096, nullptr, 3, nullptr, 0)!=pdPASS){ LOGE("UWB","No se pudo crear task"); return false; }
    return true;
  }
  void loop() override { /* vacío: event-driven */ }
};
REGISTER_MODULE(Module_Uwb);
