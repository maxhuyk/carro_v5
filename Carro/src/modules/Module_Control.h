#pragma once
#include "core/Module.h"
#include "core/UwbPipeline.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void ControlTask(void* arg){
  LOGI("CTRL","TaskControl start core=%d", xPortGetCoreID());
  while(true){
    UwbMeasurement m; if(UwbPipeline::wait(m, UINT32_MAX)){
      uint32_t lat = micros() - m.t_us; UwbPipeline::recordControlLatency(lat);
      // Aquí irá la lógica real de control (trilateración, filtros, PID, motores)
    }
  }
}

class Module_Control : public Module {
public:
  Module_Control(): Module("Control", {"Motor","EspNow"}) {}
  void configure() override { /* cargar gains stub */ }
  bool setup() override {
    LOGI("CTRL","Creando task control");
    if(xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, nullptr, 2, nullptr, 1)!=pdPASS){ LOGE("CTRL","No se pudo crear task"); return false; }
    return true;
  }
  void loop() override { /* vacío */ }
};
REGISTER_MODULE(Module_Control);
