#pragma once
#include "core/Module.h"
#include "uwb/UWBCore.h" // ahora leemos directamente del core UWB
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void ControlTask(void* arg){
  LOGI("CTRL","TaskControl start core=%d", xPortGetCoreID());
  unsigned long lastCount = 0;
  UWBRawData raw{};
  while(true){
    unsigned long c = UWBCore_getMeasurementCount();
    if(c != lastCount){
      if(UWBCore_getRawData(raw)){
        // Aquí irá la lógica real: usar raw.distance1..3 y valid flags
        // latencia aproximada: micros() - (raw.timestamp*1000)
      }
      lastCount = c;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // ceder CPU
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
