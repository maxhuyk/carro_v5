#pragma once
#include "core/Module.h"
#include "uwb/UWBCore.h"
#include "motor/MotorController.h"
#include "control/ControlCore.h"
#include "config/Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void ControlTask(void* arg){
  LOGI("CTRL","TaskControl start core=%d", xPortGetCoreID());
  ControlCore_init();
  unsigned long lastCount = 0;
  UWBRawData raw{};
  CarroData data{};
  while(true){
    unsigned long c = UWBCore_getMeasurementCount();
    if(c != lastCount){
      // Obtener raw (aunque anchor inválido, igual corremos)
      if(UWBCore_getRawData(raw)){
        // Convertir a mm (raw.distanceX está en cm?) Asumimos cm->mm multiplicando por 10 si venía en cm.
        data.distancias[0] = raw.distance1 * 10.0f;
        data.distancias[1] = raw.distance2 * 10.0f;
        data.distancias[2] = raw.distance3 * 10.0f;
        data.data_valid = true; // Incluso si alguna inválida, mantenemos true (ajuste futuro si se necesita)
        ControlCore_run(data);
      }
      lastCount = c;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

class Module_Control : public Module {
public:
  Module_Control(): Module("Control", {"Motor","EspNow","Uwb"}) {}
  bool setup() override {
    LOGI("CTRL","Creando task control");
    if(xTaskCreatePinnedToCore(ControlTask, "ControlTask", 6144, nullptr, 2, nullptr, 1)!=pdPASS){ LOGE("CTRL","No se pudo crear task"); return false; }
    return true;
  }
  void loop() override {}
};
REGISTER_MODULE(Module_Control);
