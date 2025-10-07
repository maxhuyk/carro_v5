#pragma once
#include "core/Module.h"
#include "uwb/UWBCore.h"
#include "motor/MotorController.h"
#include "control/ControlCore.h"
#include "config/Config.h"
#include "modules/Module_EspNow.h" // para EspNow_getState
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
      unsigned long tControlStartUs = micros();
      if(UWBCore_getRawData(raw)){
        // Distancias: convertimos a mm si no son NAN
        data.distancias[0] = (isnan(raw.distance1)? -1.0f : raw.distance1 * 10.0f);
        data.distancias[1] = (isnan(raw.distance2)? -1.0f : raw.distance2 * 10.0f);
        data.distancias[2] = (isnan(raw.distance3)? -1.0f : raw.distance3 * 10.0f);
        // Integrar estado ESP-NOW
        auto &st = EspNow_getState();
        data.control_data[0] = st.battery_mV / 1000.0f; // battery voltage en volts
        data.control_data[1] = st.modo;                 // modo
        data.buttons_data[2] = st.manualVelL;           // manual izquierda
        data.buttons_data[3] = st.manualVelR;           // manual derecha
        // Decidir data_valid: ejecutamos siempre (true) para que la lógica de seguridad actúe internamente
        data.data_valid = true; // si quisieras condicionar: (st.valid || true)
        // Ejecutar control
        ControlCore_run(data);
        // Latencia UWB->Control (timestamp raw en ms vs ahora)
        static unsigned long lastLatLog=0; unsigned long nowMs=millis();
        if(nowMs - lastLatLog > 5000){
          unsigned long ageMs = (nowMs > raw.timestamp)? (nowMs - raw.timestamp) : 0;
          LOGD("CTRL","Latencia UWB->Control=%lums seq=%lu", ageMs, c);
          lastLatLog = nowMs;
        }
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
