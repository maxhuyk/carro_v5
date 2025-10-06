#pragma once
// Adaptado: usa la implementación literal UWBCore y aprovecha el pipeline
#include "core/Module.h"
#include "core/UwbPipeline.h" // para publish dentro de UWBCore (ya integrado en UWBCore_task)
#include "uwb/UWBCore.h"
#include <Wire.h>
#include "DW3000.h"

class Module_Uwb : public Module {
public:
  Module_Uwb(): Module("Uwb") {}
  void configure() override { /* sin cambios: setup real en UWBCore_setup */ }
  bool setup() override {
    LOGI("UWB","Integrando UWBCore literal");
    // Inicialización necesaria previa (literal del main original adaptada aquí)
    // I2C sólo para MCP23008
    Wire.begin(5,4); // SDA=5 SCL=4 (como CarroClean)
    Wire.setClock(500000);
    if(!DW3000Class::initMCP23008()){
      LOGE("UWB","Fallo init MCP23008");
      return false;
    }
    // (Pipeline opcional: si se quiere mantener, descomentar)
    // UwbPipeline::init();
    UWBCore_setup();
    UWBCore_startTask();
    return true; }
  void loop() override { /* vacío: event-driven */ }
};
REGISTER_MODULE(Module_Uwb);
