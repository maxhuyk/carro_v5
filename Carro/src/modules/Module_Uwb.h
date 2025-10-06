#pragma once
// Adaptado: usa la implementación literal UWBCore y aprovecha el pipeline
#include "core/Module.h"
#include "core/UwbPipeline.h" // para publish dentro de UWBCore (ya integrado en UWBCore_task)
#include "uwb/UWBCore.h"

class Module_Uwb : public Module {
public:
  Module_Uwb(): Module("Uwb") {}
  void configure() override { /* sin cambios: setup real en UWBCore_setup */ }
  bool setup() override {
    LOGI("UWB","Integrando UWBCore literal");
    if(!UwbPipeline::init()) return false; // pipeline activo para ControlTask
    UWBCore_setup();
    UWBCore_startTask();
    return true; }
  void loop() override { /* vacío: event-driven */ }
};
REGISTER_MODULE(Module_Uwb);
