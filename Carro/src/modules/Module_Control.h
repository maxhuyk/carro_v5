#pragma once
#include "core/Module.h"

class Module_Control : public Module {
public:
  Module_Control(): Module("Control", {"Motor","EspNow"}) {}
  void configure() override { /* cargar gains stub */ }
  bool setup() override { LOGI("CTRL","(stub) control listo"); return true; }
  void loop() override { /* control loop stub */ }
};
REGISTER_MODULE(Module_Control);
