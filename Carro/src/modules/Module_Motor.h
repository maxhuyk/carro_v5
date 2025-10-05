#pragma once
#include "core/Module.h"

class Module_Motor : public Module {
public:
  Module_Motor(): Module("Motor") {}
  void configure() override { /* leer pwm config futura */ }
  bool setup() override { LOGI("MOTOR","(stub) driver listo"); return true; }
  void loop() override { /* aplicar último comando */ }
};
REGISTER_MODULE(Module_Motor);
