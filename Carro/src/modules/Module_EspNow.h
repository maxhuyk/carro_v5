#pragma once
#include "core/Module.h"

class Module_EspNow : public Module {
public:
  Module_EspNow(): Module("EspNow") {}
  void configure() override { /* cargar canal si existiera */ }
  bool setup() override {
    LOGI("ESP","(stub) init esp-now" );
    return true; }
  void loop() override { /* stub recepción */ }
};
REGISTER_MODULE(Module_EspNow);
