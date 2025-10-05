#pragma once
#include "core/Module.h"

class Module_Emergency : public Module {
public:
  Module_Emergency(): Module("Emergency", {"Motor"}) {}
  void configure() override { pin_ = 36; threshold_ = 1000; }
  bool setup() override { pinMode(pin_, INPUT); LOGI("EMERG","(stub) pin=%d", pin_); return true; }
  void loop() override {
    int raw = analogRead(pin_);
    if(raw > threshold_) { /* tomar control motores en el futuro */ }
  }
private:
  int pin_=36; int threshold_=1000;
};
REGISTER_MODULE(Module_Emergency);
