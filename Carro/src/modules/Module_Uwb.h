#pragma once
#include "core/Module.h"

class Module_Uwb : public Module {
public:
  Module_Uwb(): Module("Uwb") {}
  void configure() override { /* anchors config stub */ }
  bool setup() override { LOGI("UWB","(stub) uwb init pospuesto"); return true; }
  void loop() override { /* ranging futuro */ }
};
REGISTER_MODULE(Module_Uwb);
