#include <Arduino.h>
#include "core/Module.h"
#include "core/Log.h"
#include "core/UwbPipeline.h"
#include "modules/Module_EspNow.h"
#include "modules/Module_Motor.h"
#include "modules/Module_Emergency.h"
#include "modules/Module_Uwb.h"
#include "modules/Module_Control.h"
#include "core/BootConfig.h"

void setup(){
  Serial.begin(500000);
  delay(200);
  // Logger temporal para boot (INFO) antes de aplicar runtime
  LogConfig lc; lc.minLevel = LogLevel::INFO; LoggerMini::instance().configure(lc);
  LOGI("BOOT","Inicializando");

  // Aplicar configuración centralizada (defaults -> enable -> runtime)
  BootConfig::applyAll();

  ModuleManager::instance().configureAll();
  ModuleManager::instance().setupAll();
  ModuleManager::instance().dumpStatus();
  LOGI("BOOT","Inicio completo");
}

unsigned long lastDiag=0;
void loop(){
  ModuleManager::instance().loopAll();
  if(millis()-lastDiag>5000){ ModuleManager::instance().dumpStatus(); UwbPipeline::dumpStats(); lastDiag=millis(); }
  delay(2);
}