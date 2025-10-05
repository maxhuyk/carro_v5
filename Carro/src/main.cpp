#include <Arduino.h>
#include "core/Module.h"
#include "core/ConfigStore.h"
#include "core/Log.h"
// Los módulos se registran por sus headers (registradores estáticos)
#include "modules/Module_EspNow.h"
#include "modules/Module_Motor.h"
#include "modules/Module_Emergency.h"
#include "modules/Module_Uwb.h"
#include "modules/Module_Control.h"

void applyConfig(){
  auto& cfg = ConfigStore::instance();
  // Ejemplos de llaves (en futuro cargar desde JSON / SPIFFS)
  cfg.set("enable.Uwb", "false");
  cfg.set("enable.Control", "true");
  cfg.set("enable.EspNow", "true");
  cfg.set("enable.Motor", "true");
  cfg.set("enable.Emergency", "true");
}

void enableFromConfig(){
  auto& cfg = ConfigStore::instance();
  // Lista de módulos conocidos (si agregas más, extiende este arreglo):
  const char* names[] = {"Uwb","Control","EspNow","Motor","Emergency"};
  for(auto name: names){
    String key = String("enable.") + name;
    // Por defecto: true si no existe la clave
    bool en = cfg.getBool(key, true);
    if(!en){
      ModuleManager::instance().disableModule(name);
      LOGW("CFG","Deshabilitado via config: %s", name);
    }
  }
}

void setup(){
  Serial.begin(500000);
  delay(200);
  LogConfig lc; lc.minLevel = LogLevel::INFO; // Ajusta el nivel global aquí
  LoggerMini::instance().configure(lc);
  LOGI("BOOT","Esqueleto modular inicializando");
  applyConfig();
  enableFromConfig();
  // Fases
  ModuleManager::instance().configureAll();
  ModuleManager::instance().setupAll();
  ModuleManager::instance().dumpStatus();
  LOGI("BOOT","Inicio completo");
}

unsigned long lastDiag=0;
void loop(){
  ModuleManager::instance().loopAll();
  if(millis()-lastDiag>5000){ ModuleManager::instance().dumpStatus(); lastDiag=millis(); }
  delay(2);
}