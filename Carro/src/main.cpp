#include <Arduino.h>
#include "core/Module.h"
#include "core/ConfigStore.h"
#include "core/Log.h"
#include "core/UwbPipeline.h" // para comandos uwbstats/uwbreset
// Los módulos se registran por sus headers (registradores estáticos)
#include "modules/Module_EspNow.h"
#include "modules/Module_Motor.h"
#include "modules/Module_Emergency.h"
#include "modules/Module_Uwb.h"
#include "modules/Module_Control.h"
#include "uwb/UWBCore.h"

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

static void applyRuntimeConfig(){
  auto &cfg = ConfigStore::instance();
  // Nivel de log desde config: log.level = trace|debug|info|warn|error
  String lvl;
  if(cfg.get("log.level", lvl)){
    lvl.toLowerCase();
    LogConfig lc;
    if(lvl=="trace") lc.minLevel=LogLevel::TRACE; else if(lvl=="debug") lc.minLevel=LogLevel::DEBUG; else if(lvl=="warn") lc.minLevel=LogLevel::WARN; else if(lvl=="error") lc.minLevel=LogLevel::ERROR; else lc.minLevel=LogLevel::INFO;
    LoggerMini::instance().configure(lc);
  }
  // Verbose UWB: uwb.verbose = true/false
  bool uwbV = cfg.getBool("uwb.verbose", false);
  UWBCore_setVerbose(uwbV);
}

void setup(){
  Serial.begin(500000);
  delay(200);
  // Logger default (INFO) antes de cargar config para ver boot
  LogConfig lc; lc.minLevel = LogLevel::INFO; LoggerMini::instance().configure(lc);
  LOGI("BOOT","Esqueleto modular inicializando");
  applyConfig();             // carga valores en memoria (luego se reemplazará por lectura de archivo)
  enableFromConfig();        // habilita/deshabilita módulos
  applyRuntimeConfig();      // aplica nivel de log y verbose UWB desde config
  ModuleManager::instance().configureAll();
  ModuleManager::instance().setupAll();
  ModuleManager::instance().dumpStatus();
  LOGI("BOOT","Inicio completo");
}

unsigned long lastDiag=0;
void loop(){
  ModuleManager::instance().loopAll();
  if(millis()-lastDiag>5000){ ModuleManager::instance().dumpStatus(); UwbPipeline::dumpStats(); lastDiag=millis(); }
  // Sin interfaz de comandos: toda la configuración proviene de ConfigStore / archivos (pendiente persistencia)
  delay(2);
}