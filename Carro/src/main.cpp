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

void setup(){
  Serial.begin(500000);
  delay(200);
  LogConfig lc; lc.minLevel = LogLevel::INFO; // Ajusta el nivel global aquí
  LoggerMini::instance().configure(lc);
  LOGI("BOOT","Esqueleto modular inicializando");
  applyConfig();
  enableFromConfig();
  // Periodos antiguos omitidos: Control y UWB ahora son event-driven via tasks multicore.
  // Fases
  ModuleManager::instance().configureAll();
  ModuleManager::instance().setupAll();
  ModuleManager::instance().dumpStatus();
  LOGI("BOOT","Inicio completo");
}

unsigned long lastDiag=0;
static char cmdBuf[64];
static uint8_t cmdLen=0;

void processCommand(const char* c){
  if(strcmp(c,"status")==0){ ModuleManager::instance().dumpStatus(); return; }
  if(strcmp(c,"uwbstats")==0){ UwbPipeline::dumpStats(); return; }
  if(strcmp(c,"uwbreset")==0){ UwbPipeline::resetStats(); Serial.println("UWB stats reset"); return; }
  if(strcmp(c,"uwbverbose on")==0){ UWBCore_setVerbose(true); Serial.println("UWB verbose ON"); return; }
  if(strcmp(c,"uwbverbose off")==0){ UWBCore_setVerbose(false); Serial.println("UWB verbose OFF"); return; }
  if(strcmp(c,"uwbnow")==0){ UWBRawData snap; if(UWBCore_snapshot(snap)) Serial.printf("UWB snapshot: d1=%.1f v1=%d d2=%.1f v2=%d d3=%.1f v3=%d ts=%lu\n", snap.distance1,snap.valid1,snap.distance2,snap.valid2,snap.distance3,snap.valid3,snap.timestamp); else Serial.println("UWB snapshot fail"); return; }
  if(strncmp(c,"log ",4)==0){
    const char* lvl = c+4;
    LogConfig lc; if(strcmp(lvl,"trace")==0) lc.minLevel=LogLevel::TRACE; else if(strcmp(lvl,"debug")==0) lc.minLevel=LogLevel::DEBUG; else if(strcmp(lvl,"info")==0) lc.minLevel=LogLevel::INFO; else if(strcmp(lvl,"warn")==0) lc.minLevel=LogLevel::WARN; else if(strcmp(lvl,"error")==0) lc.minLevel=LogLevel::ERROR; else { Serial.println("Nivel invalido"); return; }
    LoggerMini::instance().configure(lc); Serial.println("Nivel log actualizado"); return;
  }
  if(strcmp(c,"help")==0){ Serial.println("Comandos: status, uwbstats, uwbreset, uwbverbose on|off, uwbnow, log <trace|debug|info|warn|error>, help"); return; }
  Serial.println("Cmd desconocido (help)");
}

void loop(){
  ModuleManager::instance().loopAll();
  if(millis()-lastDiag>5000){ ModuleManager::instance().dumpStatus(); lastDiag=millis(); }
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(ch=='\r') continue;
    if(ch=='\n'){ cmdBuf[cmdLen]=0; if(cmdLen>0) processCommand(cmdBuf); cmdLen=0; continue; }
    if(cmdLen<sizeof(cmdBuf)-1) cmdBuf[cmdLen++]=ch;
  }
  delay(2);
}