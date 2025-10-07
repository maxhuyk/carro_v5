#include "core/BootConfig.h"
#include "core/ConfigStore.h"
#include "core/Module.h"
#include "core/Log.h"

namespace BootConfig {

void applyDefaults(){
  auto &cfg = ConfigStore::instance();
  cfg.set("enable.Uwb","true");
  cfg.set("enable.Control","true");
  cfg.set("enable.EspNow","true");
  cfg.set("enable.Motor","true");
  cfg.set("enable.Emergency","true");
  cfg.set("log.level","debug");
}

void applyModuleEnables(){
  auto &cfg = ConfigStore::instance();
  const char* names[] = {"Uwb","Control","EspNow","Motor","Emergency"};
  for(auto name: names){
    String key = String("enable.") + name;
    bool en = cfg.getBool(key, true);
    if(!en){ ModuleManager::instance().disableModule(name); LOGW("CFG","Deshabilitado via config: %s", name); }
  }
}

void applyRuntime(){
  auto &cfg = ConfigStore::instance();
  String lvl; if(cfg.get("log.level", lvl)){
    lvl.toLowerCase(); LogConfig lc; if(lvl=="trace") lc.minLevel=LogLevel::TRACE; else if(lvl=="debug") lc.minLevel=LogLevel::DEBUG; else if(lvl=="warn") lc.minLevel=LogLevel::WARN; else if(lvl=="error") lc.minLevel=LogLevel::ERROR; else lc.minLevel=LogLevel::INFO; LoggerMini::instance().configure(lc);
  }
}

} // namespace BootConfig
