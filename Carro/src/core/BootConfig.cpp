#include "core/BootConfig.h"
#include "core/ConfigStore.h"
#include "core/Module.h"
#include "core/Log.h"
#include "config/Config.h" // para CONFIG_LOG_LEVEL_DEFAULT

namespace BootConfig {

void applyDefaults(){
  auto &cfg = ConfigStore::instance();
  cfg.set("enable.Uwb","true");
  cfg.set("enable.Control","true");
  cfg.set("enable.EspNow","true");
  cfg.set("enable.Motor","true");
  cfg.set("enable.Emergency","true");

  // Mapear CONFIG_LOG_LEVEL_DEFAULT numérico a string runtime
  // 0=TRACE 1=DEBUG 2=INFO 3=WARN 4=ERROR (otros -> info)
#if   CONFIG_LOG_LEVEL_DEFAULT == 0
  cfg.set("log.level","trace");
#elif CONFIG_LOG_LEVEL_DEFAULT == 1
  cfg.set("log.level","debug");
#elif CONFIG_LOG_LEVEL_DEFAULT == 2
  cfg.set("log.level","info");
#elif CONFIG_LOG_LEVEL_DEFAULT == 3
  cfg.set("log.level","warn");
#elif CONFIG_LOG_LEVEL_DEFAULT == 4
  cfg.set("log.level","error");
#else
  cfg.set("log.level","info");
#endif
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
