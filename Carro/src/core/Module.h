/** Base de módulo y macros de registro */
#pragma once
#include <Arduino.h>
#include <vector>
#include <string>
#include <functional>
#include "Log.h"

class Module {
public:
  Module(const char* name, std::initializer_list<const char*> deps = {}) : name_(name), deps_(deps) {}
  virtual ~Module() = default;
  const char* name() const { return name_; }
  bool enabled() const { return enabled_; }
  void setEnabled(bool e) { enabled_ = e; }
  const std::vector<const char*>& deps() const { return deps_; }
  bool failedSetup() const { return failedSetup_; }
  // Fases
  virtual void configure() {}
  virtual bool setup() { return true; }
  virtual void loop() {}
  // Métricas
  unsigned long setupDurationMs() const { return setupDurMs_; }
  unsigned long loopAccumulatedUs() const { return loopAccumUs_; }
  void _setSetupDuration(unsigned long ms){ setupDurMs_=ms; }
  void _accLoop(unsigned long us){ loopAccumUs_ += us; }
  void _markFailed(){ failedSetup_=true; enabled_=false; }
private:
  const char* name_;
  std::vector<const char*> deps_;
  bool enabled_ = true;
  bool failedSetup_ = false;
  unsigned long setupDurMs_ = 0;
  unsigned long loopAccumUs_ = 0;
};

class ModuleManager {
public:
  static ModuleManager& instance(){ static ModuleManager M; return M; }
  void registerModule(Module* m){ modules_.push_back(m); }
  void configureAll(){ for(auto*m:modules_) if(m->enabled()) m->configure(); }
  void setupAll(){
    // Resolver dependencias simples: si un dep falla, deshabilitar dependiente
    for(auto*m:modules_){ if(!m->enabled()) continue; if(!depsAvailable(m)){ LOGW("MOD","Deshabilitando %s (dep no lista)", m->name()); m->setEnabled(false); }}
    for(auto*m:modules_) {
      if(!m->enabled()) continue;
      unsigned long t0=millis(); bool ok = m->setup(); m->_setSetupDuration(millis()-t0);
      if(!ok){ LOGE("MOD","Setup fallo en %s", m->name()); m->_markFailed(); disableDependents(m->name()); }
      else LOGI("MOD","Setup %s ok (%lums)", m->name(), m->setupDurationMs());
    }
  }
  void loopAll(){ for(auto*m:modules_) if(m->enabled()){ auto t0=micros(); m->loop(); m->_accLoop(micros()-t0);} }
  void disableModule(const char* name){ for(auto*m:modules_) if(strcmp(m->name(),name)==0) m->setEnabled(false); }
  void dumpStatus(){
    Serial.println("--- MODULE STATUS ---");
    for(auto*m:modules_){ Serial.printf("%s : %s setup=%lums loop_us=%lu failed=%d\n", m->name(), m->enabled()?"ON":"OFF", m->setupDurationMs(), m->loopAccumulatedUs(), m->failedSetup()); }
  }
private:
  bool depsAvailable(Module* m){
    for(auto& d: m->deps()){
      auto* ptr = find(d);
      if(!ptr || ptr->failedSetup() || !ptr->enabled()) return false;
    }
    return true;
  }
  void disableDependents(const char* failed){
    for(auto*m:modules_) if(m->enabled()) for(auto& d: m->deps()) if(strcmp(d,failed)==0){ LOGW("MOD","Deshabilitado %s por dependencia %s", m->name(), failed); m->setEnabled(false);} }
  Module* find(const char* n){ for(auto*m:modules_) if(strcmp(m->name(),n)==0) return m; return nullptr; }
  std::vector<Module*> modules_;
};

#define REGISTER_MODULE(cls) static cls _instance_##cls; struct _registrar_##cls { _registrar_##cls(){ ModuleManager::instance().registerModule(&_instance_##cls);} } _registrar_instance_##cls;
