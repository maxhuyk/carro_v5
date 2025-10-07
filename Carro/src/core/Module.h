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
  // Configuración runtime de timing
  void setPeriodMs(uint32_t p){ periodMs_=p; }
  void setLoopBudgetUs(uint32_t b){ budgetUs_=b; }
  uint32_t periodMs() const { return periodMs_; }
  uint32_t loopBudgetUs() const { return budgetUs_; }
  uint32_t worstLoopUs() const { return worstLoopUs_; }
  uint32_t overBudgetCount() const { return overBudgetCount_; }
  // Fases
  virtual void configure() {}
  virtual bool setup() { return true; }
  virtual void loop() {}
  // Métricas
  unsigned long setupDurationMs() const { return setupDurMs_; }
  unsigned long loopAccumulatedUs() const { return loopAccumUs_; }
  void _setSetupDuration(unsigned long ms){ setupDurMs_=ms; }
  void _accLoop(unsigned long us){ loopAccumUs_ += us; if(us>worstLoopUs_) worstLoopUs_=us; if(budgetUs_ && us>budgetUs_) overBudgetCount_++; }
  void _markFailed(){ failedSetup_=true; enabled_=false; }
  bool _dueToRun(unsigned long nowMs) {
    if(!periodMs_) return true; // correr cada iteración
    if(nowMs < lastRunMs_) { lastRunMs_=nowMs; return true; } // wrap
    return (nowMs - lastRunMs_) >= periodMs_;
  }
  void _markRun(unsigned long nowMs){ lastRunMs_=nowMs; }
private:
  const char* name_;
  std::vector<const char*> deps_;
  bool enabled_ = true;
  bool failedSetup_ = false;
  unsigned long setupDurMs_ = 0;
  unsigned long loopAccumUs_ = 0;
  // Scheduling
  uint32_t periodMs_ = 0; // 0 = cada iteración
  uint32_t budgetUs_ = 0; // 0 = sin budget
  uint32_t worstLoopUs_ = 0;
  uint32_t overBudgetCount_ = 0;
  unsigned long lastRunMs_ = 0;
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
  void loopAll(){
    unsigned long nowMs = millis();
    for(auto*m:modules_) {
      if(!m->enabled()) continue;
      if(!m->_dueToRun(nowMs)) continue;
      m->_markRun(nowMs);
      auto t0=micros();
      m->loop();
      auto dur = micros()-t0;
      m->_accLoop(dur);
      if(m->loopBudgetUs() && dur>m->loopBudgetUs()*4) { // 4x budget => log fuerte
        LOGE("MOD","%s loop %luus excede 4x budget %u", m->name(), (unsigned long)dur, m->loopBudgetUs());
      } else if(m->loopBudgetUs() && dur>m->loopBudgetUs()) {
        LOGW("MOD","%s loop %luus > budget %u", m->name(), (unsigned long)dur, m->loopBudgetUs());
      }
    }
  }
  void disableModule(const char* name){ for(auto*m:modules_) if(strcmp(m->name(),name)==0) m->setEnabled(false); }
  void dumpStatus(){
    for(auto*m:modules_){
      LOGI("MOD","STATUS %s=%s setup=%lums loop_us=%lu worst=%u budget=%u over=%u period=%u failed=%d",
        m->name(), m->enabled()?"ON":"OFF", m->setupDurationMs(), m->loopAccumulatedUs(), m->worstLoopUs(), m->loopBudgetUs(), m->overBudgetCount(), m->periodMs(), m->failedSetup());
    }
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
