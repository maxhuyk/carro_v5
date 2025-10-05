#pragma once
#include <Arduino.h>
#include <unordered_map>
#include <string>

// ConfigStore simple (clave->string). Futuro: cargar desde flash / JSON.
class ConfigStore {
public:
  static ConfigStore& instance(){ static ConfigStore C; return C; }
  void set(const String& k, const String& v){ data_[k.c_str()] = v.c_str(); }
  bool get(const String& k, String& out) const { auto it=data_.find(k.c_str()); if(it==data_.end()) return false; out=it->second.c_str(); return true; }
  int  getInt(const String& k, int def=0) const { String s; return get(k,s) ? s.toInt() : def; }
  bool getBool(const String& k, bool def=false) const { String s; if(!get(k,s)) return def; s.toLowerCase(); return (s=="1"||s=="true"||s=="on"); }
private:
  std::unordered_map<std::string,std::string> data_; };
