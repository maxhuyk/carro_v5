/** Logging mínimo (se amplía luego). */
#pragma once
#include <Arduino.h>

enum class LogLevel { TRACE, DEBUG, INFO, WARN, ERROR, NONE }; 

struct LogConfig { LogLevel minLevel = LogLevel::INFO; };

class LoggerMini {
public:
  static LoggerMini& instance(){ static LoggerMini L; return L; }
  void configure(const LogConfig& c){ cfg_=c; }
  void logf(LogLevel lvl, const char* tag, const char* fmt, ...){
    if(lvl < cfg_.minLevel) return;
    char buf[160]; va_list ap; va_start(ap, fmt); vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
    Serial.printf("[%s] %s\n", tag, buf);
  }
private: LogConfig cfg_{}; };

#define LOGT(tag, fmt, ...) LoggerMini::instance().logf(LogLevel::TRACE, tag, fmt, ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) LoggerMini::instance().logf(LogLevel::DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOGI(tag, fmt, ...) LoggerMini::instance().logf(LogLevel::INFO, tag, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) LoggerMini::instance().logf(LogLevel::WARN, tag, fmt, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) LoggerMini::instance().logf(LogLevel::ERROR, tag, fmt, ##__VA_ARGS__)
