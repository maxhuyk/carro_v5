// LogMacros.h - Macros de conveniencia para el sistema de logging
#pragma once
#include "monitoreo/LoggerReinicios.h"

// Macros de nivel (cat = etiqueta corta, fmt estilo printf)
#define LOG_TRACE(cat, fmt, ...) monitoreo::Logger::instancia().logf(config::LogLevel::TRACE, cat, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(cat, fmt, ...) monitoreo::Logger::instancia().logf(config::LogLevel::DEBUG, cat, fmt, ##__VA_ARGS__)
#define LOG_INFO(cat, fmt, ...)  monitoreo::Logger::instancia().logf(config::LogLevel::INFO,  cat, fmt, ##__VA_ARGS__)
#define LOG_WARN(cat, fmt, ...)  monitoreo::Logger::instancia().logf(config::LogLevel::WARN,  cat, fmt, ##__VA_ARGS__)
#define LOG_ERROR(cat, fmt, ...) monitoreo::Logger::instancia().logf(config::LogLevel::ERROR, cat, fmt, ##__VA_ARGS__)
#define LOG_CRIT(cat, fmt, ...)  monitoreo::Logger::instancia().logf(config::LogLevel::CRITICAL, cat, fmt, ##__VA_ARGS__)
