#pragma once
#include <Arduino.h>

// Configuración del subsistema de logging.
// Se puede especializar luego para cargar desde NVS.

namespace config {

// Niveles de severidad (orden creciente)
enum class LogLevel : uint8_t {
    TRACE = 0,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    CRITICAL,
    NONE // deshabilita
};

// Modo de salida del logger
enum class LogMode : uint8_t {
    DESHABILITADO = 0,
    SOLO_SERIAL,
    SOLO_ARCHIVO,
    SERIAL_Y_ARCHIVO
};

struct LoggingConfig {
    LogLevel nivel_minimo = LogLevel::INFO;   // Nivel mínimo a registrar
    LogMode  modo = LogMode::SERIAL_Y_ARCHIVO; // Destino(s)
    const char* ruta_archivo = "/log.txt";   // Archivo en SPIFFS
    size_t tam_max_archivo = 200*1024;        // 200 KB para rotación
    bool timestamp_relativo = true;           // true = millis, false = epoch (si hubiese RTC)
};

// Config por defecto (podrá sobreescribirse antes de iniciar el logger)
LoggingConfig obtenerLoggingConfigDefault();

} // namespace config
