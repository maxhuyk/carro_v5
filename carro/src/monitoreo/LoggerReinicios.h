/**
 * @file LoggerReinicios.h
 * @brief Logger con soporte de niveles, persistencia en SPIFFS y registro de reinicios.
 */
#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <vector>
#include "config/ConfigLogging.h"

namespace monitoreo {

using config::LogLevel;
using config::LogMode;
using config::LoggingConfig;

// Entrada registrada de un reinicio
struct EntradaReinicio {
    uint32_t timestamp_ms;     // millis() cuando se registró
    esp_reset_reason_t razon;  // Razón devuelta por el chip
    uint32_t duracion_init_ms; // Tiempo que tardó la inicialización actual
};

// Línea genérica de log
struct LineaLog {
    uint32_t ts_ms;      // Timestamp relativo (o absoluto si se implementa)
    LogLevel nivel;      // Severidad
    String categoria;    // Módulo / etiqueta
    String mensaje;      // Contenido
};

class Logger {
public:
    static Logger& instancia();

    // Debe llamarse antes de cualquier log. Monta SPIFFS y abre archivo si corresponde
    bool iniciar(const LoggingConfig& cfg);

    // Registrar arranque
    void registrarBoot(uint32_t duracion_init_ms);

    // Log genérico
    void log(LogLevel nivel, const __FlashStringHelper* categoria, const __FlashStringHelper* msg);
    void logf(LogLevel nivel, const char* categoria, const char* fmt, ...);

    // Volcado por Serial del historial en RAM (reinicios + últimas líneas)
    void volcarEstado(Stream& out) const;

    // Cambio dinámico del nivel mínimo
    void setNivelMinimo(LogLevel lvl) { config_.nivel_minimo = lvl; }
    LogLevel getNivelMinimo() const { return config_.nivel_minimo; }

private:
    Logger() = default;
    void escribirLinea(const LineaLog& l);
    void rotarSiNecesario();
    const __FlashStringHelper* nivelToString(LogLevel n) const;

    LoggingConfig config_{};
    bool iniciado_ = false;
    std::vector<EntradaReinicio> reinicios_;
    std::vector<LineaLog> buffer_reciente_; // pequeño buffer en RAM (ej: últimas 64)
};

} // namespace monitoreo
