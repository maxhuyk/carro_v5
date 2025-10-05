#include "LoggerReinicios.h"
#include <cstdarg>

namespace monitoreo {

static const size_t BUFFER_MAX = 64; // líneas de log recientes en RAM

Logger& Logger::instancia() { static Logger L; return L; }

bool Logger::iniciar(const LoggingConfig& cfg) {
    config_ = cfg;
    if (config_.modo == LogMode::DESHABILITADO) {
        iniciado_ = true; // Nada que hacer
        return true;
    }
    if (!SPIFFS.begin(true)) {
        Serial.println(F("[LOGGER] ERROR montando SPIFFS"));
        // Aún así permitir logs a Serial
    }
    iniciado_ = true;
    return true;
}

void Logger::registrarBoot(uint32_t duracion_init_ms) {
    EntradaReinicio r{ millis(), esp_reset_reason(), duracion_init_ms };
    reinicios_.push_back(r);

    logf(LogLevel::INFO, "BOOT", "ResetReason=%d init=%lums", (int)r.razon, (unsigned long)duracion_init_ms);
}

void Logger::log(LogLevel nivel, const __FlashStringHelper* categoria, const __FlashStringHelper* msg) {
    if (!iniciado_ || nivel < config_.nivel_minimo || config_.modo == LogMode::DESHABILITADO) return;
    LineaLog l{ millis(), nivel, String((const __FlashStringHelper*)categoria), String((const __FlashStringHelper*)msg) };
    escribirLinea(l);
}

void Logger::logf(LogLevel nivel, const char* categoria, const char* fmt, ...) {
    if (!iniciado_ || nivel < config_.nivel_minimo || config_.modo == LogMode::DESHABILITADO) return;
    char buffer[256];
    va_list ap; va_start(ap, fmt); vsnprintf(buffer, sizeof(buffer), fmt, ap); va_end(ap);
    LineaLog l{ millis(), nivel, String(categoria), String(buffer) };
    escribirLinea(l);
}

void Logger::escribirLinea(const LineaLog& l) {
    // Guardar en buffer circular en RAM
    if (buffer_reciente_.size() >= BUFFER_MAX) {
        buffer_reciente_.erase(buffer_reciente_.begin());
    }
    buffer_reciente_.push_back(l);

    // Formatear línea
    char linea[320];
    snprintf(linea, sizeof(linea), "%lu [%s] (%s) %s\n", (unsigned long)l.ts_ms, nivelToString(l.nivel), l.categoria.c_str(), l.mensaje.c_str());

    // Serial
    if (config_.modo == LogMode::SOLO_SERIAL || config_.modo == LogMode::SERIAL_Y_ARCHIVO) {
        Serial.print(linea);
    }
    // Archivo
    if ((config_.modo == LogMode::SOLO_ARCHIVO || config_.modo == LogMode::SERIAL_Y_ARCHIVO) && SPIFFS.begin()) {
        rotarSiNecesario();
        File f = SPIFFS.open(config_.ruta_archivo, FILE_APPEND);
        if (f) { f.print(linea); f.close(); }
    }
}

void Logger::rotarSiNecesario() {
    if (!SPIFFS.exists(config_.ruta_archivo)) return;
    File f = SPIFFS.open(config_.ruta_archivo, FILE_READ);
    if (!f) return;
    size_t tam = f.size();
    f.close();
    if (tam < config_.tam_max_archivo) return;
    // Rotación simple: renombrar log actual con sufijo .old y empezar nuevo
    String antiguo = String(config_.ruta_archivo) + ".old";
    if (SPIFFS.exists(antiguo)) SPIFFS.remove(antiguo);
    SPIFFS.rename(config_.ruta_archivo, antiguo);
}

void Logger::volcarEstado(Stream& out) const {
    out.println(F("--- ESTADO LOGGER ---"));
    out.printf("Reinicios registrados: %u\n", (unsigned)reinicios_.size());
    for (size_t i = 0; i < reinicios_.size(); ++i) {
        const auto& r = reinicios_[i];
        out.printf("  #%u razon=%d init=%lums\n", (unsigned)i, (int)r.razon, (unsigned long)r.duracion_init_ms);
    }
    out.printf("Lineas recientes: %u\n", (unsigned)buffer_reciente_.size());
}

const __FlashStringHelper* Logger::nivelToString(LogLevel n) const {
    switch(n) {
        case LogLevel::TRACE: return F("TRC");
        case LogLevel::DEBUG: return F("DBG");
        case LogLevel::INFO: return F("INF");
        case LogLevel::WARN: return F("WRN");
        case LogLevel::ERROR: return F("ERR");
        case LogLevel::CRITICAL: return F("CRT");
        default: return F("???");
    }
}

} // namespace monitoreo
