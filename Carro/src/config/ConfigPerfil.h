#pragma once
#include <Arduino.h>
#include "ConfigLogging.h"
#include "ConfigMotores.h"

namespace config {

struct CommsConfig {
    bool habilitar_espnow = true;
    uint8_t canal_wifi = 1; // puede ajustarse
};

struct UwbConfig {
    bool habilitar_uwb = true;
    uint16_t intervalo_ms = 50; // frecuencia objetivo de medición
};

struct ControlConfig {
    bool habilitar_control_auto = true;
    bool habilitar_control_manual = true;
    int velocidad_max = 75; // -100..100 lógica
};

// Perfil completo que agrupa todos los sub-configs
struct ConfigPerfil {
    LoggingConfig logging = obtenerLoggingConfigDefault();
    MotorConfig   motores = obtenerMotorConfigDefault();
    CommsConfig   comms{};
    UwbConfig     uwb{};
    ControlConfig control{};
};

ConfigPerfil cargarConfigDefault(); // En futuro cargar desde NVS / archivo

} // namespace config
