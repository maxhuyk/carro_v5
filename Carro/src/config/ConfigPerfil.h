/**
 * @file ConfigPerfil.h
 * @brief Estructuras agregadas de configuración de alto nivel (logging, motores, comms, uwb, control).
 */
#pragma once
#include <Arduino.h>
#include "ConfigLogging.h"
#include "ConfigMotores.h"

namespace config {

/** @brief Configuración de comunicaciones (ESP-NOW). */
struct CommsConfig {
    bool habilitar_espnow = true;
    uint8_t canal_wifi = 1; // puede ajustarse
};

/** @brief Configuración de la capa de ranging UWB de alto nivel. */
struct UwbConfig {
    bool habilitar_uwb = true;
    uint16_t intervalo_ms = 50; // frecuencia objetivo de medición
};

/** @brief Configuración de habilitación y parámetros de control manual/auto. */
struct ControlConfig {
    bool habilitar_control_auto = true;
    bool habilitar_control_manual = true;
    int velocidad_max = 75; // -100..100 lógica
    // Rango crudo de joystick y zona muerta (para ControlManual)
    int joy_raw_min = 600;
    int joy_raw_max = 3400;
    int joy_raw_center = 2000;
    int joy_deadzone = 250; // +/- alrededor de center
};

// Perfil completo que agrupa todos los sub-configs
/** @brief Perfil completo del sistema combinando todos los sub-módulos. */
struct ConfigPerfil {
    LoggingConfig logging = obtenerLoggingConfigDefault();
    MotorConfig   motores = obtenerMotorConfigDefault();
    CommsConfig   comms{};
    UwbConfig     uwb{};
    ControlConfig control{};
};

ConfigPerfil cargarConfigDefault(); // En futuro cargar desde NVS / archivo

} // namespace config
