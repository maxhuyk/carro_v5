#pragma once
#include <Arduino.h>

namespace config {

// Configuración específica de los motores / driver
struct MotorConfig {
    // Pines (DRV8701 o similar)
    int pin_enable = 14;
    int pin_l1 = 26;
    int pin_l2 = 25;
    int pin_r1 = 33;
    int pin_r2 = 32;

    // PWM
    uint32_t pwm_freq = 20000;  // 20 kHz
    uint8_t pwm_res_bits = 8;   // 0..255

    // Deadzone en porcentaje (0..100) aplicada a comandos distintos de cero
    uint8_t deadzone_percent = 60; // asegura torque inicial

    // Nivel mínimo de comando aceptado (|valor| < este => 0) en escala -100..100
    uint8_t umbral_comando = 2;

    // Tiempo (ms) tras el cual se detienen motores si no hay nuevos comandos
    uint32_t timeout_ms = 2000;

    // Habilitar logs detallados
    bool log_detallado = true;
};

MotorConfig obtenerMotorConfigDefault();

} // namespace config
