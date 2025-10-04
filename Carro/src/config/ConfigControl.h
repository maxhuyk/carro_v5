/**
 * @file ConfigControl.h
 * @brief Parámetros de sintonía y límites de control (filtros, PID angular, heurísticas).
 */
#pragma once
#include <Arduino.h>

namespace config {

// Parámetros de control (portados desde CarroClean config.h)
/**
 * @brief Sintonía y constantes de control utilizadas por FollowController.
 */
struct ControlTuning {
    // Filtros
    uint8_t media_movil_ventana = 3;
    float   kalman_Q = 1e-2f;
    float   kalman_R = 0.1f;

    // Límites y deltas
    float max_delta_pos = 100.0f;
    float max_delta_angulo = 6.0f;

    // Recuperación rápida
    uint16_t recovery_grace_ms = 700;
    uint16_t recovery_k_cycles = 20;
    float    recovery_P_init = 1e6f;
    float    recovery_Q_temp = 1.0f;
    float    recovery_delta_pos = 1000.0f; // delta ampliado opcional

    // Umbral dinámico
    float umbral_minimo = 25.0f;    // cerca
    float umbral_maximo = 10.0f;    // lejos
    float distancia_umbral_min = 1500.0f;
    float distancia_umbral_max = 10000.0f;

    // PID angular base
    // En CarroClean (control.cpp) se inicializa pid_init(&pid, 1.2, 0.01, 0.3,...)
    // Se ignoran los macros de config.h originales (0.25,0.01,0.8). Adoptamos los efectivos.
    float pid_ang_kp = 1.2f;
    float pid_ang_ki = 0.01f;
    float pid_ang_kd = 0.3f;
    float pid_ang_alpha = 0.5f;
    float pid_ang_salida_max = 10.0f;
    float pid_ang_integral_max = 20.0f;

    // Distancia objetivo (se mantiene para heurística; PID distancia eliminado por no usarse)
    float distancia_objetivo = 1500.0f; // mm

    // Velocidades / aceleraciones (escala -100..100)
    int velocidad_maxima = 75;
    int velocidad_manual_max = 40;
    int velocidad_fallback = 15;
    float distancia_fallback = 2500.0f; // mm
    float timeout_fallback_s = 1.0f;
    float aceleracion_max = 10.0f;
    float desaceleracion_max = 35.0f;

    // Posiciones de anchors (mm) usadas en trilateración (migradas desde CarroClean)
    // A1 (derecha), A2 (izquierda), A3 (trasera) ejemplo layout triángulo
    float anchor_pos[3][3] = { {280.0f, 0.0f, 0.0f}, {-280.0f, 0.0f, 0.0f}, {-165.0f, -270.0f, 0.0f} };
};

ControlTuning obtenerControlTuningDefault();

} // namespace config
