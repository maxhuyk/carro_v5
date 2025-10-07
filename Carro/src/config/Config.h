#pragma once

// Configuración global unificada
// TODO: Si hay otros parámetros de módulos (UWB, ESP-NOW, etc.) se agregan aquí.

// ===================== LOGGING =====================
#define CONFIG_LOG_LEVEL_DEFAULT 3 // 0=TRACE 1=DEBUG 2=INFO 3=WARN 4=ERROR

// ===================== ENABLE/DISABLE MÓDULOS =====================
#define CONFIG_ENABLE_MODULE_ESPNOW   1
#define CONFIG_ENABLE_MODULE_MOTOR    1
#define CONFIG_ENABLE_MODULE_EMERGENCY 1
#define CONFIG_ENABLE_MODULE_UWB      1
#define CONFIG_ENABLE_MODULE_CONTROL  1

// #################################################################
// CONFIGURACIÓN DE CONTROL (migrada de CarroClean config.h)
// #################################################################
#define N_SENSORES 3

// Media móvil
#define MEDIA_MOVIL_VENTANA 3

// Kalman
#define KALMAN_Q 1e-2
#define KALMAN_R 0.1

// Límites y deltas
#define MAX_DELTA_POS 100
#define MAX_DELTA 6

// Recuperación rápida
#define RECOVERY_GRACE_MS 700
#define RECOVERY_K_CYCLES 20
#define RECOVERY_P_INIT 1e6
#define RECOVERY_Q_TEMP 1.0f
#define RECOVERY_DELTA_POS 1000

// Umbral dinámico
#define UMBRAL_MINIMO 25.0
#define UMBRAL_MAXIMO 10.0
#define DISTANCIA_UMBRAL_MIN 1500.0
#define DISTANCIA_UMBRAL_MAX 10000.0

// PID ángulo (valores originales literal CarroClean: 1.2, 0.01, 0.3)
#define PID_KP 1.2
#define PID_KI 0.01
#define PID_KD 0.3
#define PID_SETPOINT 0.0
#define PID_ALPHA 0.5
#define PID_SALIDA_MAX 10.0
#define INTEGRAL_MAX 20.0

// Control distancia
#define DISTANCIA_OBJETIVO 1500.0
#define PID_DISTANCIA_KP 0.3
#define PID_DISTANCIA_KI 0.05
#define PID_DISTANCIA_KD 0.15
#define PID_DISTANCIA_ALPHA 0.5
#define PID_DISTANCIA_INTEGRAL_MAX 100.0

// Velocidades y aceleraciones
#define VELOCIDAD_MAXIMA 75
#define ACELERACION_MAXIMA 10
#define DESACELERACION_MAXIMA 35

// Control manual
#define VELOCIDAD_MANUAL 40

// Movimiento sin señal
#define DISTANCIA_FALLBACK 2500
#define VELOCIDAD_FALLBACK 15
#define TIMEOUT_FALLBACK 1.0

// Posiciones sensores (migradas literal de CarroClean config.cpp)
static const float SENSOR_POSICIONES[3][3] = {
    {280.0, 0.0, 0.0},     // Sensor 0: izquierda
    {-280.0, 0.0, 0.0},    // Sensor 1: derecha
    {-165.0, -270.0, 0.0}  // Sensor 2: frente-derecha
};
