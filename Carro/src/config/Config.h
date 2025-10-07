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

// Control distancia (objetivo usado para perfil lineal)
#define DISTANCIA_OBJETIVO 1500.0

// Velocidades y aceleraciones
#define VELOCIDAD_MAXIMA 75
#define VELOCIDAD_MANUAL 40
#define ACELERACION_MAXIMA 10
#define DESACELERACION_MAXIMA 35

// Velocidad máxima en modo emergencia (escala 0..100 usada por motor_enviar_pwm)
#define EMERGENCY_MAX_SPEED 25
// Margen muerto adicional dentro del modo emergencia para evitar movimiento con pot al mínimo
// Valor en unidades RAW ADC (0..4095). Debe ser >0. Ej: 120 ~ ~0.1V aprox.
#define EMERGENCY_DEADZONE_RAW 60
// Umbral mínimo (0..100) debajo del cual NO se envía ningún comando (evita que 1..4% activen el torque mínimo)
#define EMERGENCY_MIN_COMMAND 3


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
