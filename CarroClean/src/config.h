#ifndef CONFIG_H
#define CONFIG_H

// #################################################################
// COMUNICACIÓN UART
// #################################################################
#define DATA_PORT "/dev/ttyAMA0"  // Placeholder - será diferente en ESP32
#define BAUDRATE 500000

// #################################################################
// CONFIGURACIÓN DE SENSORES
// #################################################################
#define N_SENSORES 3

// #################################################################
// TIEMPO DE ESPERA
// #################################################################
#define TIEMPO_ESPERA 0.01  // 10ms

// #################################################################
// PASO 2: FILTRO DE MEDIA MOVIL
// #################################################################
#define MEDIA_MOVIL_VENTANA 2

// #################################################################
// PASO 3: FILTRO KALMAN
// #################################################################
#define KALMAN_Q 1e-2
#define KALMAN_R 0.1

// #################################################################
// LÍMITES Y DELTAS
// #################################################################
#define MAX_DELTA_POS 100   // Delta para las posiciones iniciales antes de media
#define MAX_DELTA 10        // Delta del ángulo

// #################################################################
// UMBRAL A PARTIR DEL CUAL SE CORRIGE
// #################################################################
#define UMBRAL 10

// #################################################################
// PARÁMETROS DEL CONTROLADOR PID DE ÁNGULO
// #################################################################
#define PID_KP 2.0
#define PID_KI 0.05
#define PID_KD 1.0
#define PID_SETPOINT 0.0
#define PID_ALPHA 0.3
#define PID_SALIDA_MAX 30.0
#define INTEGRAL_MAX 50.0

// #################################################################
// CONTROL DE VELOCIDAD LINEAL POR DISTANCIA
// #################################################################
#define DISTANCIA_OBJETIVO 1500.0  // mm (1.5 metros)

// Parámetros del PID para control de distancia
#define PID_DISTANCIA_KP 0.2
#define PID_DISTANCIA_KI 0.03
#define PID_DISTANCIA_KD 0.08
#define PID_DISTANCIA_ALPHA 0.7
#define PID_DISTANCIA_INTEGRAL_MAX 200.0

// #################################################################
// CONTROL DE VELOCIDAD Y ACELERACIÓN
// #################################################################
#define VELOCIDAD_MAXIMA 60
#define ACELERACION_MAXIMA 3  // Máximo cambio de velocidad por ciclo

// #################################################################
// CONTROL MANUAL (MODO 3)
// #################################################################
#define VELOCIDAD_MANUAL 30

// #################################################################
// CONTROL DE MOVIMIENTO SIN SEÑAL
// #################################################################
#define DISTANCIA_FALLBACK 2500
#define VELOCIDAD_FALLBACK 15
#define TIMEOUT_FALLBACK 1.0

// #################################################################
// NÚMERO DE CICLOS Y CONFIGURACIÓN
// #################################################################
#define NUM_CICLOS 100
#define DISTANCIA_MINIMA_PARADA 0.9

// Posiciones de sensores (matriz 3x3)
extern const float SENSOR_POSICIONES[3][3];

#endif // CONFIG_H
