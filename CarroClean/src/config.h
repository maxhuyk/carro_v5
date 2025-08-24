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
#define MEDIA_MOVIL_VENTANA 3

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
#define UMBRAL 10  // Aumentado de 5 a 8 para evitar correcciones constantes

// #################################################################
// PARÁMETROS DEL CONTROLADOR PID DE ÁNGULO
// #################################################################
#define PID_KP 0.7          // Reducido de 1.0 a 0.7 para correcciones más suaves
#define PID_KI 0.03         // Reducido de 0.08 a 0.03 para evitar oscilaciones
#define PID_KD 1.5          // Incrementado de 1.0 a 1.2 para mejor estabilidad
#define PID_SETPOINT 0.0
#define PID_ALPHA 0.3
#define PID_SALIDA_MAX 50.0 // Reducido de 40.0 a 15.0 para correcciones más suaves
#define INTEGRAL_MAX 50.0

// #################################################################
// CONTROL DE VELOCIDAD LINEAL POR DISTANCIA
// #################################################################
#define DISTANCIA_OBJETIVO 1500.0  // mm (1.5 metros)

// Parámetros del PID para control de distancia
#define PID_DISTANCIA_KP 0.1
#define PID_DISTANCIA_KI 0.03
#define PID_DISTANCIA_KD 0.08
#define PID_DISTANCIA_ALPHA 0.5
#define PID_DISTANCIA_INTEGRAL_MAX 50.0

// #################################################################
// CONTROL DE VELOCIDAD Y ACELERACIÓN
// #################################################################
#define VELOCIDAD_MAXIMA 45  // Reducido de 90 a 40 para seguimiento más suave
#define ACELERACION_MAXIMA 15  // Incrementado de 1 a 10 para respuesta más rápida

// #################################################################
// CONTROL MANUAL (MODO 3)
// #################################################################
#define VELOCIDAD_MANUAL 40

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
