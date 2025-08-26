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
#define TIEMPO_ESPERA 0.0 // 10ms

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
#define MAX_DELTA 6        // Delta del ángulo

// #################################################################
// RECUPERACIÓN RÁPIDA TRAS PÉRDIDA DE SEÑAL
// #################################################################
// Tiempo (ms) durante el cual se relaja el limitador de variación tras recuperar
#define RECOVERY_GRACE_MS 700
// Número de ciclos mínimos tras recuperación antes de volver a parámetros normales
#define RECOVERY_K_CYCLES 20
// Covarianza inicial grande para forzar rápida adaptación del Kalman al nuevo estado
#define RECOVERY_P_INIT 1e6
// Q temporal alto para permitir que el Kalman siga las primeras mediciones sin inercia
#define RECOVERY_Q_TEMP 1.0f
// Si se desea permitir limitador con delta ampliado en lugar de saltarlo, usar este valor
#define RECOVERY_DELTA_POS 1000

// #################################################################
// UMBRAL DINÁMICO BASADO EN DISTANCIA
// #################################################################
#define UMBRAL_MINIMO 25.0       // Umbral a 1.5m (cerca) - más tolerante
#define UMBRAL_MAXIMO 10.0      // Umbral a 10m (lejos) - más estricto
#define DISTANCIA_UMBRAL_MIN 1500.0   // 1.5m en mm
#define DISTANCIA_UMBRAL_MAX 10000.0  // 10m en mm

// #################################################################
// PARÁMETROS DEL CONTROLADOR PID DE ÁNGULO ADAPTATIVO
// #################################################################
#define PID_KP 0.25         // Base KP para velocidad media
#define PID_KI 0.01         // Base KI
#define PID_KD 0.8          // Base KD
#define PID_SETPOINT 0.0
#define PID_ALPHA 0.5       // Mayor filtrado del error
#define PID_SALIDA_MAX 10.0 // Máxima salida del PID
// PID ADAPTATIVO POR VELOCIDAD
#define PID_SALIDA_MAX_BAJA_VEL 15.0   // Máxima corrección a velocidad baja (agresivo)
#define PID_SALIDA_MAX_ALTA_VEL 3.0    // Máxima corrección a velocidad alta (suave)
#define VELOCIDAD_UMBRAL_BAJA 10       // Velocidad considerada "baja" para PID agresivo
#define VELOCIDAD_UMBRAL_ALTA 40       // Velocidad considerada "alta" para PID suave

#define INTEGRAL_MAX 20.0   // Reducido para evitar windup


// #################################################################
// CONTROL DE VELOCIDAD LINEAL POR DISTANCIA
// #################################################################
#define DISTANCIA_OBJETIVO 1500.0  // mm (1.5 metros)

// Parámetros del PID para control de distancia
#define PID_DISTANCIA_KP 0.3      // Incrementado de 0.1 a 0.3 para respuesta más fuerte
#define PID_DISTANCIA_KI 0.05     // Incrementado de 0.03 a 0.05 para eliminar error residual
#define PID_DISTANCIA_KD 0.15     // Incrementado de 0.08 a 0.15 para mejor estabilidad
#define PID_DISTANCIA_ALPHA 0.5
#define PID_DISTANCIA_INTEGRAL_MAX 100.0  // Incrementado de 50.0 a 100.0

// #################################################################
// CONTROL DE VELOCIDAD Y ACELERACIÓN
// #################################################################
#define VELOCIDAD_MAXIMA 75  // Incrementado de 45 a 70 para mejor velocidad de seguimiento
#define ACELERACION_MAXIMA 10  // Para acelerar (arranque suave)
#define DESACELERACION_MAXIMA 35  // Para frenar (frenado más rápido)

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
