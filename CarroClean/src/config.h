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
// UMBRAL DINÁMICO BASADO EN DISTANCIA
// #################################################################
#define UMBRAL_MINIMO 25.0       // Umbral a 1.5m (cerca) - más tolerante
#define UMBRAL_MAXIMO 10.0      // Umbral a 10m (lejos) - más estricto
#define DISTANCIA_UMBRAL_MIN 1500.0   // 1.5m en mm
#define DISTANCIA_UMBRAL_MAX 10000.0  // 10m en mm

// #################################################################
// PARÁMETROS DEL CONTROLADOR PID DE ÁNGULO
// #################################################################
#define PID_KP 0.25         // Reducido aún más para suavidad
#define PID_KI 0.01         // Reducido para evitar acumulación agresiva
#define PID_KD 0.8          // Reducido para menos reactividad
#define PID_SETPOINT 0.0
#define PID_ALPHA 0.5       // Mayor filtrado del error
#define PID_SALIDA_MAX  10.0 // MÁXIMA CORRECCIÓN POR CICLO: 3°/ciclo × 15Hz = 45°/s máximo
#define INTEGRAL_MAX 20.0   // Reducido para evitar windup

// Control de suavidad adicional
#define MAX_GIRO_POR_SEGUNDO 60.0  // Máximo 60°/segundo de corrección total
#define FACTOR_SUAVIZADO_GIRO 0.7  // Factor de suavizado para transiciones (0.0-1.0)

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
#define VELOCIDAD_MAXIMA 60  // Incrementado de 45 a 70 para mejor velocidad de seguimiento
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
