#ifndef CONFIG_H
#define CONFIG_H

// Configuración general
#define NUM_CICLOS 1
#define TIEMPO_ESPERA 1
#define DATA_PORT 1
#define BAUDRATE 1
#define N_SENSORES 1
#define MEDIA_MOVIL_VENTANA 1

// Filtro Kalman
#define KALMAN_Q 1
#define KALMAN_R 1

// Límites y umbrales
#define MAX_DELTA 1
#define MAX_DELTA_POS 1
#define UMBRAL 1

// PID de ángulo
#define PID_KP 1
#define PID_KI 1
#define PID_KD 1
#define PID_SETPOINT 1
#define PID_ALPHA 1
#define PID_SALIDA_MAX 1
#define INTEGRAL_MAX 1

// Control de distancia
#define DISTANCIA_OBJETIVO 1
#define PID_DISTANCIA_KP 1
#define PID_DISTANCIA_KI 1
#define PID_DISTANCIA_KD 1
#define PID_DISTANCIA_ALPHA 1
#define PID_DISTANCIA_INTEGRAL_MAX 1

// Control de velocidad
#define VELOCIDAD_MAXIMA 1
#define ACELERACION_MAXIMA 1

// Posiciones de sensores (matriz 3x3)
extern const float SENSOR_POSICIONES[3][3];

#endif // CONFIG_H
