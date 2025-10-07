#pragma once
#include <Arduino.h>

// ===== Migración literal de calculos.h (CarroClean) =====

// Media móvil
void media_movil_init(int n_sensores, int ventana);
void media_movil_actualizar(float* nuevos_valores, float* resultado);

// Kalman
void kalman_init(int n_sensores, float Q, float R);
void kalman_filtrar(float* mediciones, float* resultado);
void kalman_force_state(float* mediciones, float P_init);
void kalman_set_Q(float Q);
float kalman_get_Q();

// PID
struct PIDController { float kp, ki, kd, setpoint, alpha, error_anterior, integral, salida_maxima, integral_maxima; };
void pid_init(PIDController* pid, float kp, float ki, float kd, float setpoint, float alpha, float salida_maxima, float integral_maxima);
float pid_update(PIDController* pid, float medida_actual);

// Variación
void limitar_variacion(float* dist_actual, float* dist_anterior, float* resultado, int n_sensores, float max_delta);

// Trilateración
void trilateracion_3d(const float sensor_posiciones[3][3], float* distancias, float* resultado);

// Ángulos
enum { ANG_OK=0 };
float angulo_direccion_xy(float* pos_tag);
float desenrollar_angulo(float previo, float actual);
float limitar_cambio(float previo, float actual, float max_delta);

bool debe_corregir(float angulo, float umbral);
float calcular_umbral_dinamico(float distancia_mm, float umbral_min, float umbral_max, float dist_min, float dist_max);

float suavizar_velocidad_avanzada(float velocidad_actual, float velocidad_objetivo, float aceleracion_maxima, float desaceleracion_maxima);

// Diferencial
struct VelocidadesDiferenciales { int vel_izq; int vel_der; float giro_normalizado; };
VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lineal, float angulo_relativo, float max_v);
