#ifndef CALCULOS_H
#define CALCULOS_H

// Funciones para media móvil
void media_movil_init(int n_sensores, int ventana);
void media_movil_actualizar(float* nuevos_valores, float* resultado);

// Funciones para filtro Kalman
void kalman_init(int n_sensores, float Q, float R);
void kalman_filtrar(float* mediciones, float* resultado);

// Estructura para controlador PID
struct PIDController {
    float kp, ki, kd;
    float setpoint;
    float alpha;
    float error_anterior;
    float integral;
    float salida_maxima;
    float integral_maxima;
};

// Funciones para controlador PID
void pid_init(PIDController* pid, float kp, float ki, float kd, float setpoint, float alpha, float salida_maxima, float integral_maxima);
float pid_update(PIDController* pid, float medida_actual);

// Función para limitar variación de distancias
void limitar_variacion(float* dist_actual, float* dist_anterior, float* resultado, int n_sensores, float max_delta);

// Función para trilateración 3D
void trilateracion_3d(const float sensor_posiciones[3][3], float* distancias, float* resultado);

// Funciones para cálculo y manejo de ángulos
float angulo_direccion_xy(float* pos_tag);
float desenrollar_angulo(float previo, float actual);
float limitar_cambio(float previo, float actual, float max_delta);

// Función de validación/umbral
bool debe_corregir(float angulo, float umbral);

// Función para suavizar velocidad (control de aceleración)
float suavizar_velocidad(float velocidad_actual, float velocidad_objetivo, float aceleracion_maxima);

// Estructura para resultado de velocidades diferenciales
struct VelocidadesDiferenciales {
    int vel_izq;
    int vel_der;
    float giro_normalizado;
};

// Función para calcular velocidades diferenciales
VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lineal, float angulo_relativo, float max_v);

#endif
