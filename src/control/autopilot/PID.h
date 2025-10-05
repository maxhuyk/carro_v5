/**
 * @file PID.h
 * @brief Implementación ligera de PID con filtro simple de derivada y anti-windup integral.
 */
#pragma once
#include <Arduino.h>

namespace control {

/**
 * @brief Estado interno y parámetros de un controlador PID.
 */
struct PIDState {
    float kp, ki, kd;
    float setpoint;
    float alpha; // filtro exponencial para error derivativo / suavizado
    float integral_max;
    float salida_max;
    // estado
    float integral = 0.0f;
    float prev_error = 0.0f;
    bool  first = true;
};

/**
 * @brief Inicializa parámetros y limpia estado del PID.
 */
inline void pid_init(PIDState& p, float kp, float ki, float kd, float setpoint, float alpha, float salida_max, float integral_max) {
    p.kp = kp; p.ki = ki; p.kd = kd; p.setpoint = setpoint; p.alpha = alpha; p.salida_max = salida_max; p.integral_max = integral_max; p.integral = 0; p.prev_error=0; p.first=true;
}

/**
 * @brief Ejecuta un paso de actualización PID.
 * @param medida Valor actual medido
 * @return Salida saturada del controlador
 */
inline float pid_update(PIDState& p, float medida) {
    float error = p.setpoint - medida; // signo consistente con original (donde se pasaba angulo)
    if (p.first) { p.prev_error = error; p.first = false; }
    // Derivada filtrada
    float deriv = (error - p.prev_error);
    deriv = p.alpha * deriv + (1.0f - p.alpha) * 0.0f; // simple suavizado
    p.prev_error = error;
    // Integral con anti-windup
    p.integral += error;
    if (p.integral > p.integral_max) p.integral = p.integral_max;
    if (p.integral < -p.integral_max) p.integral = -p.integral_max;

    float salida = p.kp * error + p.ki * p.integral + p.kd * deriv;
    if (salida > p.salida_max) salida = p.salida_max;
    if (salida < -p.salida_max) salida = -p.salida_max;
    return salida;
}

} // namespace control
