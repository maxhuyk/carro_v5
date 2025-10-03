#include "FollowController.h"
#include <cmath>

namespace control {

bool FollowController::iniciar() {
    pid_init(pid_ang_, cfg_.pid_ang_kp, cfg_.pid_ang_ki, cfg_.pid_ang_kd, 0.0f, cfg_.pid_ang_alpha, cfg_.pid_ang_salida_max, cfg_.pid_ang_integral_max);
    pid_init(pid_dist_, cfg_.pid_dist_kp, cfg_.pid_dist_ki, cfg_.pid_dist_kd, cfg_.distancia_objetivo, cfg_.pid_dist_alpha, cfg_.velocidad_maxima, cfg_.pid_dist_integral_max);
    LOG_INFO("AUTO", "FollowController iniciado");
    return true;
}

void FollowController::pushMeasurement(const Measurement& m) {
    last_ = m; has_measurement_ = true; }

float FollowController::limitar_cambio(float anterior, float nuevo, float max_delta) {
    float delta = nuevo - anterior;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    return anterior + delta;
}

float FollowController::calcular_umbral_dinamico(float d) {
    if (d <= cfg_.distancia_umbral_min) return cfg_.umbral_minimo;
    if (d >= cfg_.distancia_umbral_max) return cfg_.umbral_maximo;
    float frac = (d - cfg_.distancia_umbral_min) / (cfg_.distancia_umbral_max - cfg_.distancia_umbral_min);
    return cfg_.umbral_minimo + (cfg_.umbral_maximo - cfg_.umbral_minimo) * frac;
}

bool FollowController::debe_corregir(float angulo_relativo, float umbral) {
    return fabsf(angulo_relativo) > umbral; }

float FollowController::suavizar_velocidad(float actual, float objetivo, float acel, float desacel) {
    if (objetivo > actual) {
        float next = actual + acel; if (next > objetivo) next = objetivo; return next; }
    else if (objetivo < actual) {
        float next = actual - desacel; if (next < objetivo) next = objetivo; return next; }
    return actual;
}

DiffVel FollowController::calcular_velocidades_diferenciales(int v_lineal, float correccion, int max_v) {
    float vL = v_lineal - correccion; float vR = v_lineal + correccion;
    // Normalizar si exceden
    float m = fmaxf(fabsf(vL), fabsf(vR));
    if (m > max_v && m > 0.0f) { vL = vL * max_v / m; vR = vR * max_v / m; }
    return { vL, vR, correccion };
}

void FollowController::actualizar() {
    if (!has_measurement_) return;

    // Distancias base / validación mínima
    float d0 = last_.distancias[0]; float d1 = last_.distancias[1]; float d2 = last_.distancias[2];
    if (d0 <= 0 || d1 <= 0) {
        LOG_WARN("AUTO", "Distancias invalidas d0=%.1f d1=%.1f", d0, d1); return; }

    // Distancia promedio frontal
    float distancia = (d0 + d1) / 2.0f;

    // Para ser fieles a CarroClean usaríamos trilateración + desenrollado + limitar y luego PID.
    // Aquí mantenemos placeholder: diferencia de sensores escalada pequeña.
    float angulo_rel = (d0 - d1) * 0.05f;
    angulo_rel = limitar_cambio(0.0f, angulo_rel, cfg_.max_delta_angulo);

    float umbral = calcular_umbral_dinamico(distancia);
    float correccion = 0.0f;
    if (debe_corregir(angulo_rel, umbral)) {
        correccion = pid_update(pid_ang_, angulo_rel);
    }

    // PID distancia -> velocidad objetivo lineal
    float _ = pid_update(pid_dist_, distancia); // no usamos salida directa, seguimos heurística original
    float error_dist = distancia - cfg_.distancia_objetivo;
    int velocidad_objetivo;
    if (error_dist > 500) velocidad_objetivo = cfg_.velocidad_maxima;
    else if (error_dist > 100) {
        float vp = 20 + (error_dist - 100) * 0.016f;
        if (vp > cfg_.velocidad_maxima) vp = (float)cfg_.velocidad_maxima;
        velocidad_objetivo = (int)vp;
    } else if (error_dist < 50) velocidad_objetivo = 0;
    else velocidad_objetivo = (int)constrain(8 + error_dist * 0.05f, 0.0f, 25.0f);

    velocidad_actual_ = suavizar_velocidad(velocidad_actual_, (float)velocidad_objetivo, cfg_.aceleracion_max, cfg_.desaceleracion_max);
    int v_lineal = (int)velocidad_actual_;

    if (modo_ == 3) { // manual override
        // En este follow controller, si modo manual, se ignora autopilot
        return; }

    if (modo_ != 1) return; // Sólo autopilot en modo 1

    if (v_lineal > 0) {
        auto dv = calcular_velocidades_diferenciales(v_lineal, correccion, cfg_.velocidad_maxima);
        motion::MotionCommand mc; mc.left = (int16_t)dv.velL; mc.right = (int16_t)dv.velR; mc.source = motion::MotionCommand::Source::AUTOPILOT;
        driver_.aplicar(mc);
        LOG_DEBUG("AUTO", "dist=%.0f err=%.0f v=%d corr=%.2f L=%d R=%d", distancia, error_dist, v_lineal, correccion, (int)dv.velL, (int)dv.velR);
    } else {
        // velocidad = 0 -> frenar
        motion::MotionCommand mc; mc.left = 0; mc.right = 0; mc.source = motion::MotionCommand::Source::AUTOPILOT;
        driver_.aplicar(mc);
    }
}

} // namespace control
