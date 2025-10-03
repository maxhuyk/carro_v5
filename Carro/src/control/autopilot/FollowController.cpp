#include "FollowController.h"
#include <cmath>

namespace control {

bool FollowController::iniciar() {
    pid_init(pid_ang_, cfg_.pid_ang_kp, cfg_.pid_ang_ki, cfg_.pid_ang_kd, 0.0f, cfg_.pid_ang_alpha, cfg_.pid_ang_salida_max, cfg_.pid_ang_integral_max);
    pid_init(pid_dist_, cfg_.pid_dist_kp, cfg_.pid_dist_ki, cfg_.pid_dist_kd, cfg_.distancia_objetivo, cfg_.pid_dist_alpha, cfg_.velocidad_maxima, cfg_.pid_dist_integral_max);
    media_.init(3, cfg_.media_movil_ventana);
    kalman_.init(3, cfg_.kalman_Q, cfg_.kalman_R);
    LOG_INFO("AUTO", "FollowController iniciado");
    return true;
}

void FollowController::pushMeasurement(const Measurement& m) {
    last_ = m; has_measurement_ = true; last_measurement_wall_ms_ = millis(); }

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

    // ===== PIPELINE FILTROS =====
    float dist_crudas[3] = {d0,d1,d2};
    float dist_mv[3]; media_.actualizar(dist_crudas, dist_mv);
    float dist_kal[3]; kalman_.filtrar(dist_mv, dist_kal);

    // Limitar variación (anti salto) excepto en recuperación gracia
    if (dist_prev_valid_) {
        bool saltar_limitador = false;
        if (in_recovery_) {
            unsigned long elapsed = millis() - recovery_start_ms_;
            if (elapsed < cfg_.recovery_grace_ms) saltar_limitador = true; else saltar_limitador = false;
        }
        if (!saltar_limitador) {
            float dist_limited[3];
            utils::limitar_variacion(dist_kal, dist_prev_, dist_limited, 3, in_recovery_ ? cfg_.recovery_delta_pos : cfg_.max_delta_pos);
            for (int i=0;i<3;i++) dist_kal[i] = dist_limited[i];
        }
    }
    for(int i=0;i<3;i++) dist_prev_[i]=dist_kal[i]; dist_prev_valid_=true;

    // Trilateración
    // (Por ahora dependemos de posiciones definidas externamente; placeholder simple si alguna distancia <=0)
    float pos_tag[3] = {0,0,0};
    bool dist_validas = (dist_kal[0]>0 && dist_kal[1]>0 && dist_kal[2]>0);
    if (dist_validas) {
        // Requiere posiciones de sensores - en esta fase se puede inyectar luego
        // Aquí asumimos un triángulo aproximado similar al original
        static const float SENSOR_POS[3][3] = { {280.0f,0,0}, {-280.0f,0,0}, {-165.0f,-270.0f,0} };
        utils::trilateracion_3d(SENSOR_POS, dist_kal, pos_tag);
    }

    // Ángulo con desenrollado y limitador
    float angulo_act = dist_validas ? utils::angulo_direccion_xy(pos_tag) : 0.0f;
    float angulo_desen = utils::desenrollar_angulo(angulo_prev_, angulo_act);
    float angulo_rel = utils::limitar_cambio(angulo_prev_, angulo_desen, cfg_.max_delta_angulo);
    angulo_prev_ = angulo_rel;

    // Distancia promedio frontal (usamos distancias filtradas para control de distancia)
    float distancia = (dist_kal[0] + dist_kal[1]) / 2.0f;

    // ====================== SEGURIDAD / SIGNAL LOST ======================
    bool early_return = false;
    manejarSignalLost(dist_kal, distancia, 0.0f /*correccion prelim*/, early_return);
    if (early_return) return; // safety tomó control

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

void FollowController::manejarSignalLost(float* distancias_filtradas, float distancia_promedio, float correccion, bool& early_return_flag) {
    // Determinar si hubo timeout de medición (p.ej. > 1000 ms sin nueva)
    unsigned long now = millis();
    bool data_fresh = (now - last_measurement_wall_ms_) < 1000; // criterio similar a CarroClean

    if (!data_fresh) {
        // SEÑAL PERDIDA
        if (!signal_lost_) {
            signal_lost_ = true;
            signal_lost_time_ = now;
            last_valid_distance_ = distancia_promedio;
            was_moving_when_lost_ = (velocidad_actual_ > 0);
            LOG_WARN("SAFETY", "SIGNAL_LOST ultima_dist=%.1f", last_valid_distance_);
        }

        if (last_valid_distance_ < cfg_.distancia_fallback) {
            // Cerca -> detener inmediatamente
            motion::MotionCommand mc; mc.left=0; mc.right=0; mc.source=motion::MotionCommand::Source::SAFETY;
            driver_.aplicar(mc);
            velocidad_actual_ = 0;
            LOG_INFO("SAFETY", "Cerca objetivo: stop inmediato");
            early_return_flag = true; return;
        } else {
            // Lejos: mantener durante timeout_fallback_s
            float elapsed_s = (now - signal_lost_time_) / 1000.0f;
            if (elapsed_s < cfg_.timeout_fallback_s && was_moving_when_lost_) {
                // Continuar con velocidad reducida fallback
                int vfb = cfg_.velocidad_fallback;
                if (vfb > 0) {
                    auto dv = calcular_velocidades_diferenciales(vfb, last_valid_correction_, cfg_.velocidad_maxima);
                    motion::MotionCommand mc; mc.left=(int16_t)dv.velL; mc.right=(int16_t)dv.velR; mc.source=motion::MotionCommand::Source::SAFETY;
                    driver_.aplicar(mc);
                    LOG_DEBUG("SAFETY", "Fallback continuando %.1fs restante", cfg_.timeout_fallback_s - elapsed_s);
                    early_return_flag = true; return;
                }
            } else {
                motion::MotionCommand mc; mc.left=0; mc.right=0; mc.source=motion::MotionCommand::Source::SAFETY;
                driver_.aplicar(mc);
                velocidad_actual_ = 0;
                LOG_INFO("SAFETY", "Fallback timeout -> stop");
                early_return_flag = true; return;
            }
        }
    } else {
        // SEÑAL VÁLIDA
        if (signal_lost_) {
            signal_lost_ = false;
            in_recovery_ = true; recovery_start_ms_ = now; recovery_cycles_ = 0;
            saved_normal_Q_ = kalman_.getQ();
            kalman_.setQ(cfg_.recovery_Q_temp);
            kalman_.forceState(distancias_filtradas, cfg_.recovery_P_init);
            LOG_INFO("RECOVERY", "Iniciada Q=%.3f->%.3f P=%.1f", saved_normal_Q_, cfg_.recovery_Q_temp, cfg_.recovery_P_init);
        }
        if (in_recovery_) {
            recovery_cycles_++;
            if (recovery_cycles_ >= cfg_.recovery_k_cycles && (now - recovery_start_ms_) > cfg_.recovery_grace_ms) {
                kalman_.setQ(saved_normal_Q_);
                in_recovery_ = false;
                LOG_INFO("RECOVERY", "Completada, Q restaurada=%.4f", saved_normal_Q_);
            }
        }
    }
}

} // namespace control
