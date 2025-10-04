#include "FollowController.h"
#include <cmath>

namespace control {

bool FollowController::iniciar() {
    pid_init(pid_ang_, cfg_.pid_ang_kp, cfg_.pid_ang_ki, cfg_.pid_ang_kd, 0.0f, cfg_.pid_ang_alpha, cfg_.pid_ang_salida_max, cfg_.pid_ang_integral_max);
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
    // Validaciones como en CarroClean (rango y >0)
    if (isnan(d0) || isnan(d1) || isnan(d2) || d0 <= 0 || d1 <= 0 || d2 <= 0 || d0 > 40000 || d1 > 40000 || d2 > 40000) {
        LOG_WARN("AUTO", "Dist invalidas d0=%.1f d1=%.1f d2=%.1f", d0, d1, d2);
        // No se hace pipeline; safety gestionará por timeout si persiste
        return;
    }

    // ===== PIPELINE FILTROS (orden CarroClean) =====
    // 1. Limitador de variación sobre distancias crudas (si hay previas y no gracia recovery)
    float dist_work[3] = {d0,d1,d2};
    if (dist_prev_valid_) {
        bool saltar_limitador = false;
        if (in_recovery_) {
            unsigned long elapsed = millis() - recovery_start_ms_;
            if (elapsed < cfg_.recovery_grace_ms) saltar_limitador = true;
        }
        if (!saltar_limitador) {
            float dist_lim[3];
            utils::limitar_variacion(dist_work, dist_prev_, dist_lim, 3, in_recovery_ ? cfg_.recovery_delta_pos : cfg_.max_delta_pos);
            for(int i=0;i<3;i++) dist_work[i] = dist_lim[i];
        }
    }
    // 2. Media móvil
    float dist_mv[3]; media_.actualizar(dist_work, dist_mv);
    // 3. Kalman
    float dist_kal[3]; kalman_.filtrar(dist_mv, dist_kal);
    // Actualizar previas para próximo ciclo
    for(int i=0;i<3;i++) dist_prev_[i] = dist_work[i]; dist_prev_valid_ = true;

    // Trilateración
    // (Por ahora dependemos de posiciones definidas externamente; placeholder simple si alguna distancia <=0)
    float pos_tag[3] = {0,0,0};
    bool dist_validas = (dist_kal[0]>0 && dist_kal[1]>0 && dist_kal[2]>0);
    if (dist_validas) {
        utils::trilateracion_3d(cfg_.anchor_pos, dist_kal, pos_tag);
    }

    // Ángulo con desenrollado y limitador
    float angulo_act = dist_validas ? utils::angulo_direccion_xy(pos_tag) : 0.0f;
    float angulo_desen = utils::desenrollar_angulo(angulo_prev_, angulo_act);
    float angulo_rel = utils::limitar_cambio(angulo_prev_, angulo_desen, cfg_.max_delta_angulo);
    angulo_prev_ = angulo_rel;

    // Distancia promedio frontal (usamos distancias filtradas para control de distancia)
    float distancia = (dist_kal[0] + dist_kal[1]) * 0.5f; // promedio frontal sin filtro extra

    // ====================== SEGURIDAD / SIGNAL LOST ======================
    bool early_return = false;
    manejarSignalLost(dist_kal, distancia, 0.0f /*correccion prelim*/, early_return);
    if (early_return) return; // safety tomó control

    float umbral = calcular_umbral_dinamico(distancia);
    float correccion = 0.0f;
    if (debe_corregir(angulo_rel, umbral)) {
        correccion = pid_update(pid_ang_, angulo_rel);
    }

    float error_dist = distancia - cfg_.distancia_objetivo;
    int velocidad_objetivo;
    if (error_dist > 5000) velocidad_objetivo = cfg_.velocidad_maxima; // umbral CarroClean
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

    // Diferencial CarroClean: nunca invertir; usa correccion como angulo_relativo
    auto velDiff = utils::calcular_velocidades_diferenciales((float)v_lineal, correccion, (float)cfg_.velocidad_maxima);
    int outL = velDiff.vel_izq; int outR = velDiff.vel_der;
    if (v_lineal > 0 && modo_ == 1) {
        motion::MotionCommand mc; mc.left = (int16_t)outL; mc.right = (int16_t)outR; mc.source = motion::MotionCommand::Source::AUTOPILOT;
        driver_.aplicar(mc);
        LOG_DEBUG("AUTO", "dist=%.0f err=%.0f v=%d corr=%.2f L=%d R=%d", distancia, error_dist, v_lineal, correccion, outL, outR);
    } else if (modo_ == 1) {
        motion::MotionCommand mc; mc.left = 0; mc.right = 0; mc.source = motion::MotionCommand::Source::AUTOPILOT;
        driver_.aplicar(mc);
    }
    // Guardar siempre (aun con velocidad 0) para fallback fiel
    last_valid_distance_ = distancia;
    last_valid_correction_ = correccion;
    last_valid_velocity_ = v_lineal;
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
            // Reset integrales para evitar sobrecorrecciones tras pérdida
            pid_ang_.integral = 0; pid_ang_.prev_error = 0; pid_ang_.first = true;
            // (No PID distancia que resetear)
            velocidad_actual_ = 0;
            // Rellenar media móvil con la primera medición (replicar comportamiento CarroClean)
            media_.init(3, cfg_.media_movil_ventana);
            float seed[3] = {distancias_filtradas[0], distancias_filtradas[1], distancias_filtradas[2]};
            float dummy[3];
            for (int r=0; r<cfg_.media_movil_ventana; ++r) media_.actualizar(seed, dummy);
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
