#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "motion/DriverMotores.h"
#include "monitoreo/LogMacros.h"
#include "config/ConfigControl.h"
#include "control/autopilot/PID.h"
#include "utils/Filtros.h"

namespace control {

struct Measurement {
    float distancias[3];
    bool  anchor_ok[3];
    uint32_t count = 0;
    uint32_t ts_ms = 0;
};

// Resultado de cálculo diferencial
struct DiffVel { float velL; float velR; float giro_norm; };

class FollowController : public core::Modulo {
public:
    FollowController(const config::ControlTuning& cfg, motion::DriverMotores& driver)
    : cfg_(cfg), driver_(driver) {}

    bool iniciar() override;
    void actualizar() override;

    // Alimentar nueva medición (simulada hasta migrar UWB real)
    void pushMeasurement(const Measurement& m);

    // Set modo y velocidades manuales (para gating modo 1/3 similar a CarroClean)
    void setModo(uint8_t modo) {
        if (modo_ != modo) {
            // Reset de estados críticos al cambiar de modo
            pid_ang_.integral = 0; pid_ang_.prev_error = 0; pid_ang_.first = true;
            velocidad_actual_ = 0;
        }
        modo_ = modo;
    }
    void setManual(int vL, int vR) { manual_L_ = vL; manual_R_ = vR; }

private:
    config::ControlTuning cfg_;
    motion::DriverMotores& driver_;

    // Buffers
    Measurement last_; bool has_measurement_ = false;
    unsigned long last_measurement_wall_ms_ = 0; // para timeouts

    // Estados heredados
    bool signal_lost_ = false; unsigned long signal_lost_time_ = 0; bool in_recovery_ = false;
    float saved_normal_Q_ = 0; // placeholder
    unsigned long recovery_start_ms_ = 0; int recovery_cycles_ = 0;
    float last_valid_distance_ = 0; float last_valid_correction_ = 0; int last_valid_velocity_ = 0; bool was_moving_when_lost_ = false;
    float dist_prev_[3] = {0,0,0}; bool dist_prev_valid_ = false;
    float angulo_prev_ = 0.0f;

    // PID angular (único utilizado)
    PIDState pid_ang_{};

    // Velocidad suavizada
    float velocidad_actual_ = 0.0f;

    // Modo / manual
    uint8_t modo_ = 0; int manual_L_ = 0; int manual_R_ = 0;

    // Filtros
    utils::MediaMovilMulti media_;
    utils::KalmanMulti     kalman_;

    // Métodos auxiliares
    float limitar_cambio(float anterior, float nuevo, float max_delta);
    float calcular_umbral_dinamico(float distancia);
    bool debe_corregir(float angulo_relativo, float umbral);
    float suavizar_velocidad(float actual, float objetivo, float acel, float desacel);
    DiffVel calcular_velocidades_diferenciales(int v_lineal, float correccion, int max_v);
    void manejarSignalLost(float* distancias_filtradas, float distancia_promedio, float correccion, bool& early_return_flag);
};

} // namespace control
