#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "config/ConfigMotores.h"
#include "monitoreo/LoggerReinicios.h"
#include "MotionCommand.h"

namespace motion {

/**
 * Driver de motores (dual H-bridge vía DRV8701 equivalente):
 *  - Aplica deadzone porcentual (cfg_.deadzone_percent)
 *  - Mapea -100..100 a PWM 0..(2^res-1)
 *  - Gestiona ENABLE y timeout de seguridad
 */
class DriverMotores : public core::Modulo {
public:
    explicit DriverMotores(const config::MotorConfig& cfg) : cfg_(cfg) {}

    bool iniciar() override;
    void actualizar() override;
    void detener() override;

    /** Aplica un comando lógico (aplica deadzone + mapping PWM). */
    void aplicar(const MotionCommand& cmd);

    /** Detiene ambos motores inmediatamente y pone ENABLE LOW. */
    void stop();

private:
    config::MotorConfig cfg_;
    bool pwm_ok_ = false;
    unsigned long last_cmd_ms_ = 0;
    motion::MotionCommand ultimo_{}; // se inicializa en iniciar()

    // Helpers
    void configurarPWM();
    void setMotorL(int pwm_percent, bool dir);
    void setMotorR(int pwm_percent, bool dir);
};

} // namespace motion
