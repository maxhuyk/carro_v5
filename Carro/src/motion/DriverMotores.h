#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "config/ConfigMotores.h"
#include "monitoreo/LoggerReinicios.h"
#include "MotionCommand.h"

namespace motion {

class DriverMotores : public core::Modulo {
public:
    explicit DriverMotores(const config::MotorConfig& cfg) : cfg_(cfg) {}

    bool iniciar() override;
    void actualizar() override;
    void detener() override;

    // Aplica un comando lógico (internamente aplica deadzone y mapping)
    void aplicar(const MotionCommand& cmd);

    // Detiene ambos motores inmediatamente
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
