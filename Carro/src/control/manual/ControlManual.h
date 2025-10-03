#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "comms/espnow/EspNowReceiver.h"
#include "motion/DriverMotores.h"
#include "config/ConfigPerfil.h"
#include "monitoreo/LoggerReinicios.h"

namespace control {

class ControlManual : public core::Modulo {
public:
    ControlManual(const config::ControlConfig& cfg, motion::DriverMotores& driver)
        : cfg_(cfg), driver_(driver) {}

    bool iniciar() override { return true; }
    void actualizar() override {}

    void onManualCommand(const comms::ManualCommand& cmd);

private:
    config::ControlConfig cfg_;
    motion::DriverMotores& driver_;
};

} // namespace control
