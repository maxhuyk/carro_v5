/**
 * @file ControlManual.h
 * @brief Módulo que aplica comandos manuales recibidos por ESP-NOW a los motores.
 *
 * Responsable de:
 *  - Mapear ejes crudos de joystick (según ConfigPerfil::ControlConfig) a -100..100
 *  - Aplicar zona muerta configurada y recorte de saturaciones
 *  - Generar un MotionCommand y delegar a DriverMotores
 */
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

    /**
     * @brief Callback invocado por EspNowReceiver al llegar un comando.
     * @param cmd Paquete crudo recibido (timestamps y ejes raw)
     */
    void onManualCommand(const comms::ManualCommand& cmd);

private:
    config::ControlConfig cfg_;
    motion::DriverMotores& driver_;
};

} // namespace control
