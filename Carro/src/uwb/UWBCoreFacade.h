#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "monitoreo/LogMacros.h"
#include "control/autopilot/FollowController.h" // Para usar Measurement

namespace uwb {

struct UwbConfigFacade {
    uint16_t intervalo_ms = 50; // frecuencia objetivo
    bool log_periodico = true;
};

// Interfaz simplificada que simula mediciones UWB hasta integrar DW3000 real
class UWBCoreFacade : public core::Modulo {
public:
    UWBCoreFacade(const UwbConfigFacade& cfg, control::FollowController& follow)
    : cfg_(cfg), follow_(follow) {}

    bool iniciar() override {
        last_emit_ = millis();
        LOG_INFO("UWB", "Facade iniciado intervalo=%ums", cfg_.intervalo_ms);
        return true;
    }

    void actualizar() override {
        unsigned long now = millis();
        if (now - last_emit_ >= cfg_.intervalo_ms) {
            last_emit_ = now;
            // Generar medición simulada (placeholder): distancias frontales variando suavemente
            static float phase = 0.0f;
            phase += 0.05f; if (phase > TWO_PI) phase -= TWO_PI;
            control::Measurement m;
            m.distancias[0] = 2000.0f + 150.0f * sinf(phase); // mm
            m.distancias[1] = 2000.0f + 150.0f * sinf(phase + 0.3f);
            m.distancias[2] = 2200.0f; // fija
            m.anchor_ok[0]=m.anchor_ok[1]=m.anchor_ok[2]=true;
            m.count = ++counter_;
            m.ts_ms = now;
            follow_.pushMeasurement(m);
            if (cfg_.log_periodico && (counter_ % 40 == 0)) {
                LOG_DEBUG("UWB", "Sim dist=%.0f/%.0f/%.0f cnt=%lu", m.distancias[0], m.distancias[1], m.distancias[2], (unsigned long)m.count);
            }
        }
    }

private:
    UwbConfigFacade cfg_;
    control::FollowController& follow_;
    unsigned long last_emit_ = 0;
    uint32_t counter_ = 0;
};

} // namespace uwb
