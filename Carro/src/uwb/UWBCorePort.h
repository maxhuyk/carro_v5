#pragma once
#include <Arduino.h>
#include <Adafruit_MCP23008.h>
#include "core/Modulo.h"
#include "monitoreo/LogMacros.h"
#include "control/autopilot/FollowController.h"
#include "DW3000.h"

namespace uwb {

struct UWBRawData {
    float d1, d2, d3; // cm en CarroClean, convertimos a mm luego
    bool v1, v2, v3;
    unsigned long timestamp;
};

struct UWBCoreConfig {
    // Pines (idénticos a CarroClean)
    uint8_t anchor1_cs = 0; // MCP GP0
    uint8_t anchor1_rst = 4; // GP4
    uint8_t anchor1_irq = 35; // GPIO35
    uint8_t anchor2_cs = 1; uint8_t anchor2_rst = 5; uint8_t anchor2_irq = 27;
    uint8_t anchor3_cs = 2; uint8_t anchor3_rst = 6; uint8_t anchor3_irq = 13;

    // Delays críticos (valores derivados del original)
    uint16_t delay_soft_reset_ms = 200;
    uint16_t delay_check_idle_ms = 200;
    uint16_t rx_timeout_ms = 30;
    uint16_t inter_anchor_delay_us = 1000; // 1ms
    uint16_t cycle_delay_ms = 1;
};

class UWBCorePort : public core::Modulo {
public:
    UWBCorePort(const UWBCoreConfig& cfg, control::FollowController& follow)
    : cfg_(cfg), follow_(follow), a1(cfg.anchor1_cs,cfg.anchor1_rst,cfg.anchor1_irq),
      a2(cfg.anchor2_cs,cfg.anchor2_rst,cfg.anchor2_irq), a3(cfg.anchor3_cs,cfg.anchor3_rst,cfg.anchor3_irq) {}

    bool iniciar() override;
    void actualizar() override; // ejecuta un ciclo parcial por loop (cooperativo)
    void detener() override {}

private:
    UWBCoreConfig cfg_;
    control::FollowController& follow_;
    DW3000Class a1, a2, a3;

    // Estado de iteración
    enum Stage { A1, A2, A3 } stage_ = A1;
    unsigned long last_cycle_ms_ = 0;
    uint32_t count_ = 0;

    // Recovery por anchor
    struct AnchorState { uint16_t consecNotIdle=0; unsigned long lastSuccessMs=0; uint8_t softResets=0; uint8_t hardResets=0; };
    AnchorState s1_, s2_, s3_;

    // Métodos
    void initAnchor(DW3000Class& D, const char* name, AnchorState& S);
    float dsr_once(DW3000Class& D, uint16_t& failCount);
    void try_recovery(DW3000Class& D, AnchorState& S, const char* name);
};

} // namespace uwb
