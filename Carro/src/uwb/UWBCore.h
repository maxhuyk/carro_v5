#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "control/autopilot/FollowController.h"
#include "DW3000.h"

namespace uwb {

struct UWBRawData {
    float d1; float d2; float d3; // cm (como CarroClean)
    bool v1; bool v2; bool v3;
    unsigned long timestamp; // millis
};

struct UWBCoreRuntimeCounters {
    uint32_t measurement_count = 0;
};

// Config (se exponen solo delays principales para ajustar sin tocar fidelidad)
struct UWBCoreConfig {
    uint16_t rx_timeout_ms = 30;
    uint16_t inter_anchor_delay_us = 1000; // CarroClean: 1000us
    uint16_t cycle_delay_ms = 1;           // CarroClean: 1ms
};

class UWBCore : public core::Modulo {
public:
    UWBCore(const UWBCoreConfig& cfg, control::FollowController& follow)
        : cfg_(cfg), follow_(follow) {}

    bool iniciar() override;
    void actualizar() override; // toma lectura lista y la pasa al FollowController
    void detener() override {}

private:
    UWBCoreConfig cfg_;
    control::FollowController& follow_;

    // Raw data compartida (simulando el esquema multi-core de CarroClean)
    volatile UWBRawData raw_{NAN,NAN,NAN,false,false,false,0};
    volatile bool data_ready_ = false;
    volatile uint32_t measurement_count_ = 0;
    SemaphoreHandle_t sem_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;

    // Anchor DW3000 instances (pines idénticos)
    DW3000Class a1_{0,4,35};
    DW3000Class a2_{1,5,27};
    DW3000Class a3_{2,6,13};

    struct AnchorState { uint16_t consecNotIdle=0; unsigned long lastSuccessMs=0; uint8_t softResets=0; uint8_t hardResets=0; };
    AnchorState s1_, s2_, s3_;

    // Task static trampolines
    static void taskEntry(void* self);
    void taskLoop();

    // Internos
    float dsr_once(DW3000Class& D, uint16_t& failCount, unsigned long rxTimeoutMs);
    void anchor_soft_reinit(DW3000Class& D);
    void try_anchor_recovery(DW3000Class& D, AnchorState& S, const char* name);
};

} // namespace uwb
