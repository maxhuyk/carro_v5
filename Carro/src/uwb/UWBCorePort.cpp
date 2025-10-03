#include "UWBCorePort.h"

namespace uwb {
using namespace monitoreo;

static uint16_t failCountA1 = 0, failCountA2 = 0, failCountA3 = 0;

bool UWBCorePort::iniciar() {
    // Inicializar MCP23008 (lo hace la librería DW3000 de forma estática)
    if (!DW3000Class::initMCP23008()) {
        LOG_ERROR("UWB", "Fallo init MCP23008");
        return false;
    }
    LOG_INFO("UWB", "Inicializando anchors...");
    initAnchor(a1, "A1", s1_);
    initAnchor(a2, "A2", s2_);
    initAnchor(a3, "A3", s3_);
    LOG_INFO("UWB", "Anchors listos");
    return true;
}

void UWBCorePort::initAnchor(DW3000Class& D, const char* name, AnchorState& S) {
    LOG_INFO("UWB", "Init %s", name);
    D.begin();
    D.hardReset();
    while (!D.checkForIDLE()) {
        delay(cfg_.delay_check_idle_ms);
    }
    D.softReset(); delay(cfg_.delay_soft_reset_ms);
    D.init();
    D.setupGPIO();
    D.configureAsTX();
    D.clearSystemStatus();
    S.lastSuccessMs = millis();
}

void UWBCorePort::try_recovery(DW3000Class& D, AnchorState& S, const char* name) {
    if (!D.checkForIDLE()) {
        S.consecNotIdle++;
        if (S.consecNotIdle >= 2) {
            LOG_WARN("UWB", "%s no IDLE -> softReset", name);
            D.softReset(); delay(cfg_.delay_soft_reset_ms);
            D.init(); D.setupGPIO(); D.configureAsTX(); D.clearSystemStatus();
            S.softResets++; S.consecNotIdle = 0;
            if (!D.checkForIDLE()) {
                LOG_WARN("UWB", "%s sigue no IDLE -> hardReset", name);
                D.hardReset();
                unsigned long t0 = millis();
                while (!D.checkForIDLE() && (millis()-t0) < 1000) delay(50);
                D.softReset(); delay(cfg_.delay_soft_reset_ms);
                D.init(); D.setupGPIO(); D.configureAsTX(); D.clearSystemStatus();
                S.hardResets++;
            }
        }
    } else {
        S.consecNotIdle = 0;
    }
}

float UWBCorePort::dsr_once(DW3000Class& D, uint16_t& failCount) {
    long long tx=0, rx=0; int t_roundA=0, t_replyA=0; int clock_offset=0;
    D.ds_sendFrame(1); tx = D.readTXTimestamp();
    { // stage 1
        unsigned long t0 = millis();
        while (true) {
            int rx_status = D.receivedFrameSucc();
            if (rx_status) {
                D.clearSystemStatus();
                if (rx_status == 1) {
                    if (D.ds_isErrorFrame() || D.ds_getStage() != 2) { if (failCount<0xFFFF) failCount++; return NAN; }
                    break;
                } else { if (failCount<0xFFFF) failCount++; return NAN; }
            }
            if (millis()-t0 > cfg_.rx_timeout_ms) { D.clearSystemStatus(); if (failCount<0xFFFF) failCount++; return NAN; }
            delayMicroseconds(200);
        }
    }
    rx = D.readRXTimestamp(); D.ds_sendFrame(3); t_roundA = (int)(rx - tx); tx = D.readTXTimestamp(); t_replyA = (int)(tx - rx);
    { // stage 3
        unsigned long t0 = millis();
        while (true) {
            int rx_status = D.receivedFrameSucc();
            if (rx_status) {
                D.clearSystemStatus();
                if (rx_status == 1) {
                    if (D.ds_isErrorFrame()) { if (failCount<0xFFFF) failCount++; return NAN; }
                    clock_offset = D.getRawClockOffset();
                    break;
                } else { if (failCount<0xFFFF) failCount++; return NAN; }
            }
            if (millis()-t0 > cfg_.rx_timeout_ms) { D.clearSystemStatus(); if (failCount<0xFFFF) failCount++; return NAN; }
            delayMicroseconds(200);
        }
    }
    int t_roundB = D.read(0x12, 0x04); int t_replyB = D.read(0x12,0x08);
    int ranging_time = D.ds_processRTInfo(t_roundA, t_replyA, t_roundB, t_replyB, clock_offset);
    float dist_cm = D.convertToCM(ranging_time);
    if (dist_cm < 0 || dist_cm > 100000) return NAN;
    return dist_cm;
}

void UWBCorePort::actualizar() {
    unsigned long now = millis();
    // Secuencia cooperativa: un anchor por loop para no bloquear demasiado.
    switch(stage_) {
        case A1: {
            try_recovery(a1,s1_,"A1");
            float d1 = dsr_once(a1, failCountA1);
            if (!isnan(d1)) { s1_.lastSuccessMs = now; }
            last_cycle_ms_ = now;
            // Pasar a siguiente stage y acumular en buffer temporal
            // Guardamos resultado parcial en variables estáticas locales
            static float cache_d1, cache_d2, cache_d3; static bool v1,v2,v3; static bool have_d1=false, have_d2=false;
            cache_d1 = d1; v1 = !isnan(d1); have_d1 = true; stage_ = A2;
            break;
        }
        case A2: {
            try_recovery(a2,s2_,"A2");
            float d2 = dsr_once(a2, failCountA2);
            if (!isnan(d2)) { s2_.lastSuccessMs = now; }
            static float cache_d1, cache_d2, cache_d3; static bool v1,v2,v3; static bool have_d1, have_d2;
            // Recuperar d1 del static anterior (misma función - definidos arriba, repetimos para scope compartido)
            cache_d2 = d2; v2 = !isnan(d2); have_d2 = true; stage_ = A3;
            break;
        }
        case A3: {
            try_recovery(a3,s3_,"A3");
            float d3 = dsr_once(a3, failCountA3);
            if (!isnan(d3)) { s3_.lastSuccessMs = now; }
            static float cache_d1, cache_d2, cache_d3; static bool v1,v2,v3; static bool have_d1, have_d2;
            cache_d3 = d3; v3 = !isnan(d3);
            if (true) {
                // Ensamblar medición completa (en mm) y notificar
                control::Measurement m; m.distancias[0]=cache_d1*10.0f; m.distancias[1]=cache_d2*10.0f; m.distancias[2]=cache_d3*10.0f;
                m.anchor_ok[0]=v1; m.anchor_ok[1]=v2; m.anchor_ok[2]=v3; m.count=++count_; m.ts_ms=now;
                follow_.pushMeasurement(m);
                if (count_ % 30 == 0) {
                    LOG_DEBUG("UWB", "Cnt=%lu d(mm)=%.0f/%.0f/%.0f", (unsigned long)count_, m.distancias[0], m.distancias[1], m.distancias[2]);
                }
            }
            stage_ = A1; // reiniciar ciclo
            break;
        }
    }
}

} // namespace uwb
