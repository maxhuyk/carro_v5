#include "UWBCore.h"
#include "monitoreo/LogMacros.h"

namespace uwb {
using namespace monitoreo;

static uint16_t failCountA1 = 0, failCountA2 = 0, failCountA3 = 0;

bool UWBCore::iniciar() {
    sem_ = xSemaphoreCreateBinary();
    if (!sem_) return false;
    xSemaphoreGive(sem_);

    // Inicialización anchors con timings CarroClean
    auto initAnchor = [&](DW3000Class& D, const char* name, AnchorState& S) {
        LOG_INFO("UWB", "Init %s", name);
        D.begin();
        D.hardReset();
        while(!D.checkForIDLE()) { delay(200); }
        D.softReset(); delay(200);
        D.init();
        D.setupGPIO();
        D.configureAsTX();
        D.clearSystemStatus();
        S.lastSuccessMs = millis();
    };
    initAnchor(a1_,"A1",s1_);
    initAnchor(a2_,"A2",s2_);
    initAnchor(a3_,"A3",s3_);

    BaseType_t r = xTaskCreatePinnedToCore(taskEntry, "UWB_Core_Task", 4096, this, 2, &task_handle_, 0);
    if (r != pdPASS) {
        LOG_ERROR("UWB","No se pudo crear tarea UWB");
        return false;
    }
    LOG_INFO("UWB","Tarea UWB creada (Core0)");
    return true;
}

void UWBCore::actualizar() {
    // Consumir dato listo y empujar al FollowController
    if (xSemaphoreTake(sem_, 0) == pdTRUE) {
        bool ready = data_ready_;
        UWBRawData local = raw_;
        if (ready) {
            data_ready_ = false; // marcar consumido
            xSemaphoreGive(sem_);
            control::Measurement m; // convertir cm -> mm
            m.distancias[0] = local.d1 * 10.0f;
            m.distancias[1] = local.d2 * 10.0f;
            m.distancias[2] = local.d3 * 10.0f;
            m.anchor_ok[0] = local.v1; m.anchor_ok[1] = local.v2; m.anchor_ok[2] = local.v3;
            m.ts_ms = local.timestamp; m.count = measurement_count_;
            follow_.pushMeasurement(m);
        } else {
            xSemaphoreGive(sem_);
        }
    }
}

void UWBCore::taskEntry(void* self) { static_cast<UWBCore*>(self)->taskLoop(); }

void UWBCore::anchor_soft_reinit(DW3000Class& D) {
    D.softReset(); delay(200); D.init(); D.setupGPIO(); D.configureAsTX(); D.clearSystemStatus(); }

void UWBCore::try_anchor_recovery(DW3000Class& D, AnchorState& S, const char* name) {
    if (!D.checkForIDLE()) {
        S.consecNotIdle++;
        if (S.consecNotIdle >= 2) {
            LOG_WARN("UWB","%s no IDLE -> soft reinit", name);
            anchor_soft_reinit(D); S.softResets++; S.consecNotIdle = 0; delay(100);
            if (!D.checkForIDLE()) {
                LOG_WARN("UWB","%s sigue no IDLE -> hardReset", name);
                D.hardReset(); unsigned long t0=millis(); while(!D.checkForIDLE() && (millis()-t0)<1000) delay(50);
                anchor_soft_reinit(D); S.hardResets++;
            }
        }
    } else {
        S.consecNotIdle = 0;
    }
}

float UWBCore::dsr_once(DW3000Class& D, uint16_t& failCount, unsigned long rxTimeoutMs) {
    long long tx=0, rx=0; int t_roundA=0, t_replyA=0; int clock_offset=0;
    D.ds_sendFrame(1); tx = D.readTXTimestamp();
    {
        unsigned long t0=millis();
        while(true){
            int rx_status = D.receivedFrameSucc();
            if (rx_status){
                D.clearSystemStatus();
                if (rx_status==1){
                    if (D.ds_isErrorFrame() || D.ds_getStage()!=2){ if(failCount<0xFFFF) failCount++; return NAN; }
                    break;
                } else { if(failCount<0xFFFF) failCount++; return NAN; }
            }
            if (millis()-t0 > rxTimeoutMs){ D.clearSystemStatus(); if(failCount<0xFFFF) failCount++; return NAN; }
            delayMicroseconds(200);
        }
    }
    rx = D.readRXTimestamp(); D.ds_sendFrame(3); t_roundA=(int)(rx-tx); tx=D.readTXTimestamp(); t_replyA=(int)(tx-rx);
    {
        unsigned long t0=millis();
        while(true){
            int rx_status = D.receivedFrameSucc();
            if (rx_status){
                D.clearSystemStatus();
                if (rx_status==1){
                    if (D.ds_isErrorFrame()){ if(failCount<0xFFFF) failCount++; return NAN; }
                    clock_offset = D.getRawClockOffset();
                    break;
                } else { if(failCount<0xFFFF) failCount++; return NAN; }
            }
            if (millis()-t0 > rxTimeoutMs){ D.clearSystemStatus(); if(failCount<0xFFFF) failCount++; return NAN; }
            delayMicroseconds(200);
        }
    }
    int t_roundB = D.read(0x12,0x04); int t_replyB = D.read(0x12,0x08);
    int ranging_time = D.ds_processRTInfo(t_roundA,t_replyA,t_roundB,t_replyB,clock_offset);
    float dist_cm = D.convertToCM(ranging_time);
    if (dist_cm < 0 || dist_cm > 100000) return NAN;
    return dist_cm;
}

void UWBCore::taskLoop() {
    unsigned long now0 = millis();
    s1_.lastSuccessMs = s2_.lastSuccessMs = s3_.lastSuccessMs = now0;
    while(true){
        // Delays CarroClean (comentario original: 1ms cycle + inter-anchor 1000us)
        try_anchor_recovery(a1_, s1_, "A1");
        float d1 = dsr_once(a1_, failCountA1, cfg_.rx_timeout_ms);
        if(!isnan(d1)) s1_.lastSuccessMs = millis();
        delayMicroseconds(cfg_.inter_anchor_delay_us);

        try_anchor_recovery(a2_, s2_, "A2");
        float d2 = dsr_once(a2_, failCountA2, cfg_.rx_timeout_ms);
        if(!isnan(d2)) s2_.lastSuccessMs = millis();
        delayMicroseconds(cfg_.inter_anchor_delay_us);

        try_anchor_recovery(a3_, s3_, "A3");
        float d3 = dsr_once(a3_, failCountA3, cfg_.rx_timeout_ms);
        if(!isnan(d3)) s3_.lastSuccessMs = millis();

        // Publicar
        if (xSemaphoreTake(sem_, pdMS_TO_TICKS(2)) == pdTRUE) {
            raw_.d1 = d1; raw_.d2 = d2; raw_.d3 = d3;
            raw_.v1 = !isnan(d1); raw_.v2 = !isnan(d2); raw_.v3 = !isnan(d3);
            raw_.timestamp = millis();
            data_ready_ = true; measurement_count_++;
            xSemaphoreGive(sem_);
        }
        vTaskDelay(pdMS_TO_TICKS(cfg_.cycle_delay_ms));
    }
}

} // namespace uwb
