#include "UWBCore.h"
#include "DW3000.h"
#include "core/UwbPipeline.h"
#include <Arduino.h>
#include <math.h>

// Configuración de pines del multiplexor MCP23008 (literal)
#define ANCHOR1_CS 0
#define ANCHOR1_RST 4
#define ANCHOR1_IRQ 35
#define ANCHOR2_CS 1
#define ANCHOR2_RST 5
#define ANCHOR2_IRQ 27
#define ANCHOR3_CS 2
#define ANCHOR3_RST 6
#define ANCHOR3_IRQ 13

// Instancias DW3000 (literal)
static DW3000Class anchor1(ANCHOR1_CS, ANCHOR1_RST, ANCHOR1_IRQ);
static DW3000Class anchor2(ANCHOR2_CS, ANCHOR2_RST, ANCHOR2_IRQ);
static DW3000Class anchor3(ANCHOR3_CS, ANCHOR3_RST, ANCHOR3_IRQ);

// Variables compartidas (literal)
volatile UWBRawData g_uwb_raw_data = {NAN, NAN, NAN, false, false, false, 0};
volatile bool g_uwb_data_ready = false;
volatile unsigned long g_uwb_measurement_count = 0;
volatile unsigned long g_uwb_last_measurement_time = 0;

static SemaphoreHandle_t uwbSemaphore = NULL; // literal threading primitive
static TaskHandle_t uwbTaskHandle = NULL;

static uint16_t failCountA1 = 0;
static uint16_t failCountA2 = 0;
static uint16_t failCountA3 = 0;

struct AnchorState {
    uint16_t consecTimeouts = 0;
    uint16_t consecErrors = 0;
    uint16_t consecNotIdle = 0;
    unsigned long lastSuccessMs = 0;
    uint8_t softResets = 0;
    uint8_t hardResets = 0;
};
static AnchorState a1State, a2State, a3State;

static void anchor_soft_reinit(DW3000Class& D) {
    D.softReset();
    delay(200);
    D.init();
    D.setupGPIO();
    D.configureAsTX();
    D.clearSystemStatus();
}
static void try_anchor_recovery(DW3000Class& D, AnchorState& S, const char* name) {
    if (!D.checkForIDLE()) {
        S.consecNotIdle++;
        if (S.consecNotIdle >= 2) {
            Serial.printf("[UWBCore][RECOVERY] %s no IDLE (%u). Soft reinit...\n", name, S.consecNotIdle);
            anchor_soft_reinit(D);
            S.softResets++; S.consecNotIdle = 0; delay(100);
            if (!D.checkForIDLE()) {
                Serial.printf("[UWBCore][RECOVERY] %s sigue no IDLE. Hard reset...\n", name);
                D.hardReset(); unsigned long t0 = millis();
                while (!D.checkForIDLE() && (millis()-t0)<1000) delay(50);
                D.softReset(); delay(200); D.init(); D.setupGPIO(); D.configureAsTX(); D.clearSystemStatus(); S.hardResets++;
            }
        }
    } else {
        S.consecNotIdle = 0;
    }
}

static float g_distances[NUM_ANCHORS] = {NAN,NAN,NAN};
static bool  g_anchor_status[NUM_ANCHORS] = {false,false,false};

static float dsr_once(DW3000Class& D, uint16_t& failCount, unsigned long rxTimeoutMs=30){
    long long tx=0, rx=0; int t_roundA=0, t_replyA=0; int clock_offset=0;
    D.ds_sendFrame(1); tx = D.readTXTimestamp();
    { unsigned long t0=millis(); while(true){ int r=D.receivedFrameSucc(); if(r){ D.clearSystemStatus(); if(r==1){ if(D.ds_isErrorFrame()){ if(failCount<0xFFFF) failCount++; return NAN; } else if(D.ds_getStage()!=2){ D.ds_sendErrorFrame(); if(failCount<0xFFFF) failCount++; return NAN; } break; } else { if(failCount<0xFFFF) failCount++; return NAN; } } if(millis()-t0>rxTimeoutMs){ D.clearSystemStatus(); if(failCount<0xFFFF) failCount++; return NAN; } delayMicroseconds(200);} }
    rx=D.readRXTimestamp(); D.ds_sendFrame(3); t_roundA=(int)(rx-tx); tx=D.readTXTimestamp(); t_replyA=(int)(tx-rx);
    { unsigned long t0=millis(); while(true){ int r=D.receivedFrameSucc(); if(r){ D.clearSystemStatus(); if(r==1){ if(D.ds_isErrorFrame()){ if(failCount<0xFFFF) failCount++; return NAN; } clock_offset=D.getRawClockOffset(); break; } else { if(failCount<0xFFFF) failCount++; D.clearSystemStatus(); return NAN; } } if(millis()-t0>rxTimeoutMs){ D.clearSystemStatus(); if(failCount<0xFFFF) failCount++; return NAN; } delayMicroseconds(200);} }
    int t_roundB = D.read(0x12,0x04); int t_replyB = D.read(0x12,0x08);
    int ranging_time = D.ds_processRTInfo(t_roundA,t_replyA,t_roundB,t_replyB,clock_offset);
    float distance = D.convertToCM(ranging_time); if(distance<0 || distance>100000) distance = NAN; return distance;
}

void UWBCore_setup(){
    uwbSemaphore = xSemaphoreCreateBinary(); xSemaphoreGive(uwbSemaphore);
    Serial.println("[UWBCore] Inicializando anchors con timings optimizados");
    Serial.println("[UWBCore] Inicializando Anchor 1..."); anchor1.begin(); anchor1.hardReset(); while(!anchor1.checkForIDLE()) delay(200); anchor1.softReset(); delay(200); anchor1.init(); anchor1.setupGPIO(); anchor1.configureAsTX(); anchor1.clearSystemStatus();
    Serial.println("[UWBCore] Inicializando Anchor 2..."); anchor2.begin(); anchor2.hardReset(); while(!anchor2.checkForIDLE()) delay(200); anchor2.softReset(); delay(200); anchor2.init(); anchor2.setupGPIO(); anchor2.configureAsTX(); anchor2.clearSystemStatus();
    Serial.println("[UWBCore] Inicializando Anchor 3..."); anchor3.begin(); anchor3.hardReset(); while(!anchor3.checkForIDLE()) delay(200); anchor3.softReset(); delay(200); anchor3.init(); anchor3.setupGPIO(); anchor3.configureAsTX(); anchor3.clearSystemStatus();
    Serial.println("[UWBCore] Todos los anchors inicializados");
}

void UWBCore_task(void* parameter){
    unsigned long now0 = millis(); a1State.lastSuccessMs=now0; a2State.lastSuccessMs=now0; a3State.lastSuccessMs=now0;
    while(true){
        const int inter_anchor_delay_us = 1000; // literal comentado original
        const int cycle_delay_ms = 1;
        try_anchor_recovery(anchor1,a1State,"A1"); float d1 = dsr_once(anchor1, failCountA1); if(!isnan(d1)){ a1State.lastSuccessMs=millis(); a1State.consecTimeouts=0; a1State.consecErrors=0; } else { if(!anchor1.checkForIDLE()) a1State.consecNotIdle++; a1State.consecTimeouts++; }
        delayMicroseconds(inter_anchor_delay_us);
        try_anchor_recovery(anchor2,a2State,"A2"); float d2 = dsr_once(anchor2, failCountA2); if(!isnan(d2)){ a2State.lastSuccessMs=millis(); a2State.consecTimeouts=0; a2State.consecErrors=0; } else { if(!anchor2.checkForIDLE()) a2State.consecNotIdle++; a2State.consecTimeouts++; }
        delayMicroseconds(inter_anchor_delay_us);
        try_anchor_recovery(anchor3,a3State,"A3"); float d3 = dsr_once(anchor3, failCountA3); if(!isnan(d3)){ a3State.lastSuccessMs=millis(); a3State.consecTimeouts=0; a3State.consecErrors=0; } else { if(!anchor3.checkForIDLE()) a3State.consecNotIdle++; a3State.consecTimeouts++; }
        unsigned long nowMs = millis(); bool v1=!isnan(d1), v2=!isnan(d2), v3=!isnan(d3);
        if(xSemaphoreTake(uwbSemaphore,pdMS_TO_TICKS(5))==pdTRUE){
            g_uwb_raw_data.distance1=d1; g_uwb_raw_data.distance2=d2; g_uwb_raw_data.distance3=d3;
            g_uwb_raw_data.valid1=v1; g_uwb_raw_data.valid2=v2; g_uwb_raw_data.valid3=v3; g_uwb_raw_data.timestamp=nowMs;
            g_uwb_data_ready=true; g_uwb_measurement_count++; g_uwb_last_measurement_time=nowMs; xSemaphoreGive(uwbSemaphore);
        }
        // Integración con pipeline: publicar medición combinada sólo si al menos un anchor válido (mantener literal, añadir publish condicional)
        if(v1||v2||v3){ static uint32_t seq=0; UwbMeasurement m; m.seq=++seq; m.t_ms=nowMs; m.t_us=micros(); m.valid=(v1||v2||v3); m.d[0]=d1; m.d[1]=d2; m.d[2]=d3; UwbPipeline::publish(m); }
        vTaskDelay(pdMS_TO_TICKS(cycle_delay_ms));
    }
}

void UWBCore_startTask(){
    Serial.println("[UWBCore] Iniciando tarea UWB en Core 0...");
    if(xTaskCreatePinnedToCore(UWBCore_task, "UWB_Core_Task", 4096, nullptr, 3, &uwbTaskHandle, 0)!=pdPASS){ Serial.println("[UWBCore] ERROR creando tarea UWB"); }
}

bool UWBCore_getRawData(UWBRawData& data){
    if(xSemaphoreTake(uwbSemaphore,pdMS_TO_TICKS(10))==pdTRUE){
        data = g_uwb_raw_data; bool had = g_uwb_data_ready; if(g_uwb_data_ready) g_uwb_data_ready=false; xSemaphoreGive(uwbSemaphore); return had; }
    return false;
}

bool UWBCore_update(){ UWBRawData raw; UWBCore_getRawData(raw); // mantiene interfaz
    static unsigned long lastDebug=0; if(millis()-lastDebug>3000){ Serial.printf("[UWB] Raw: d1=%.1f(v=%d) d2=%.1f(v=%d) d3=%.1f(v=%d)\n", raw.distance1, raw.valid1, raw.distance2, raw.valid2, raw.distance3, raw.valid3); lastDebug=millis(); }
    g_distances[0]=raw.distance1; g_distances[1]=raw.distance2; g_distances[2]=raw.distance3; g_anchor_status[0]=raw.valid1; g_anchor_status[1]=raw.valid2; g_anchor_status[2]=raw.valid3; return true; }

void UWBCore_getDistances(float distances[NUM_ANCHORS]){ for(int i=0;i<NUM_ANCHORS;i++) distances[i]=g_distances[i]; }
void UWBCore_getAnchorStatus(bool status[NUM_ANCHORS]){ for(int i=0;i<NUM_ANCHORS;i++) status[i]=g_anchor_status[i]; }
unsigned long UWBCore_getMeasurementCount(){ unsigned long c=0; if(xSemaphoreTake(uwbSemaphore,pdMS_TO_TICKS(10))==pdTRUE){ c=g_uwb_measurement_count; xSemaphoreGive(uwbSemaphore);} return c; }
