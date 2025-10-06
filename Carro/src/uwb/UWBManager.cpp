#include <Arduino.h>
#include "UWBManager.h"
#include "UWBCore.h"

static float g_distances[NUM_ANCHORS] = {NAN, NAN, NAN};
static bool  g_anchor_status[NUM_ANCHORS] = {false,false,false};

void UWBManager_setup(){
    UWBCore_setup();
    UWBCore_startTask();
    Serial.println("[UWBManager] Dual-core UWB system initialized");
}

bool UWBManager_update(float &tag_x, float &tag_y){
    UWBRawData rawData; UWBCore_getRawData(rawData);
    static unsigned long lastDebug=0; if(millis()-lastDebug>3000){
        Serial.printf("[UWBManager] Raw from Core0: d1=%.1f(v=%d), d2=%.1f(v=%d), d3=%.1f(v=%d)\n",
            rawData.distance1, rawData.valid1,
            rawData.distance2, rawData.valid2,
            rawData.distance3, rawData.valid3);
        lastDebug=millis();
    }
    g_distances[0]=rawData.distance1; g_distances[1]=rawData.distance2; g_distances[2]=rawData.distance3;
    g_anchor_status[0]=rawData.valid1; g_anchor_status[1]=rawData.valid2; g_anchor_status[2]=rawData.valid3;
    tag_x = NAN; tag_y = NAN; // sin trilateracion (RPi lo hará)
    return false;
}
void UWBManager_getDistances(float distances[NUM_ANCHORS]){ for(int i=0;i<NUM_ANCHORS;i++) distances[i]=g_distances[i]; }
void UWBManager_getAnchorStatus(bool status[NUM_ANCHORS]){ for(int i=0;i<NUM_ANCHORS;i++) status[i]=g_anchor_status[i]; }
unsigned long UWBManager_getMeasurementCount(){ return UWBCore_getMeasurementCount(); }
unsigned long UWBManager_getLastMeasurementTime(){ return g_uwb_last_measurement_time; }
void UWBManager_runDiagnostics(){ /* original llama a UWBCore_diagnostics (no incluida aún) */ }
