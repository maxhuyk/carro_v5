#include <Arduino.h>
#include "UWBManager.h"
#include "UWBCore.h"

// Variables globales para almacenar las últimas distancias medidas
static float g_distances[NUM_ANCHORS] = {NAN, NAN, NAN};
static bool g_anchor_status[NUM_ANCHORS] = {false, false, false};

void UWBManager_setup() {
    // Inicializar el core UWB (Core 0)
    UWBCore_setup();
    
    // Iniciar la tarea UWB en Core 0
    UWBCore_startTask();
    
    Serial.println("[UWBManager] Dual-core UWB system initialized");
}

bool UWBManager_update(float &tag_x, float &tag_y) {
    // Obtener datos raw del Core 0 (UWB) - solo para almacenar
    UWBRawData rawData;
    UWBCore_getRawData(rawData);
    
    // DEBUG: Imprimir datos recibidos del Core 0
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 3000) {  // Debug cada 3 segundos
        Serial.printf("[UWBManager] Raw from Core0: d1=%.1f(v=%d), d2=%.1f(v=%d), d3=%.1f(v=%d)\n",
                     rawData.distance1, rawData.valid1,
                     rawData.distance2, rawData.valid2,
                     rawData.distance3, rawData.valid3);
        lastDebug = millis();
    }
    
    // Extraer distancias del Core 0
    float distance1 = rawData.distance1;
    float distance2 = rawData.distance2;
    float distance3 = rawData.distance3;
    
    // Almacenar distancias y estado de los anchors para envío a RPi
    g_distances[0] = distance1;
    g_distances[1] = distance2;
    g_distances[2] = distance3;
    g_anchor_status[0] = rawData.valid1;
    g_anchor_status[1] = rawData.valid2;
    g_anchor_status[2] = rawData.valid3;
    
    // Recovery eliminado: Si todos los anchors fallan, solo se reporta pero no se intenta recovery
    // Si necesitas recovery manual, reinicia el sistema desde el usuario
    
    // Ya no hacemos trilateración aquí - la RPi se encarga
    tag_x = NAN;
    tag_y = NAN;
    
    return false; // Siempre false ya que no calculamos posición
}

void UWBManager_getDistances(float distances[NUM_ANCHORS]) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        distances[i] = g_distances[i];
    }
}

void UWBManager_getAnchorStatus(bool status[NUM_ANCHORS]) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        status[i] = g_anchor_status[i];
    }
}

unsigned long UWBManager_getMeasurementCount() {
    return UWBCore_getMeasurementCount();
}

void UWBManager_runDiagnostics() {
    UWBCore_diagnostics();
}
// UWBManager_recoverAnchors eliminado (no usado)

