#pragma once
#include "UWBCore.h"

// Configuración del número de anchors (2 o 3)
#define NUM_ANCHORS 3  // Cambiar a 2 para usar solo 2 anchors

// Inicializa el sistema UWB dual-core (debe llamarse en setup)
void UWBManager_setup();

// Procesa los datos del Core 0 y actualiza variables internas (Core 1)
// Ya no calcula posición - solo almacena distancias para envío a RPi
bool UWBManager_update(float &tag_x, float &tag_y);

// Obtiene las distancias individuales de cada anchor
// distances[0] = distancia anchor 1, distances[1] = distancia anchor 2, etc.
void UWBManager_getDistances(float distances[NUM_ANCHORS]);

// Obtiene el estado de conexión de cada anchor
void UWBManager_getAnchorStatus(bool status[NUM_ANCHORS]);

// Obtiene el número total de mediciones realizadas
unsigned long UWBManager_getMeasurementCount();

// Obtiene el timestamp de la última medición
unsigned long UWBManager_getLastMeasurementTime();

// Nueva función de diagnóstico
void UWBManager_runDiagnostics();
// (APIs de actividad de motor y recuperación manual eliminadas por no usarse)
