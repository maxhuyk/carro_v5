#pragma once
#include "UWBCore.h"
#include <Arduino.h>

#define NUM_ANCHORS 3

void UWBManager_setup();
bool UWBManager_update(float &tag_x, float &tag_y);
void UWBManager_getDistances(float distances[NUM_ANCHORS]);
void UWBManager_getAnchorStatus(bool status[NUM_ANCHORS]);
unsigned long UWBManager_getMeasurementCount();
unsigned long UWBManager_getLastMeasurementTime(); // no implementado en original visible, se delegará
void UWBManager_runDiagnostics();
