#pragma once
#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Número de anchors
#define NUM_ANCHORS 3

struct UWBRawData {
    float distance1;
    float distance2;
    float distance3;
    bool valid1;
    bool valid2;
    bool valid3;
    unsigned long timestamp;
};

extern volatile UWBRawData g_uwb_raw_data;
extern volatile bool g_uwb_data_ready;
extern volatile unsigned long g_uwb_measurement_count;
extern volatile unsigned long g_uwb_last_measurement_time;

void UWBCore_setup();
void UWBCore_task(void* parameter);
bool UWBCore_getRawData(UWBRawData& data);
void UWBCore_startTask();
unsigned long UWBCore_getMeasurementCount();
// Snapshot no-consumidora (no altera g_uwb_data_ready)
bool UWBCore_snapshot(UWBRawData& out);

// Adaptation helpers to emit pipeline events (Option B integration)
bool UWBCore_update(); // original function kept as-is
void UWBCore_getDistances(float distances[NUM_ANCHORS]);
void UWBCore_getAnchorStatus(bool status[NUM_ANCHORS]);
void UWBCore_setVerbose(bool v);
bool UWBCore_getVerbose();
