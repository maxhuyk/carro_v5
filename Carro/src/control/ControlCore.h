#pragma once
#include <Arduino.h>
#include "control/ControlMath.h"
#include "config/Config.h"

struct CarroData {
  float distancias[3];
  float power_data[3];
  float imu_data[6];
  float tag_sensors[3];
  float control_data[2];
  float buttons_data[4];
  bool data_valid;
};

void ControlCore_init();
void ControlCore_run(const CarroData& data); // ejecutar una iteración (similar control_main)
