#pragma once

#include <Arduino.h>
#include "RPiComm.h"

// Funciones públicas del controlador
void setupMotorController();
void processSerialCommands();
void emergencyStop();

// Funciones base de PWM de motores
void disableMotors();
void setMotorL(int pwm, bool dir);
void setMotorR(int pwm, bool dir);

// (Lecturas de sensores se realizan en RPiComm)

// Protocolo de comunicación:
// ESP32 <- RPi: comandos de motor (lista o JSON)
// ESP32 -> RPi: lista compacta UWB + sensores + TAG
