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

// ===== NUEVAS FUNCIONES PARA CONTROL DIRECTO =====
// Funciones para ser usadas como callbacks desde control.cpp
void motor_enviar_pwm(int vel_izq, int vel_der);
void motor_detener();
void setupMotorControlDirect();