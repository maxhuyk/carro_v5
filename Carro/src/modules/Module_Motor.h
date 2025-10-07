#pragma once

#include "core/Module.h"
#include "motor/MotorController.h"
#include "core/Log.h"

class Module_Motor : public Module {
public:
    Module_Motor() : Module("Motor") {}

    bool setup() override {
        LOGI("Module_Motor", "Setting up motor controller...");
        
        // Inicializar el controlador de motores
        setupMotorController();
        
        // Configurar modo de control directo
        setupMotorControlDirect();
        
        LOGI("Module_Motor", "Motor module initialized successfully");
        return true;
    }

    void loop() override {
        // El control de motores es principalmente reactivo
        // Los comandos vienen desde Module_Control via motor_enviar_pwm()
        // No necesita procesamiento continuo en el loop principal
    }
};
REGISTER_MODULE(Module_Motor);
