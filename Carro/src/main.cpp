#include <Arduino.h>
#include "core/GestorSistema.h"
#include "monitoreo/LoggerReinicios.h"
#include "monitoreo/LogMacros.h"
#include "config/ConfigPerfil.h"
#include "motion/DriverMotores.h"
#include "comms/espnow/EspNowReceiver.h"
#include "control/manual/ControlManual.h"

using namespace core;
using namespace monitoreo;

// En el futuro: instancias de módulos concretos (ESP-NOW, UWB, Control, Motores).
// Por ahora sólo demostramos el ciclo de vida básico.

void setup() {
    Serial.begin(500000);
    delay(300);
    // Mensaje inicial solo para asegurar que el Serial está activo antes del logger
    Serial.println(F("[BOOT] Inicializando (se cambiará a logger)..."));

    auto t0 = millis();

    // Cargar perfil completo de configuración
    static config::ConfigPerfil perfil = config::cargarConfigDefault();

    // Iniciar logger con la parte de logging del perfil
    monitoreo::Logger::instancia().iniciar(perfil.logging);

    // Instanciar módulos condicionalmente según el perfil
    static motion::DriverMotores driverMotores(perfil.motores);
    static comms::EspNowReceiver espNow(perfil.comms.canal_wifi);
    static control::ControlManual ctrlManual(perfil.control, driverMotores);

    if (perfil.motores.pin_enable >= 0)
        core::GestorSistema::instancia().registrar(&driverMotores);
    if (perfil.comms.habilitar_espnow)
        core::GestorSistema::instancia().registrar(&espNow);
    if (perfil.control.habilitar_control_manual)
        core::GestorSistema::instancia().registrar(&ctrlManual);

    // Conectar callback del receptor al controlador manual
    espNow.setCallback([&](const comms::ManualCommand& cmd){ ctrlManual.onManualCommand(cmd); });

    if (!GestorSistema::instancia().iniciar()) {
        LOG_ERROR("CORE", "Fallo inicializando modulos");
    } else {
        LOG_INFO("CORE", "Modulos iniciados");
    }
    auto t1 = millis();
    monitoreo::Logger::instancia().registrarBoot(t1 - t0);
    LOG_INFO("BOOT", "Duracion init=%lums", (unsigned long)(t1-t0));
}

void loop() {
    GestorSistema::instancia().loop();
    delay(5);
}