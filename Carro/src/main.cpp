#include <Arduino.h>
#include "core/GestorSistema.h"
#include "monitoreo/LoggerReinicios.h"
#include "monitoreo/LogMacros.h"
#include "config/ConfigPerfil.h"
#include "motion/DriverMotores.h"
#include "comms/espnow/EspNowReceiver.h"
#include "control/manual/ControlManual.h"
#include "config/ConfigControl.h"
#include "control/autopilot/FollowController.h"
#include "uwb/UWBCorePort.h"

using namespace core;
using namespace monitoreo;

// Declaración adelantada para consola (se define en setup)
namespace control { class FollowController; }
extern control::FollowController& followCtrlRef;

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
    static config::ControlTuning tuning = config::obtenerControlTuningDefault();
    static comms::EspNowReceiver espNow(perfil.comms.canal_wifi);
    static control::ControlManual ctrlManual(perfil.control, driverMotores);
    static control::FollowController followCtrl(tuning, driverMotores);
    // Exponer referencia global controlada (solo lectura externa) para consola
    control::FollowController& followCtrlRef = followCtrl;
    static uwb::UWBCoreConfig uwbCfg; // valores por defecto portados
    static uwb::UWBCorePort uwbCore(uwbCfg, followCtrl);

    if (perfil.motores.pin_enable >= 0)
        core::GestorSistema::instancia().registrar(&driverMotores);
    if (perfil.comms.habilitar_espnow)
        core::GestorSistema::instancia().registrar(&espNow);
    if (perfil.control.habilitar_control_manual)
        core::GestorSistema::instancia().registrar(&ctrlManual);
    if (tuning.velocidad_maxima > 0)
        core::GestorSistema::instancia().registrar(&followCtrl);
    core::GestorSistema::instancia().registrar(&uwbCore);

    // Conectar callback del receptor al controlador manual
    espNow.setCallback([&](const comms::ManualCommand& cmd){ 
        ctrlManual.onManualCommand(cmd);
        // También actualizamos modo al follow controller (modo viene en cmd.modo)
        followCtrl.setModo(cmd.modo);
    });

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
    // Consola serial simple (no bloqueante)
    static String linea;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c=='\r') {
            if (linea.length()>0) {
                // parse comando
                if (linea.startsWith("setlog ")) {
                    String lvl = linea.substring(7); lvl.trim();
                    using config::LogLevel; LogLevel nuevo = config::LogLevel::INFO;
                    if (lvl == "TRACE") nuevo = LogLevel::TRACE; else if (lvl=="DEBUG") nuevo=LogLevel::DEBUG; else if (lvl=="INFO") nuevo=LogLevel::INFO; else if (lvl=="WARN") nuevo=LogLevel::WARN; else if (lvl=="ERROR") nuevo=LogLevel::ERROR; else if (lvl=="CRIT") nuevo=LogLevel::CRITICAL; else {
                        Serial.println(F("Nivel invalido")); goto fin_cmd; }
                    monitoreo::Logger::instancia().setNivelMinimo(nuevo);
                    Serial.println(F("OK setlog"));
                } else if (linea == "dump log") {
                    monitoreo::Logger::instancia().volcarEstado(Serial);
                } else if (linea == "dump pid") {
                    // Necesitamos acceso a followCtrl estatico de setup()
                    extern control::FollowController followCtrlRef; // declaracion externa
                    followCtrlRef.dumpPID(Serial);
                } else if (linea.startsWith("set pidang ")) {
                    float kp,ki,kd; if (sscanf(linea.c_str(), "set pidang %f %f %f", &kp,&ki,&kd)==3) {
                        extern control::FollowController followCtrlRef; followCtrlRef.setPIDAng(kp,ki,kd); Serial.println(F("OK pid ang")); }
                    else Serial.println(F("Uso: set pidang kp ki kd"));
                } else if (linea.startsWith("set piddist ")) {
                    float kp,ki,kd; if (sscanf(linea.c_str(), "set piddist %f %f %f", &kp,&ki,&kd)==3) {
                        extern control::FollowController followCtrlRef; followCtrlRef.setPIDDist(kp,ki,kd); Serial.println(F("OK pid dist")); }
                    else Serial.println(F("Uso: set piddist kp ki kd"));
                } else if (linea.startsWith("dump uwb")) {
                    uint8_t n=10; sscanf(linea.c_str(), "dump uwb %hhu", &n);
                    extern control::FollowController followCtrlRef; followCtrlRef.dumpMeasurements(Serial, n);
                } else if (linea == "modo 1") { extern control::FollowController followCtrlRef; followCtrlRef.setModo(1); Serial.println(F("OK modo 1")); }
                else if (linea == "modo 3") { extern control::FollowController followCtrlRef; followCtrlRef.setModo(3); Serial.println(F("OK modo 3")); }
                else if (linea == "help") {
                    Serial.println(F("Comandos: setlog <TRACE|DEBUG|INFO|WARN|ERROR|CRIT>, dump log, dump pid, set pidang kp ki kd, set piddist kp ki kd, dump uwb [N], modo 1, modo 3"));
                }
                else {
                    Serial.println(F("Comando desconocido (help)"));
                }
            fin_cmd:
                linea = ""; // reset
            }
        } else if (isPrintable(c)) {
            linea += c;
            if (linea.length()>120) linea.remove(0, linea.length()-120); // evitar overflow
        }
    }
    delay(5);
}