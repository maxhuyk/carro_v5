#pragma once
#include "core/Module.h"
#include "motor/MotorController.h"
#include "config/Config.h"
#include "core/Log.h"

// Modo emergencia:
//  - Lectura analógica en pin 36 (ADC1_CH0)
//  - Si V > 0.9V (umbral) => tomar control absoluto de motores
//  - Mapear lectura (raw) en rango [1000 .. 4095] -> [0 .. EMERGENCY_MAX_SPEED]
//  - Mientras activo, se ignoran otros comandos (control, manual, etc.)
//  - Al descender por debajo del umbral (raw<1000 aprox) se libera el control

class Module_Emergency : public Module {
public:
  Module_Emergency(): Module("Emergency", {"Motor"}) {}
  void configure() override {
    pin_ = 36;
    // Umbral de activación: 0.9V ~ raw = 4095 * 0.9/3.3 ≈ 1116 -> uso 1100 para histéresis leve
    activateThreshold_ = 900;
    releaseThreshold_ = 700; // para evitar flapping, se libera un poco más abajo
  }
  bool setup() override {
    pinMode(pin_, INPUT);
    LOGI("EMERG","Inicializado pin=%d thr_on=%d thr_off=%d vmax=%d", pin_, activateThreshold_, releaseThreshold_, EMERGENCY_MAX_SPEED);
    return true;
  }
  void loop() override {
    int raw = analogRead(pin_);
    unsigned long now = millis();
    // Determinar estado
    if(!active_){
      if(raw >= activateThreshold_){
        active_ = true; lastChangeMs_ = now;
        LOGW("EMERG","ACTIVADO raw=%d", raw);
      }
    } else {
      if(raw < releaseThreshold_){
        active_ = false; lastChangeMs_ = now;
        motor_enviar_pwm(0,0); // detener inmediatamente al liberar
        LOGW("EMERG","DESACTIVADO raw=%d", raw);
      }
    }
    if(active_){
      // Clampear rango mínimo para evitar ruido por debajo de releaseThreshold
      int clamped = raw; if(clamped < releaseThreshold_) clamped = releaseThreshold_;
      if(clamped > 4095) clamped = 4095;
      // Mapear [releaseThreshold_..4095] -> [0..EMERGENCY_MAX_SPEED]
      int spanIn = 4095 - releaseThreshold_;
      int spanRaw = clamped - releaseThreshold_;
      float ratio = spanIn>0 ? (float)spanRaw / (float)spanIn : 0.0f;
      if(ratio < 0) ratio = 0; if(ratio>1) ratio=1;
      int pwm = (int)lroundf(ratio * EMERGENCY_MAX_SPEED);
      // Enviar misma velocidad adelante (modo override total)
      motor_enviar_pwm(pwm, pwm);
      if(now - lastLogMs_ > 500){
        lastLogMs_ = now;
        LOGD("EMERG","raw=%d pwm=%d ratio=%.2f", raw, pwm, ratio);
      }
    }
  }
  bool isActive() const { return active_; }
private:
  int pin_=36;
  int activateThreshold_=1100;
  int releaseThreshold_=1000;
  bool active_=false;
  unsigned long lastChangeMs_=0;
  unsigned long lastLogMs_=0;
};
REGISTER_MODULE(Module_Emergency);
