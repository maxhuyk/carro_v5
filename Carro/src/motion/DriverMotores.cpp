#include "DriverMotores.h"

namespace motion {
using monitoreo::Logger;
using config::LogLevel;

// Canales PWM fijos
static const int CH_L1 = 0;
static const int CH_L2 = 1;
static const int CH_R1 = 2;
static const int CH_R2 = 3;

bool DriverMotores::iniciar() {
    configurarPWM();
    stop();
    last_cmd_ms_ = millis();
    Logger::instancia().logf(LogLevel::INFO, "MOTOR", "Driver iniciado (freq=%lu res=%u)", (unsigned long)cfg_.pwm_freq, cfg_.pwm_res_bits);
    return true;
}

void DriverMotores::configurarPWM() {
    ledcSetup(CH_L1, cfg_.pwm_freq, cfg_.pwm_res_bits);
    ledcSetup(CH_L2, cfg_.pwm_freq, cfg_.pwm_res_bits);
    ledcSetup(CH_R1, cfg_.pwm_freq, cfg_.pwm_res_bits);
    ledcSetup(CH_R2, cfg_.pwm_freq, cfg_.pwm_res_bits);

    ledcAttachPin(cfg_.pin_l1, CH_L1);
    ledcAttachPin(cfg_.pin_l2, CH_L2);
    ledcAttachPin(cfg_.pin_r1, CH_R1);
    ledcAttachPin(cfg_.pin_r2, CH_R2);

    pinMode(cfg_.pin_enable, OUTPUT);
    digitalWrite(cfg_.pin_enable, LOW);
    pwm_ok_ = true;
}

void DriverMotores::actualizar() {
    // Timeout de seguridad
    if ((millis() - last_cmd_ms_) > cfg_.timeout_ms) {
        if (ultimo_.left != 0 || ultimo_.right != 0) {
            Logger::instancia().logf(LogLevel::WARN, "MOTOR", "Timeout sin comando - deteniendo");
            stop();
        }
    }
}

void DriverMotores::detener() {
    stop();
    Logger::instancia().logf(LogLevel::INFO, "MOTOR", "Driver detenido");
}

void DriverMotores::stop() {
    ledcWrite(CH_L1, 0); ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0); ledcWrite(CH_R2, 0);
    digitalWrite(cfg_.pin_enable, LOW);
    ultimo_.left = ultimo_.right = 0;
}

static inline int aplicarDeadzoneMap(int valor, uint8_t dz_percent) {
    if (valor == 0) return 0;
    int sign = (valor > 0) ? 1 : -1;
    int mag = abs(valor); // 1..100
    int mapped = dz_percent + (mag * (100 - dz_percent)) / 100; // escala 60..100 p.ej.
    if (mapped > 100) mapped = 100;
    return sign * mapped;
}

void DriverMotores::aplicar(const MotionCommand& cmd) {
    if (!pwm_ok_) return;
    last_cmd_ms_ = millis();
    ultimo_ = cmd;

    // Umbral de comando
    MotionCommand c = cmd;
    if (abs(c.left) < cfg_.umbral_comando) c.left = 0;
    if (abs(c.right) < cfg_.umbral_comando) c.right = 0;

    // Aplicar deadzone (en porcentaje) sólo si distinto de cero
    int l = (c.left == 0) ? 0 : aplicarDeadzoneMap(c.left, cfg_.deadzone_percent);
    int r = (c.right == 0) ? 0 : aplicarDeadzoneMap(c.right, cfg_.deadzone_percent);

    bool dirL = l >= 0;
    bool dirR = r >= 0;
    int dutyL_percent = abs(l);
    int dutyR_percent = abs(r);

    int dutyL = map(dutyL_percent, 0, 100, 0, (1<<cfg_.pwm_res_bits)-1);
    int dutyR = map(dutyR_percent, 0, 100, 0, (1<<cfg_.pwm_res_bits)-1);

    if (dutyL > 0 || dutyR > 0) {
        digitalWrite(cfg_.pin_enable, HIGH);
    } else {
        digitalWrite(cfg_.pin_enable, LOW);
    }

    // Canal izquierdo
    if (dirL) { ledcWrite(CH_L1, dutyL); ledcWrite(CH_L2, 0); }
    else      { ledcWrite(CH_L1, 0);     ledcWrite(CH_L2, dutyL); }

    // Canal derecho
    if (dirR) { ledcWrite(CH_R1, dutyR); ledcWrite(CH_R2, 0); }
    else      { ledcWrite(CH_R1, 0);     ledcWrite(CH_R2, dutyR); }

    if (cfg_.log_detallado) {
        Logger::instancia().logf(LogLevel::DEBUG, "MOTOR", "Cmd L=%d R=%d -> dutyL=%d dutyR=%d", cmd.left, cmd.right, dutyL, dutyR);
    }
}

} // namespace motion
