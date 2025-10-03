#include "ControlManual.h"

namespace control {
using monitoreo::Logger; using config::LogLevel; using motion::MotionCommand;

void ControlManual::onManualCommand(const comms::ManualCommand& cmd) {
    // Normalización simple similar a CarroClean (-1..1) y luego a -velocidad_max..velocidad_max
    constexpr int RAW_MIN = 600;
    constexpr int RAW_MAX = 3400;
    constexpr int RAW_CENTER = 2000;
    constexpr int RAW_DEAD = 200;

    auto mapAxis = [&](int raw){
        if (raw < RAW_CENTER - RAW_DEAD) {
            float norm = (float)(raw - (RAW_CENTER - RAW_DEAD)) / (float)(RAW_MIN - (RAW_CENTER - RAW_DEAD));
            norm = constrain(norm, 0.0f, 1.0f);
            return -norm; // negativo
        } else if (raw > RAW_CENTER + RAW_DEAD) {
            float norm = (float)(raw - (RAW_CENTER + RAW_DEAD)) / (float)(RAW_MAX - (RAW_CENTER + RAW_DEAD));
            norm = constrain(norm, 0.0f, 1.0f);
            return norm;
        }
        return 0.0f;
    };

    float xNorm = mapAxis(cmd.ax_raw);
    float yNorm = mapAxis(cmd.ay_raw);

    float maxV = (float)cfg_.velocidad_max;
    float vBase = yNorm * maxV;
    float vTurn = xNorm * maxV;
    int vL = (int)lroundf(vBase - vTurn);
    int vR = (int)lroundf(vBase + vTurn);

    vL = constrain(vL, -cfg_.velocidad_max, cfg_.velocidad_max);
    vR = constrain(vR, -cfg_.velocidad_max, cfg_.velocidad_max);

    MotionCommand mc{ (int16_t)vL, (int16_t)vR, MotionCommand::Source::MANUAL };
    driver_.aplicar(mc);

    static uint32_t counter = 0;
    if (++counter % 40 == 0) {
        Logger::instancia().logf(LogLevel::DEBUG, "MANUAL", "modo=%u ax=%u ay=%u L=%d R=%d", cmd.modo, cmd.ax_raw, cmd.ay_raw, vL, vR);
    }
}

} // namespace control
