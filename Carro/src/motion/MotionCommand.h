#pragma once
#include <Arduino.h>

namespace motion {

// Comando lógico de movimiento (previo a traducción a PWM crudo)
struct MotionCommand {
    int16_t left;   // -100..100
    int16_t right;  // -100..100
    enum class Source : uint8_t { MANUAL, AUTOPILOT, SAFETY } source = Source::MANUAL;
};

} // namespace motion
