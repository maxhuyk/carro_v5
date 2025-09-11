#include "MotorController.h"
#include <ArduinoJson.h>

// ========== MOTOR PWM DEFINITIONS ==========
// Pines para los DRV8701 (ambos motores)
#define ENABLE_MOTORS 14  // Pin único de enable para ambos motores
#define PWML1 26
#define PWML2 25  // Restaurado (era 25 en rama clean) para habilitar canal reversa izquierdo
// Pines para el DRV8701 (motor derecho)
#define PWMR1 33
#define PWMR2 32
#define CURRR 39  // SENSOR_VN

// --- PWM ESP32 ---
#define PWM_FREQ 20000  // 20kHz - frecuencia original que funcionaba bien
#define PWM_RES 8       // 8 bits = 0-255
#define CH_L1 0
#define CH_L2 1
#define CH_R1 2
#define CH_R2 3

// Variables de estado
static bool motorsEnabled = false;
static bool pwm_initialized = false;
static unsigned long lastCommandTime = 0;
static const unsigned long COMMAND_TIMEOUT = 2000; // 2 segundos
static bool directControlMode = false; // Nueva variable para modo directo


// ========== MOTOR PWM BASE FUNCTIONS ==========
// Función para desactivar completamente los motores
void disableMotors() {
    digitalWrite(ENABLE_MOTORS, LOW);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
    Serial.println("[PWM] Motores desactivados completamente");
}


// pwm: 0 a 100, dir: true=adelante, false=atrás (COMO EL TEST ORIGINAL)
void setMotorL(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 100);
    int duty = map(pwm, 0, 100, 0, 255);
    
    // Activar enable si hay PWM
    if (pwm > 0) {
        digitalWrite(ENABLE_MOTORS, HIGH);
    }
    
    // Solo mostrar log si hay movimiento para reducir spam
    if (pwm > 0) {
        Serial.printf("[PWM] Izq: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)\n", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
    }
    if (dir) {
        ledcWrite(CH_L1, duty);
        ledcWrite(CH_L2, 0);
    } else {
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, duty);
    }
}



void setMotorR(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 100);
    int duty = map(pwm, 0, 100, 0, 255);
    
    // Activar enable si hay PWM
    if (pwm > 0) {
        digitalWrite(ENABLE_MOTORS, HIGH);
    }
    
    // Solo mostrar log si hay movimiento para reducir spam
    if (pwm > 0) {
        Serial.printf("[PWM] Der: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)\n", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
    }
    if (dir) {
        ledcWrite(CH_R1, duty);
        ledcWrite(CH_R2, 0);
    } else {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, duty);
    }
}



// Inicializa los pines de los drivers (sin test)
void setupMotorPWM() {
    Serial.println("[PWM] Inicializando pines y PWM...");
    pinMode(ENABLE_MOTORS, OUTPUT);
    pinMode(PWML1, OUTPUT);
    pinMode(PWML2, OUTPUT);
    pinMode(PWMR1, OUTPUT);
    pinMode(PWMR2, OUTPUT);
    digitalWrite(ENABLE_MOTORS, LOW);
    digitalWrite(PWML1, LOW);
    digitalWrite(PWML2, LOW);
    digitalWrite(PWMR1, LOW);
    digitalWrite(PWMR2, LOW);
    
    if (!pwm_initialized) {
        uint32_t freqL1 = ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
        uint32_t freqL2 = ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
        uint32_t freqR1 = ledcSetup(CH_R1, PWM_FREQ, PWM_RES);
        uint32_t freqR2 = ledcSetup(CH_R2, PWM_FREQ, PWM_RES);
        ledcAttachPin(PWML1, CH_L1);
        ledcAttachPin(PWML2, CH_L2);
        ledcAttachPin(PWMR1, CH_R1);
        ledcAttachPin(PWMR2, CH_R2);
        pwm_initialized = true;
        
        if (freqL1 == PWM_FREQ && freqL2 == PWM_FREQ && freqR1 == PWM_FREQ && freqR2 == PWM_FREQ) {
            Serial.println("[PWM] PWM inicializado correctamente");
        } else {
            Serial.println("[PWM] ADVERTENCIA: alguna frecuencia PWM no coincide");
        }
    }
}

// ========== MOTOR CONTROLLER FUNCTIONS ==========

void setupMotorController() {
    //RPiComm_setup();
    setupMotorPWM();
    disableMotors();
    
    motorsEnabled = false;
    lastCommandTime = millis();
    
    Serial.println("[MotorController] Initialized");
}


void emergencyStop() {
    disableMotors();
    motorsEnabled = false;
    
    // Notificar a la Raspberry Pi
    JsonDocument doc;
    doc["type"] = "emergency_stop_ack";
    doc["timestamp"] = millis();
    String output;
    serializeJson(doc, output);
    //RPiComm_sendJSON(output.c_str());
}

// Función para control directo desde sistema de control
void motor_enviar_pwm(int izq, int der) {
    // Ahora acepta -100..100 (signo = dirección). Mantener compatibilidad con llamadas previas (0..100)
    lastCommandTime = millis();

    izq = constrain(izq, -100, 100);
    der = constrain(der, -100, 100);

    int left_speed = map(abs(izq), 0, 100, 0, 255) * (izq < 0 ? -1 : 1);
    int right_speed = map(abs(der), 0, 100, 0, 255) * (der < 0 ? -1 : 1);
    
    Serial.printf("[MOTOR_VIA_EXECUTE] ControlSigned(%d,%d) -> UART(%d,%d)\n", 
                  izq, der, left_speed, right_speed);

    
}



void setupMotorControlDirect() {
    directControlMode = true;
    Serial.println("[MotorController] Modo directo activado");
}



