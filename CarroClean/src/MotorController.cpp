#include "MotorController.h"
#include "RPiComm.h"

// ========== MOTOR PWM DEFINITIONS ==========
// Pines para los DRV8701 (ambos motores)
#define ENABLE_MOTORS 14  // Pin único de enable para ambos motores
#define PWML1 26
#define PWML2 25
#define CURRL 36  // SENSOR_VP
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

// setMotorL_direct eliminado (no usado)

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

void setupMotorController() {
    RPiComm_setup();
    setupMotorPWM();
    disableMotors();
    
    motorsEnabled = false;
    lastCommandTime = millis();
    
    Serial.println("[MotorController] Initialized");
}

void executeMotorCommand(const MotorCommand& cmd) {
    if (cmd.emergency_stop) {
        emergencyStop();
        return;
    }
    
    // CONVERTIR valores de GUI (-255 a +255) con zona muerta REAL en duty (0-255)
    int left_speed = constrain(cmd.motor_left_speed, -255, 255);
    int right_speed = constrain(cmd.motor_right_speed, -255, 255);
    
    // DEBUG - Solo mostrar si hay movimiento para reducir spam
    bool has_movement = (left_speed != 0 || right_speed != 0);
    if (has_movement) {
        Serial.printf("[MOTOR_CMD] GUI values: L=%d, R=%d\n", left_speed, right_speed);
    }
    
    // Definir zona muerta real (PWM duty mínimo para moverse) en UNIDADES DUTY 0..255
    const int DEADZONE_DUTY = 60; // mínimo real

    // 1) Mapear magnitud GUI -> DUTY (0..255) aplicando deadzone en duty
    int dutyL = 0;
    int dutyR = 0;
    if (abs(left_speed) > 0) {
        dutyL = map(abs(left_speed), 1, 255, DEADZONE_DUTY, 255);
        if (dutyL < DEADZONE_DUTY) dutyL = DEADZONE_DUTY;
    }
    if (abs(right_speed) > 0) {
        dutyR = map(abs(right_speed), 1, 255, DEADZONE_DUTY, 255);
        if (dutyR < DEADZONE_DUTY) dutyR = DEADZONE_DUTY;
    }

    // 2) Convertir DUTY -> porcentaje 0..100 para la API setMotorL/R existente
    int left_pwm = map(dutyL, 0, 255, 0, 100);
    int right_pwm = map(dutyR, 0, 255, 0, 100);

    bool left_dir = (left_speed >= 0);
    bool right_dir = (right_speed >= 0);
    
    // Solo mostrar conversión si hay movimiento
    if (has_movement) {
    Serial.printf("[MOTOR_CMD] Converted: L=%d%% (duty=%d) dir=%s, R=%d%% (duty=%d) dir=%s\n", 
             left_pwm, dutyL, left_dir ? "FWD" : "REV", 
             right_pwm, dutyR, right_dir ? "FWD" : "REV");
    }
    
    // Verificar si ambos motores están parados
    bool motors_stopped = (left_pwm == 0 && right_pwm == 0);
    
    setMotorL(left_pwm, left_dir);
    setMotorR(right_pwm, right_dir);
    
    // Notificar estado final al UWB
    if (motors_stopped) {
        motorsEnabled = false;
        
    } else {
        motorsEnabled = true;
        
    }
}

void processSerialCommands() {
    MotorCommand cmd;
    
    // Verificar si hay comandos de la Raspberry Pi
    if (RPiComm_receiveCommand(cmd)) {
        lastCommandTime = millis();
    // Ejecutar siempre el comando recibido.
    // La parada de emergencia se maneja dentro de executeMotorCommand() con cmd.emergency_stop
    executeMotorCommand(cmd);
    }
    
    // Timeout de seguridad: si no hay comunicación, parar motores
    if (motorsEnabled && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
        emergencyStop();
        Serial.println("[MotorController] TIMEOUT: Sin comunicación con RPi, parando motores");
    }
    
}



void emergencyStop() {
    disableMotors();
    motorsEnabled = false;
    
    
}

void disableMotors() {
    digitalWrite(ENABLE_MOTORS, LOW);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
    Serial.println("[PWM] Motores desactivados completamente");
}