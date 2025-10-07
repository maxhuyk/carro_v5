#include "MotorController.h"
#include "core/Log.h"

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

// ====== Estructura de comando (reducida, sin RPi) ======
struct MotorCommand {
    char command_type = 'M';          // 'M' para motor
    int motor_left_speed = 0;         // -255..255 (signo = dirección)
    int motor_right_speed = 0;        // -255..255
    bool emergency_stop = false;      // bandera de emergencia
};

static void executeMotorCommand(const MotorCommand& cmd); // forward
static void motorSelfTest();

// ========== MOTOR PWM BASE FUNCTIONS ==========
// Función para desactivar completamente los motores
void disableMotors() {
    digitalWrite(ENABLE_MOTORS, LOW);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
    LOGI("PWM","Motores desactivados completamente");
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
        LOGD("PWM","Izq: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
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
        LOGD("PWM","Der: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
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
    LOGI("PWM","Inicializando pines y PWM...");
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
            LOGI("PWM","PWM inicializado correctamente");
        } else {
            LOGW("PWM","ADVERTENCIA: alguna frecuencia PWM no coincide");
        }
    }
}

// ========== MOTOR CONTROLLER FUNCTIONS ==========

void setupMotorController() {
    setupMotorPWM();
    disableMotors();
    
    motorsEnabled = false;
    lastCommandTime = millis();
    
    LOGI("MotorController","Initialized");
}

void emergencyStop() {
    disableMotors();
    motorsEnabled = false;
    
    LOGI("MotorController","Emergency stop executed");
}

// Función para control directo desde sistema de control
void motor_enviar_pwm(int izq, int der) {
    // Entrada lógica -100..100 -> convertimos a escala -255..255 y delegamos
    lastCommandTime = millis();
    izq = constrain(izq, -100, 100);
    der = constrain(der, -100, 100);
    MotorCommand cmd;
    cmd.command_type = 'M';
    cmd.motor_left_speed = (izq * 255) / 100;
    cmd.motor_right_speed = (der * 255) / 100;
    cmd.emergency_stop = false;
    executeMotorCommand(cmd);
}

// Parada explícita accesible desde otros módulos
void motor_detener() {
    motor_enviar_pwm(0,0);
}

void setupMotorControlDirect() {
    directControlMode = true;
    LOGI("MotorController","Modo directo activado");
}

// ===== Implementación de executeMotorCommand (sin RPi) =====
static void executeMotorCommand(const MotorCommand& cmd) {
    if (cmd.emergency_stop) {
        emergencyStop();
        return;
    }

    int left_speed = constrain(cmd.motor_left_speed, -255, 255);
    int right_speed = constrain(cmd.motor_right_speed, -255, 255);

    bool has_movement = (left_speed != 0 || right_speed != 0);
    if (has_movement) {
        LOGD("MOTOR_CMD","RAW L=%d R=%d", left_speed, right_speed);
    }

    // IMPORTANTE: En la versión original el DEADZONE se interpretaba en porcentaje (0..100)
    // y se mapeaba directamente a setMotorX que vuelve a mapear a 0..255. Eso implicaba
    // que TODO valor distinto de cero aplicaba al menos ~60% de 255 (~153 duty), generando
    // suficiente torque. Aquí restauramos esa semántica porque la versión previa había
    // reducido el mínimo real a ~23% (insuficiente para arrancar bajo carga).
    const int DEADZONE_PERCENT = 60; // mínimo en ESCALA 0..100 (no duty directo)

    auto toPercent = [&](int v){
        if (v == 0) return 0; // detenido
        int mag = abs(v); // 1..255
        int pwmPercent = map(mag, 1, 255, DEADZONE_PERCENT, 100); // mantiene 60..100%
        if (pwmPercent < DEADZONE_PERCENT) pwmPercent = DEADZONE_PERCENT;
        return pwmPercent; // 60..100
    };

    int left_pwm = toPercent(left_speed);   // 0 ó 60..100
    int right_pwm = toPercent(right_speed); // 0 ó 60..100
    bool left_dir = (left_speed >= 0);
    bool right_dir = (right_speed >= 0);

    if (has_movement) {
        LOGD("MOTOR_CMD","CONVERT L_PWM=%d(%s) R_PWM=%d(%s)",
                      left_pwm, left_dir?"FWD":"REV", right_pwm, right_dir?"FWD":"REV");
        // Log duty efectivo (lo que realmente se enviará tras map 0..100 -> 0..255)
        int dutyL = map(left_pwm, 0, 100, 0, 255);
        int dutyR = map(right_pwm, 0, 100, 0, 255);
        LOGD("MOTOR_CMD","EFFECTIVE_DUTY L=%d/255 R=%d/255", dutyL, dutyR);
    }

    setMotorL(left_pwm, left_dir);
    setMotorR(right_pwm, right_dir);
    motorsEnabled = (left_pwm != 0 || right_pwm != 0);
}

// ===== Auto-test simple de motores =====
static void motorSelfTest() {
    LOGI("MOTOR_TEST","Iniciando auto-test breve de motores (modo torque alto)");
    // Usamos 80% para superar cualquier fricción inicial; luego si funciona bajamos.
    const int TEST_PWM = 80;   // % (0..100)
    const int TEST_MS  = 500;  // duración por fase
    const int PAUSE_MS = 200;  // pausa entre fases

    digitalWrite(ENABLE_MOTORS, HIGH); // Asegurar enable alto durante el test
    LOGI("MOTOR_TEST","ENABLE=%d TEST_PWM=%d -> duty≈%d/255", digitalRead(ENABLE_MOTORS), TEST_PWM, map(TEST_PWM,0,100,0,255));

    // Fase 1: ambos adelante
    setMotorL(TEST_PWM, true);
    setMotorR(TEST_PWM, true);
    delay(TEST_MS);
    disableMotors(); delay(PAUSE_MS);

    // Fase 2: ambos atrás
    digitalWrite(ENABLE_MOTORS, HIGH);
    setMotorL(TEST_PWM, false);
    setMotorR(TEST_PWM, false);
    delay(TEST_MS);
    disableMotors(); delay(PAUSE_MS);

    // Fase 3: giro izquierda (L atrás, R adelante)
    digitalWrite(ENABLE_MOTORS, HIGH);
    setMotorL(TEST_PWM, false);
    setMotorR(TEST_PWM, true);
    delay(TEST_MS);
    disableMotors(); delay(PAUSE_MS);

    // Fase 4: giro derecha (L adelante, R atrás)
    digitalWrite(ENABLE_MOTORS, HIGH);
    setMotorL(TEST_PWM, true);
    setMotorR(TEST_PWM, false);
    delay(TEST_MS);
    disableMotors();

    motorsEnabled = false;
    lastCommandTime = millis();
    LOGI("MOTOR_TEST","Auto-test completado (si no hubo movimiento medir tensiones)");
}