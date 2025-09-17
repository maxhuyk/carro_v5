#include <Arduino.h>
#include "DW3000.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <ArduinoOTA.h>
#include <esp_sleep.h>

// (Escaneo I2C removido para versión simplificada del TAG)

// Configuración de botones
#define BTN_RIGHT  35       //ahora es joystick y
#define BTN_LEFT   39       //ahora es encendido
#define BTN_UP     14       //ahora es
#define BTN_DOWN   25       //ahora es joystick x 
#define BTN_OK     26       

// Configuración del divisor de voltaje de la batería LiPo
#define BATTERY_PIN 36
#define R1 10000.0
#define R2 33000.0
#define VOLTAGE_DIVIDER_FACTOR ((R1 + R2) / R2)
#define ADC_RESOLUTION 4095.0
#define ADC_REFERENCE_VOLTAGE 3.3
#define BATTERY_SAMPLES 10

// ESP-NOW Configuration
// MAC Address del Carro (reemplazar con la MAC real del Carro)
uint8_t carroMacAddress[] = {0x68, 0x25, 0xDD, 0x20, 0x14, 0x18}; // Placeholder - cambiar por MAC real
// mac del otro esp32  68:25:DD:20:14:18
//0x00, 0x4B, 0x12, 0x32, 0x8E, 0xFC
// Namespace para encapsular estado de comunicación ESP-NOW (alineado con CarroClean)
namespace EspNowComm {
    typedef struct {
        uint8_t  version = 1;       // Versión del protocolo
        uint16_t batteryVoltage_mV; // Voltaje en milivoltios (0-65535 mV)
        uint8_t  modo;              // Modo (0..4 usado)
        uint8_t  joystick;          // Joystick (0..7)
        uint32_t timestamp;         // Marca de tiempo
    } __attribute__((packed)) EspNowData;

    volatile EspNowData toSend = {};           // Buffer de envío
    volatile bool sendPending = false;         // Marcador opcional
    volatile unsigned long lastSendMs = 0;     // Último envío
    volatile unsigned long totalPacketsSent = 0; // Contador de envíos
}

// Sistema de modos
enum SystemMode {
    MODE_OFF = 0,       // Apagado
    MODE_TRACKING = 1,  // Seguimiento
    MODE_PAUSE = 2,     // Pausa
    MODE_MANUAL = 3,    // Manual
};

// Variables del sistema de modos
volatile SystemMode currentMode = MODE_OFF;
volatile uint8_t joystickValue = 0;

// Protección de cambios de modo
static unsigned long lastModeChangeTime = 0;
static const unsigned long MODE_CHANGE_GUARD_MS = 300; // Ignorar eventos por 300ms tras cambiar de modo

// Estructura para manejo de botones
struct ButtonState {
    // Lectura y estado estable (con debounce)
    bool raw = false;
    bool stable = false;
    bool previousStable = false;
    unsigned long lastChange = 0;

    // Edges
    bool rising = false;
    bool falling = false;

    // Long press (one-shot por pulsación)
    unsigned long pressStart = 0;
    bool longPressFired = false;
};

ButtonState buttons[5]; // RIGHT, LEFT, UP, DOWN, OK
#define BTN_IDX_RIGHT 0
#define BTN_IDX_LEFT  1
#define BTN_IDX_UP    2
#define BTN_IDX_DOWN  3
#define BTN_IDX_OK    4

const unsigned long LONG_PRESS_TIME = 1500; // 1.5s mejora reconocimiento
const unsigned long DEBOUNCE_TIME   = 40;   // 40ms debounce (modo)
// Long press especial para habilitar OTA (10s)
const unsigned long OTA_LONG_PRESS_MS = 10000;
// Apagado (mantener >3s)
const unsigned long POWER_LONG_PRESS_MS = 3000; // 3s para apagar

// Lecturas analógicas de los ejes reasignados
static int analogRightRaw = 0; // BTN_RIGHT (Y)
static int analogDownRaw  = 0; // BTN_DOWN  (X)
static float analogRightVolt = 0.0f;
static float analogDownVolt  = 0.0f;
static unsigned long lastAnalogSampleMs = 0;
static const unsigned long ANALOG_SAMPLE_INTERVAL_MS = 25; // ~40 Hz

// Normalización 0..1
static float analogRightNorm = 0.0f;
static float analogDownNorm  = 0.0f;

// Estado del botón de encendido (BTN_LEFT) para detección de long press sin interferir con latch
static bool powerPressing = false;
static unsigned long powerPressStart = 0;

// Variables compartidas entre cores (thread-safe)
volatile bool dw3000_data_ready = false;
volatile unsigned long total_cycles = 0;
volatile unsigned long successful_cycles = 0;
volatile unsigned long timeout_resets = 0;
volatile bool carro_detected = false;

// Semáforo para sincronización entre cores
SemaphoreHandle_t dw3000Semaphore = NULL;

// Handle de la tarea DW3000
TaskHandle_t dw3000TaskHandle = NULL;


uint8_t motorL = 127; // 0..255 (127 = neutro)
uint8_t motorR = 127; // 0..255 (127 = neutro)

// Estado OTA / ESP-NOW
static bool otaModeActive = false;
static bool espNowActive = true; // Mientras true, se envían paquetes ESP-NOW

// Credenciales WiFi para OTA (REEMPLAZAR por las reales)
const char* OTA_SSID = "CLAROWIFI";
const char* OTA_PASS = "11557788";

// Forward declaration
void enterOtaMode();



// ESP-NOW Callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.print("[ESP-NOW] Envío ");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ÉXITO" : "FALLO");
    if (status == ESP_NOW_SEND_SUCCESS) {
        EspNowComm::totalPacketsSent++;
        EspNowComm::lastSendMs = millis();
    }
}

// Función para inicializar ESP-NOW
bool initESPNOW() {
    if (!espNowActive) return false; // No inicializar si se ha deshabilitado por OTA
    // Configurar WiFi en modo Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // Asegurar que no esté conectado a ninguna red

    // Mostrar MAC Address del TAG
    Serial.print("[ESP-NOW] TAG MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error inicializando ESP-NOW");
        return false;
    }

    // Registrar callback de envío
    esp_now_register_send_cb(onDataSent);

    // Agregar peer (Carro)
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, carroMacAddress, 6);
    peerInfo.channel = 0;  // Canal automático
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    // Intentar agregar peer con reintentos
    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus != ESP_OK) {
        Serial.printf("[ESP-NOW] Error agregando peer: %d\n", addStatus);
        return false;
    }

    Serial.println("[ESP-NOW] Inicializado correctamente");
    Serial.printf("[ESP-NOW] Peer agregado: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  carroMacAddress[0], carroMacAddress[1], carroMacAddress[2],
                  carroMacAddress[3], carroMacAddress[4], carroMacAddress[5]);
    return true;
}

// Función para leer estado de botones (no bloqueante)
void readButtons() {
    // BTN_RIGHT (0) y BTN_DOWN (3) ahora son analógicos -> excluidos del escaneo digital
    // BTN_LEFT (1) funciona como latch de power: se maneja aparte (handlePowerButton)
    int pins[5] = {BTN_RIGHT, BTN_LEFT, BTN_UP, BTN_DOWN, BTN_OK};
    unsigned long now = millis();

    for (int i = 0; i < 5; i++) {
        if (i == BTN_IDX_RIGHT || i == BTN_IDX_DOWN || i == BTN_IDX_LEFT) {
            buttons[i].rising = false;
            buttons[i].falling = false;
            continue; // saltar
        }

        bool r = !digitalRead(pins[i]); // Activo en LOW (UP, OK)

        if (r != buttons[i].raw) {
            buttons[i].raw = r;
            buttons[i].lastChange = now;
        }

        if ((now - buttons[i].lastChange) >= DEBOUNCE_TIME && buttons[i].stable != buttons[i].raw) {
            buttons[i].previousStable = buttons[i].stable;
            buttons[i].stable = buttons[i].raw;
            buttons[i].rising = (buttons[i].stable && !buttons[i].previousStable);
            buttons[i].falling = (!buttons[i].stable && buttons[i].previousStable);

            if (buttons[i].rising) {
                buttons[i].pressStart = now;
                buttons[i].longPressFired = false;
            }
        } else {
            buttons[i].rising = false;
            buttons[i].falling = false;
        }

        if (buttons[i].stable && !buttons[i].longPressFired && (now - buttons[i].pressStart >= LONG_PRESS_TIME)) {
            buttons[i].longPressFired = true;
        }
    }
}

// Lectura analógica de los nuevos ejes (RIGHT=Y, DOWN=X)
void readAnalogAxes() {
    unsigned long now = millis();
    if (now - lastAnalogSampleMs < ANALOG_SAMPLE_INTERVAL_MS) return;
    lastAnalogSampleMs = now;

    analogRightRaw = analogRead(BTN_RIGHT); // ADC1 -> fiable con WiFi
    analogDownRaw  = analogRead(BTN_DOWN);  // ADC2 -> puede dar 0 con WiFi activo

    analogRightVolt = (analogRightRaw / 4095.0f) * 3.3f;
    analogDownVolt  = (analogDownRaw  / 4095.0f) * 3.3f;

    analogRightNorm = constrain(analogRightVolt / 3.3f, 0.0f, 1.0f);
    analogDownNorm  = constrain(analogDownVolt  / 3.3f, 0.0f, 1.0f);
}

// Manejo del botón de alimentación (long press >3s apaga)
void handlePowerButton() {
    static unsigned long lastSample = 0;
    unsigned long now = millis();
    if (now - lastSample < 30) return; // ~33 Hz
    lastSample = now;

    if (!otaModeActive) {
        // Leer estado momentáneamente poniendo INPUT
        pinMode(BTN_LEFT, INPUT);
        delayMicroseconds(40);
        bool pressed = (digitalRead(BTN_LEFT) == LOW); // activo LOW
        pinMode(BTN_LEFT, OUTPUT);
        digitalWrite(BTN_LEFT, HIGH); // reasegurar latch

        if (pressed && !powerPressing) {
            powerPressing = true;
            powerPressStart = now;
        } else if (!pressed) {
            powerPressing = false;
        }

        if (powerPressing && (now - powerPressStart >= POWER_LONG_PRESS_MS)) {
            Serial.println("[POWER] Long press >3s detectado. Apagando...");
            for (int i=0;i<3;i++){ digitalWrite(5, HIGH); delay(120); digitalWrite(5, LOW); delay(120);}            
            // Cortar alimentación
            digitalWrite(BTN_LEFT, LOW);
            delay(500);
            // Fallback: deep sleep si hardware no corta
            esp_deep_sleep_start();
        }
    } else {
        // En OTA, mantener HIGH (no permitir apagado accidental)
        digitalWrite(BTN_LEFT, HIGH);
    }
}

// Función para procesar cambios de modo
void processModeChanges() {
    unsigned long now = millis();
    if (now - lastModeChangeTime < MODE_CHANGE_GUARD_MS) return;

    auto resetButtonsAfterModeChange = [&](){
        lastModeChangeTime = now;
        for (int i = 0; i < 5; i++) {
            buttons[i].previousStable = buttons[i].stable;
            buttons[i].rising = false;
            buttons[i].falling = false;
            buttons[i].longPressFired = false;
            buttons[i].pressStart = now;
            buttons[i].lastChange = now;
        }
    };

    // OK long press: OFF -> TRACKING, others -> OFF
    if (buttons[BTN_IDX_OK].longPressFired) {
        buttons[BTN_IDX_OK].longPressFired = false;
        if (currentMode == MODE_OFF) {
            currentMode = MODE_TRACKING;
            Serial.println("[MODE] Cambiado a SEGUIMIENTO");
        } else if (currentMode == MODE_MANUAL) {
            currentMode = MODE_OFF;
            Serial.println("[MODE] Cambiado a APAGADO");
        } else {
            currentMode = MODE_OFF;
            Serial.println("[MODE] Cambiado a APAGADO");
        }
        resetButtonsAfterModeChange();
    }

    // UP long press -> MANUAL
    if (buttons[BTN_IDX_UP].longPressFired) {
        buttons[BTN_IDX_UP].longPressFired = false;
        currentMode = MODE_MANUAL;
        Serial.println("[MODE] Cambiado a MANUAL");
        resetButtonsAfterModeChange();
    }

    

    // DOWN rising: TRACKING -> PAUSE
    if (buttons[BTN_IDX_DOWN].rising && currentMode == MODE_TRACKING) {
        currentMode = MODE_PAUSE;
        Serial.println("[MODE] Cambiado a PAUSA");
        resetButtonsAfterModeChange();
        return;
    }

    // Any button rising in PAUSE -> TRACKING
    if (currentMode == MODE_PAUSE) {
        bool any = false;
        for (int i = 0; i < 5; i++) {
            if (buttons[i].rising) { any = true; break; }
        }
        if (any) {
            currentMode = MODE_TRACKING;
            Serial.println("[MODE] Cambiado a SEGUIMIENTO (desde pausa)");
            resetButtonsAfterModeChange();
        }
    }
}

// Función para calcular valor de joystick (solo modo MANUAL)
void calculateJoystickValue() {
    // Reset por defecto
    joystickValue = 0;
    motorL = motorR = 127;

    // Solo calcular joystick en modo MANUAL
    if (currentMode != MODE_MANUAL) { return; }

    // Para joystick, tomar lecturas crudas para respuesta más inmediata
    bool up = buttons[BTN_IDX_UP].raw;
    bool down = buttons[BTN_IDX_DOWN].raw;
    bool left = buttons[BTN_IDX_LEFT].raw;
    bool right = buttons[BTN_IDX_RIGHT].raw;

    // Invalidar combinaciones opuestas verticales/horizontales
    if ((up && down) || (left && right)) {
        joystickValue = 0;
        return;
    }

    // Mapeo CORREGIDO según lógica Python (solo 0-7, SIN caso 8)
    if (up && !left && !right) {
        joystickValue = 1; // (0100) adelante
    } else if (up && left) {
        joystickValue = 2; // (0110) adelante-izquierda
    } else if (!up && !down && left) {
        joystickValue = 3; // (0010) izquierda
    } else if (down && left) {
        joystickValue = 4; // (0011) atrás-izquierda
    } else if (down && !left && !right) {
        joystickValue = 5; // (0001) atrás
    } else if (down && right) {
        joystickValue = 6; // (1001) atrás-derecha
    } else if (!up && !down && right) {
        joystickValue = 7; // (1000) derecha
    } else if (up && right) {
        // PROBLEMA: Python NO tiene caso para adelante-derecha
        // Mapear a adelante simple (caso 1) como fallback
        joystickValue = 1; // adelante (fallback)
    } else {
        joystickValue = 0; // (0000) sin movimiento
    }
}

DW3000Class dw(4, 32, 34); // CS=4, RST=32, IRQ=34

// Tarea para Core 0 - Manejo del DW3000
void dw3000Task(void* parameter) {
    // Variables locales para la tarea DW3000 (Core 0)
    int frame_buffer = 0;
    int rx_status;
    int tx_status;
    int curr_stage = 0;
    int t_roundB = 0;
    int t_replyB = 0;
    long long rx = 0;
    long long tx = 0;
    unsigned long last_anchor_communication = 0;
    unsigned long communication_timeout = 1200; // Aumentado para reducir falsos timeouts
    bool local_carro_detected = false;
    unsigned long local_total_cycles = 0;
    unsigned long local_successful_cycles = 0;
    unsigned long local_timeout_resets = 0;
    unsigned long last_reset_time = 0;        // Último softReset real
    int consecutive_timeouts = 0;             // Timeouts seguidos desde última detección válida
    int consecutive_soft_resets = 0;          // Conteo de softResets para backoff

    Serial.println("[DW3000-Core0] Iniciando tarea DW3000...");

    // Inicialización DW3000 en Core 0
    dw.begin();
    dw.hardReset();
    delay(300);

    if(!dw.checkSPI()) {
        Serial.println("[ERROR] Could not establish SPI Connection to DW3000!");
        vTaskDelete(NULL);
        return;
    }

    while (!dw.checkForIDLE()) {
        Serial.println("[ERROR] IDLE1 FAILED");
        delay(500);
    }

    dw.softReset();
    delay(300);

    if (!dw.checkForIDLE()) {
        Serial.println("[ERROR] IDLE2 FAILED");
        vTaskDelete(NULL);
        return;
    }

    dw.init();
    delay(50);
    dw.setupGPIO();
    delay(30);
    dw.configureAsTX();
    delay(30);
    dw.clearSystemStatus();
    delay(20);
    dw.standardRX();

    last_anchor_communication = millis();
    local_carro_detected = false;

    Serial.println("[DW3000-Core0] DW3000 inicializado - Esperando detección del CARRO..."); 

    while (true) {
        // AUTO-DETECCIÓN DEL CARRO
        if (millis() - last_anchor_communication > communication_timeout) {
            // Timeout de comunicación
            consecutive_timeouts++;
            // Solo considerar "desconectado" si antes estaba detectado
            if (local_carro_detected) {
                if (consecutive_timeouts <= 2) {
                    // Reintento ligero: no soft reset, sólo limpiar y volver a RX
                    if ((consecutive_timeouts % 1) == 1) {
                        Serial.println("[DW3000-Core0] Timeout leve -> reintento sin softReset");
                    }
                    dw.clearSystemStatus();
                    dw.standardRX();
                    curr_stage = 0;
                } else {
                    // Soft reset escalonado con backoff mínimo 3s
                    unsigned long nowMs = millis();
                    if (nowMs - last_reset_time > 3000) {
                        Serial.printf("[DW3000-Core0] Timeout %d consecutivo -> softReset\n", consecutive_timeouts);
                        local_carro_detected = false;
                        local_timeout_resets++;
                        dw.softReset();
                        delay(120);
                        dw.init();
                        delay(25);
                        dw.setupGPIO();
                        delay(8);
                        dw.configureAsTX();
                        delay(8);
                        dw.clearSystemStatus();
                        delay(5);
                        dw.standardRX();
                        curr_stage = 0;
                        last_reset_time = nowMs;
                        consecutive_soft_resets++;
                        consecutive_timeouts = 0; // reiniciar contador tras softReset
                        // Ajustar timeout dinámicamente si muchos resets
                        if (consecutive_soft_resets >= 3 && communication_timeout < 2000) {
                            communication_timeout += 300; // ampliar ventana
                            Serial.printf("[DW3000-Core0] Aumentando communication_timeout a %lu ms\n", communication_timeout);
                        }
                        if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                            carro_detected = false;
                            timeout_resets = local_timeout_resets;
                            xSemaphoreGive(dw3000Semaphore);
                        }
                    } else {
                        // Demasiado pronto para otro softReset: sólo reintento ligero
                        dw.clearSystemStatus();
                        dw.standardRX();
                        curr_stage = 0;
                    }
                }
            } else {
                // No detectado aún: sólo seguir escuchando sin resets agresivos
                dw.standardRX();
                curr_stage = 0;
            }
            last_anchor_communication = millis();
        }

        // Máquina de estados DW3000
        switch (curr_stage) {
            case 0: { // Await ranging
                if (rx_status = dw.receivedFrameSucc()) {
                    dw.clearSystemStatus();
                    if (rx_status == 1) {
                        if (dw.ds_isErrorFrame()) {
                            curr_stage = 0;
                            dw.standardRX();
                        } else if (dw.ds_getStage() != 1) {
                            dw.ds_sendErrorFrame();
                            dw.standardRX();
                            curr_stage = 0;
                        } else {
                            // ÉXITO: Comunicación detectada con CARRO
                            if (!local_carro_detected) {
                                Serial.println("[DW3000-Core0] CARRO detectado!");
                                local_carro_detected = true;
                                consecutive_timeouts = 0; // reset de contadores
                                consecutive_soft_resets = 0;
                            }
                            last_anchor_communication = millis();
                            curr_stage = 1;
                            local_total_cycles++;
                        }
                    } else {
                        dw.clearSystemStatus();
                    }
                }
                break;
            }

            case 1:  // Ranging received. Sending response.
                dw.ds_sendFrame(2);
                delay(5);
                rx = dw.readRXTimestamp();
                delay(2);
                tx = dw.readTXTimestamp();
                t_replyB = tx - rx;
                curr_stage = 2;
                break;

            case 2:  // Awaiting response.
                if (rx_status = dw.receivedFrameSucc()) {
                    dw.clearSystemStatus();
                    if (rx_status == 1) {
                        if (dw.ds_isErrorFrame()) {
                            curr_stage = 0;
                            dw.standardRX();
                        } else if (dw.ds_getStage() != 3) {
                            dw.ds_sendErrorFrame();
                            dw.standardRX();
                            curr_stage = 0;
                        } else {
                            last_anchor_communication = millis();
                            curr_stage = 3;
                        }
                    } else {
                        dw.clearSystemStatus();
                        curr_stage = 0;
                    }
                }
                break;

            case 3:  // Second response received. Sending information frame.
                rx = dw.readRXTimestamp();
                delay(2);
                t_roundB = rx - tx;
                dw.ds_sendRTInfo(t_roundB, t_replyB);
                delay(5);

                // ÉXITO: Ciclo completado
                local_successful_cycles++;
                last_anchor_communication = millis();
                curr_stage = 0;

                // Actualizar variables compartidas thread-safe
                if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                    carro_detected = local_carro_detected;
                    total_cycles = local_total_cycles;
                    successful_cycles = local_successful_cycles;
                    timeout_resets = local_timeout_resets;
                    dw3000_data_ready = true;
                    xSemaphoreGive(dw3000Semaphore);
                }
                break;

            default:
                curr_stage = 0;
                dw.standardRX();
                break;
        }

        // Pequeño delay para no saturar el core
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Función para leer el voltaje de la batería LiPo
float readBatteryVoltage() {
    long sum = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        sum += analogRead(BATTERY_PIN);
        delayMicroseconds(100);
    }
    float average = (float)sum / BATTERY_SAMPLES;
    float voltage_at_pin = (average / ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
    float battery_voltage = voltage_at_pin * VOLTAGE_DIVIDER_FACTOR;
    return battery_voltage;
}

String getBatteryStatus(float voltage) {
    if (voltage > 4.1) return "FULL";
    else if (voltage > 3.8) return "GOOD";
    else if (voltage > 3.6) return "MID";
    else if (voltage > 3.3) return "LOW";
    else return "CRITICAL";
}

int getBatteryPercentage(float voltage) {
    if (voltage >= 4.1) return 100;
    else if (voltage >= 3.9) return 80;
    else if (voltage >= 3.8) return 60;
    else if (voltage >= 3.7) return 40;
    else if (voltage >= 3.6) return 20;
    else if (voltage >= 3.3) return 10;
    else return 0;
}

void setup() {
    // Primera instrucción: asegurar alimentación habilitada por el botón de encendido (BTN_LEFT)
    pinMode(BTN_LEFT, OUTPUT);
    digitalWrite(BTN_LEFT, HIGH); // Mantener línea de enable en alto

    pinMode(5, OUTPUT); // LED en GPIO 5
    digitalWrite(5, LOW);
    Serial.begin(500000);
    Serial.println(F("LinearAccelerometer enabled, Output in form x, y, z, accuracy, in m/s^2"));
    Serial.println(F("Gyro enabled, Output in form x, y, z, accuracy, in radians per second"));
    Serial.println(F("Rotation vector, Output in form i, j, k, real, accuracy"));

    // Blink de arranque
    digitalWrite(5, HIGH); delay(100); digitalWrite(5, LOW); delay(100);
    digitalWrite(5, HIGH); delay(100); digitalWrite(5, LOW);

    // Configurar el pin de lectura de batería
    pinMode(BATTERY_PIN, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Configurar entradas
    // Analógicos (sin pull-up): RIGHT=35 (ADC1), DOWN=25 (ADC2 - puede verse afectado por WiFi)
    pinMode(BTN_RIGHT, INPUT);
    pinMode(BTN_DOWN, INPUT);
    // BTN_LEFT ya está como OUTPUT (latch)
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_OK, INPUT_PULLUP);

    float initial_voltage = readBatteryVoltage();
    int battery_percentage = getBatteryPercentage(initial_voltage);
    Serial.printf("[BATTERY] Voltaje inicial: %.2fV (%s - %d%%)\n",
                  initial_voltage, getBatteryStatus(initial_voltage).c_str(), battery_percentage);

    Serial.println("[TAG] Inicializando sistema dual-core...");

    // Inicializar ESP-NOW
    if (!initESPNOW()) {
        Serial.println("[TAG] ERROR: No se pudo inicializar ESP-NOW");
    }

    // Inicializar datos ESP-NOW
    // Inicializar estructura de envío (evitar asignación a volatile)
    EspNowComm::toSend.version = 1;
    EspNowComm::toSend.batteryVoltage_mV = 0;
    EspNowComm::toSend.modo = 0;      // Modo inicial (apagado)
    EspNowComm::toSend.joystick = 0;  // Joystick inicial
    EspNowComm::toSend.timestamp = 0;

    Serial.println("[TAG] Sistema iniciado en MODO APAGADO");
    Serial.println("[TAG] Presiona OK 2 seg -> SEGUIMIENTO | UP 2 seg -> MANUAL");

    // Crear semáforo para comunicación entre cores
    dw3000Semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(dw3000Semaphore);

    // Crear tarea DW3000 en Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        dw3000Task,          // Función de la tarea
        "DW3000_Task",       // Nombre de la tarea
        4096,                // Tamaño del stack
        NULL,                // Parámetros
        2,                   // Prioridad (alta)
        &dw3000TaskHandle,   // Handle de la tarea
        0                    // Core 0
    );

    if (result == pdPASS) {
        Serial.println("[TAG] Tarea DW3000 creada en Core 0");
    } else {
        Serial.printf("[TAG] ERROR: No se pudo crear tarea DW3000 (error %d)\n", result);    
    }

    Serial.println("[TAG] Setup completado - Core 1 listo para otras tareas");


}
// Entrar en modo OTA (one-way hasta reinicio)
void enterOtaMode() {
    if (otaModeActive) return;
    Serial.println("\n[OTA] Activando modo OTA (deshabilitando ESP-NOW)...");
    otaModeActive = true;
    espNowActive = false;

    // Deshabilitar ESP-NOW si estaba activo
    esp_now_deinit();
    Serial.println("[OTA] ESP-NOW deshabilitado");

    // Mantener WiFi en modo STA y conectar a la red
    WiFi.mode(WIFI_STA);
    WiFi.begin(OTA_SSID, OTA_PASS);
    Serial.printf("[OTA] Conectando a WiFi SSID='%s' ...\n", OTA_SSID);

    unsigned long startConnect = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startConnect < 10000) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[OTA] WiFi conectado. IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[OTA] No se pudo conectar a WiFi (continuará intentando en background)");
    }

    // Configurar eventos OTA
    ArduinoOTA.onStart([](){
        Serial.println("[OTA] Inicio de actualización");
    });
    ArduinoOTA.onEnd([](){
        Serial.println("\n[OTA] Actualización completada");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){
        static unsigned int lastPct = 0;
        unsigned int pct = (progress * 100) / total;
        if (pct != lastPct) {
            Serial.printf("[OTA] Progreso: %u%%\n", pct);
            lastPct = pct;
        }
    });
    ArduinoOTA.onError([](ota_error_t error){
        Serial.printf("[OTA] Error %u: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.setHostname("TAG-UWB");
    ArduinoOTA.begin();
    Serial.println("[OTA] Servidor OTA listo. Mantén este modo hasta completar la actualización.");

    // Encender LED fijo para indicar modo OTA
    digitalWrite(5, HIGH);
}
//Given an accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if (accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if (accuracyNumber == 1) Serial.print(F("Low"));
  else if (accuracyNumber == 2) Serial.print(F("Medium"));
  else if (accuracyNumber == 3) Serial.print(F("High"));
}
void loop() {
    // === CORE 1 - Tareas de monitoreo y control ===
    // Activar streams como en el ejemplo (una sola vez) sin tocar setup
    


    // Lecturas analógicas
    readAnalogAxes();
    // Power button
    handlePowerButton();
    // Botones digitales restantes
    readButtons();

    // Detección de pulsación larga EXTENDIDA (10s) del botón OK para entrar a OTA
    if (!otaModeActive && buttons[BTN_IDX_OK].stable && (millis() - buttons[BTN_IDX_OK].pressStart >= OTA_LONG_PRESS_MS)) {
        enterOtaMode();
    }

    // Si estamos en OTA: sólo atender OTA y salir del loop
    if (otaModeActive) {
        ArduinoOTA.handle();
        delay(10);
        return; // Saltar resto (incluye ESP-NOW)
    }

    // Procesar cambios de modo
    processModeChanges();

    // Calcular valor de joystick si estamos en modo manual
    calculateJoystickValue();

    // Variables para envío ESP-NOW a 20Hz
    static unsigned long lastESPNowSend = 0;
    const unsigned long espNowInterval = 50; // 50ms = 20Hz

    // Leer voltaje de la batería cada 30 segundos
    static unsigned long lastBatteryRead = 0;
    static float current_battery_voltage = 0.0;
    unsigned long currentTime = millis();

    if (currentTime - lastBatteryRead >= 30000 || current_battery_voltage == 0.0) {
        current_battery_voltage = readBatteryVoltage();
        String battery_status = getBatteryStatus(current_battery_voltage);
        int battery_percentage = getBatteryPercentage(current_battery_voltage);
        Serial.printf("[BATTERY] %.2fV (%s - %d%%)\n",
                      current_battery_voltage, battery_status.c_str(), battery_percentage);  
        lastBatteryRead = currentTime;
    }

    // Enviar datos por ESP-NOW a 20Hz (paquete mínimo compatible con CarroClean)
    if (currentTime - lastESPNowSend >= espNowInterval) {
        if (!espNowActive) {
            lastESPNowSend = currentTime; // Evitar overflow lógico
        } else {
        // Actualizar datos ESP-NOW
        EspNowComm::toSend.batteryVoltage_mV = (uint16_t)(current_battery_voltage * 1000.0);
        EspNowComm::toSend.modo = (uint8_t)currentMode;
        // No enviar joystick en TILT (forzar 0). En MANUAL se envía el valor calculado.
        EspNowComm::toSend.joystick =  joystickValue;
        EspNowComm::toSend.timestamp = currentTime;
        // Nota: Se omiten cuaterniones/IMU en el paquete para mantener compatibilidad con CarroClean
        
        // Enviar datos con verificación mejorada
        esp_err_t result = esp_now_send(carroMacAddress, (uint8_t*)&EspNowComm::toSend, sizeof(EspNowComm::EspNowData));

        // Mostrar resultado detallado
        static int consecutive_errors = 0;
        if (result == ESP_OK) {
            consecutive_errors = 0;
            // Solo mostrar éxitos ocasionalmente para no saturar consola
            static int success_count = 0;
            if (++success_count % 40 == 0) { // Cada 2 segundos (40 * 50ms)
                const char* modeNames[] = {"APAGADO", "SEGUIMIENTO", "PAUSA", "MANUAL", "TILT"};
                const char* modeName = (currentMode <= 4) ? modeNames[currentMode] : "?";    
                Serial.printf("[ESP-NOW] OK: Batería=%.2fV, Modo=%s, J=%d, Envíos:%lu\n",
                              current_battery_voltage, modeName, joystickValue, (unsigned long)EspNowComm::totalPacketsSent);
            }
        } else {
            consecutive_errors++;
            Serial.printf("[ESP-NOW] ERROR %d: ", result);
            switch(result) {
                case ESP_ERR_ESPNOW_NOT_INIT:
                    Serial.println("ESP-NOW no inicializado");
                    break;
                case ESP_ERR_ESPNOW_ARG:
                    Serial.println("Argumento inválido");
                    break;
                case ESP_ERR_ESPNOW_INTERNAL:
                    Serial.println("Error interno");
                    break;
                case ESP_ERR_ESPNOW_NO_MEM:
                    Serial.println("Sin memoria");
                    break;
                case ESP_ERR_ESPNOW_NOT_FOUND:
                    Serial.println("Peer no encontrado - Verificar que Carro esté encendido");
                    break;
                case ESP_ERR_ESPNOW_IF:
                    Serial.println("Interfaz WiFi error");
                    break;
                default:
                    Serial.printf("Código error: %d\n", result);
                    break;
            }

            // Si hay muchos errores consecutivos, reintentar inicialización
            if (consecutive_errors >= 20) {
                Serial.println("[ESP-NOW] Demasiados errores, reinicializando...");
                esp_now_deinit();
                delay(1000);
                if (initESPNOW()) {
                    consecutive_errors = 0;
                    Serial.println("[ESP-NOW] Reinicialización exitosa");
                }
            }
        }

        lastESPNowSend = currentTime;
        }
    }

    // Estadísticas cada 60 segundos
    static unsigned long lastStats = 0;
    if (currentTime - lastStats >= 60000) {
        // Leer variables compartidas de forma thread-safe
        unsigned long local_total_cycles = 0;
        unsigned long local_successful_cycles = 0;
        unsigned long local_timeout_resets = 0;
        bool local_carro_detected = false;

        if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_total_cycles = total_cycles;
            local_successful_cycles = successful_cycles;
            local_timeout_resets = timeout_resets;
            local_carro_detected = carro_detected;
            xSemaphoreGive(dw3000Semaphore);
        }

        float success_rate = local_total_cycles > 0 ? (local_successful_cycles * 100.0) / local_total_cycles : 0;
    const char* modeNames[] = {"APAGADO", "SEGUIMIENTO", "PAUSA", "MANUAL", "TILT"};
    const char* modeName = (currentMode <= 4) ? modeNames[currentMode] : "?";
    Serial.printf("[STATS-Core1] Ciclos:%lu, Exitosos:%lu (%.1f%%), Resets:%lu, CARRO:%s, MODO:%s\n",
              local_total_cycles, local_successful_cycles, success_rate, local_timeout_resets,
              local_carro_detected ? "CONECTADO" : "DESCONECTADO", modeName);
        lastStats = currentTime;
    }

    // Pequeño delay para no saturar el Core 1
    static unsigned long lastAnalogDebug = 0;
    unsigned long dbgNow = millis();
    if (dbgNow - lastAnalogDebug >= 1000) {
        lastAnalogDebug = dbgNow;
        Serial.printf("[ANALOG] Y raw=%d V=%.2f n=%.2f | X raw=%d V=%.2f n=%.2f\n",
                      analogRightRaw, analogRightVolt, analogRightNorm,
                      analogDownRaw, analogDownVolt, analogDownNorm);
    }

    delay(10);
}