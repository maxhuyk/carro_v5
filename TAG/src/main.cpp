#include <Arduino.h>
#include "DW3000.h"
#include <esp_now.h>
#include <WiFi.h>

// Configuración de botones
#define BTN_RIGHT  35
#define BTN_LEFT   39
#define BTN_UP     14
#define BTN_DOWN   26
#define BTN_OK     25

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
uint8_t carroMacAddress[] = {0x00, 0x4B, 0x12, 0x32, 0x8E, 0xFC}; // Placeholder - cambiar por MAC real

// Estructura de datos para ESP-NOW (optimizada para tamaño)
typedef struct {
    uint16_t batteryVoltage_mV; // Voltaje en milivoltios (0-65535 mV = 0-65.535V)
    uint8_t modo;               // Modo (0-255, pero usaremos 0-7)
    uint8_t joystick;           // Joystick (0-255, pero usaremos 0-7)
    uint32_t timestamp;         // Timestamp para verificar frescura de datos
} __attribute__((packed)) EspNowData;

EspNowData espNowData;
volatile bool espNowSendReady = false;

// Sistema de modos
enum SystemMode {
    MODE_OFF = 0,       // Apagado
    MODE_TRACKING = 1,  // Seguimiento
    MODE_PAUSE = 2,     // Pausa
    MODE_MANUAL = 3     // Manual
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

// ESP-NOW Callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("[ESP-NOW] Envío ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ÉXITO" : "FALLO");
}

// Función para inicializar ESP-NOW
bool initESPNOW() {
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
    int pins[5] = {BTN_RIGHT, BTN_LEFT, BTN_UP, BTN_DOWN, BTN_OK};
    unsigned long now = millis();

    for (int i = 0; i < 5; i++) {
        bool r = !digitalRead(pins[i]); // Activo en LOW

        // Si cambia la lectura cruda, reiniciar temporizador de debounce
        if (r != buttons[i].raw) {
            buttons[i].raw = r;
            buttons[i].lastChange = now;
        }

        // Si la lectura se mantuvo estable por el tiempo de debounce, actualizar el estado estable
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
            // No hay cambio estable en este ciclo
            buttons[i].rising = false;
            buttons[i].falling = false;
        }

        // Detección de long-press (one-shot por pulsación)
        if (buttons[i].stable && !buttons[i].longPressFired && (now - buttons[i].pressStart >= LONG_PRESS_TIME)) {
            buttons[i].longPressFired = true;
        }
    }
}

// Función para procesar cambios de modo
void processModeChanges() {
    unsigned long now = millis();
    // Ventana de guardia para evitar rebotes de modo (misma pulsación causando ida y vuelta)
    if (now - lastModeChangeTime < MODE_CHANGE_GUARD_MS) {
        return;
    }

    auto resetButtonsAfterModeChange = [&](){
        lastModeChangeTime = now;
        // Limpiar edges y long-press para no re-disparar con la misma pulsación
        for (int i = 0; i < 5; i++) {
            buttons[i].previousStable = buttons[i].stable;
            buttons[i].rising = false;
            buttons[i].falling = false;
            buttons[i].longPressFired = false;
            buttons[i].pressStart = now;
            buttons[i].lastChange = now;
        }
    };

    // OK presionado largo
    if (buttons[BTN_IDX_OK].longPressFired) {
        // Consumir el evento hasta próxima pulsación
        buttons[BTN_IDX_OK].longPressFired = false;

        if (currentMode == MODE_OFF) {
            currentMode = MODE_TRACKING;
            Serial.println("[MODE] Cambiado a SEGUIMIENTO");
            resetButtonsAfterModeChange();
        } else if (currentMode == MODE_MANUAL) {
            currentMode = MODE_OFF;
            Serial.println("[MODE] Cambiado a APAGADO");
            resetButtonsAfterModeChange();
        } else {
            currentMode = MODE_OFF;
            Serial.println("[MODE] Cambiado a APAGADO");
            resetButtonsAfterModeChange();
        }
    }
    
    // UP presionado largo (cualquier modo -> MANUAL)
    if (buttons[BTN_IDX_UP].longPressFired) {
        buttons[BTN_IDX_UP].longPressFired = false; // consumir
        currentMode = MODE_MANUAL;
        Serial.println("[MODE] Cambiado a MANUAL");
        resetButtonsAfterModeChange();
    }
    
    // DOWN rising (TRACKING -> PAUSE)
    if (buttons[BTN_IDX_DOWN].rising) {
        if (currentMode == MODE_TRACKING) {
            currentMode = MODE_PAUSE;
            Serial.println("[MODE] Cambiado a PAUSA");
            resetButtonsAfterModeChange();
            return; // evitar que el mismo ciclo procese salida de PAUSA
        }
    }
    
    // Cualquier botón en PAUSE -> TRACKING
    if (currentMode == MODE_PAUSE) {
        bool anyButtonPressed = false;
        for (int i = 0; i < 5; i++) {
            if (buttons[i].rising) {
                anyButtonPressed = true;
                break;
            }
        }
        if (anyButtonPressed) {
            currentMode = MODE_TRACKING;
            Serial.println("[MODE] Cambiado a SEGUIMIENTO (desde pausa)");
            resetButtonsAfterModeChange();
        }
    }
}

// Función para calcular valor de joystick en modo manual
void calculateJoystickValue() {
    if (currentMode != MODE_MANUAL) {
        joystickValue = 0; // 0 = sin comando
        return;
    }

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

    // Nuevo mapeo: 0 = nada; 1..8 = direcciones
    if (up && !left && !right) {
        joystickValue = 1; // adelante
    } else if (up && left) {
        joystickValue = 2; // adelante-izquierda
    } else if (!up && !down && left) {
        joystickValue = 3; // izquierda
    } else if (down && left) {
        joystickValue = 4; // atrás-izquierda
    } else if (down && !left && !right) {
        joystickValue = 5; // atrás
    } else if (down && right) {
        joystickValue = 6; // atrás-derecha
    } else if (!up && !down && right) {
        joystickValue = 7; // derecha
    } else if (up && right) {
        joystickValue = 8; // adelante-derecha
    } else {
        joystickValue = 0; // sin movimiento
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
    unsigned long communication_timeout = 300;
    bool local_carro_detected = false;
    unsigned long local_total_cycles = 0;
    unsigned long local_successful_cycles = 0;
    unsigned long local_timeout_resets = 0;

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
            if (local_carro_detected) {
                Serial.println("[DW3000-Core0] CARRO desconectado - Reinicializando...");
                local_carro_detected = false;
                local_timeout_resets++;
                
                // Re-inicializar DW3000
                dw.softReset();
                delay(100);
                dw.init();
                delay(20);
                dw.setupGPIO();
                delay(10);
                dw.configureAsTX();
                delay(10);
                dw.clearSystemStatus();
                delay(5);
                dw.standardRX();
                
                curr_stage = 0;
                
                // Actualizar variables compartidas thread-safe
                if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                    carro_detected = false;
                    timeout_resets = local_timeout_resets;
                    xSemaphoreGive(dw3000Semaphore);
                }
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
    pinMode(5, OUTPUT); // LED en GPIO 5
    digitalWrite(5, LOW);
    Serial.begin(500000);
    
    // Blink de arranque
    digitalWrite(5, HIGH); delay(100); digitalWrite(5, LOW); delay(100); 
    digitalWrite(5, HIGH); delay(100); digitalWrite(5, LOW);

    // Configurar el pin de lectura de batería
    pinMode(BATTERY_PIN, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Configurar botones
    // NOTA: GPIO34-39 no tienen PULLUP interno. BTN_RIGHT(35) y BTN_LEFT(39) requieren pull-up externo.
    pinMode(BTN_RIGHT, INPUT); // usar pull-up externo
    pinMode(BTN_LEFT, INPUT);  // usar pull-up externo
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
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
    espNowData.modo = 0;      // Modo inicial (apagado)
    espNowData.joystick = 0;  // Joystick inicial
    
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

void loop() {
    // === CORE 1 - Tareas de monitoreo y control ===
    
    // Leer botones (no bloqueante)
    readButtons();
    
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
    
    // Enviar datos por ESP-NOW a 20Hz
    if (currentTime - lastESPNowSend >= espNowInterval) {
        // Actualizar datos ESP-NOW
        espNowData.batteryVoltage_mV = (uint16_t)(current_battery_voltage * 1000.0);
        espNowData.modo = (uint8_t)currentMode;
        espNowData.joystick = joystickValue;
        espNowData.timestamp = currentTime;
        
        // Enviar datos con verificación mejorada
        esp_err_t result = esp_now_send(carroMacAddress, (uint8_t*)&espNowData, sizeof(espNowData));
        
        // Mostrar resultado detallado
        static int consecutive_errors = 0;
        if (result == ESP_OK) {
            consecutive_errors = 0;
            // Solo mostrar éxitos ocasionalmente para no saturar consola
            static int success_count = 0;
            if (++success_count % 40 == 0) { // Cada 2 segundos (40 * 50ms)
                const char* modeNames[] = {"APAGADO", "SEGUIMIENTO", "PAUSA", "MANUAL"};
                Serial.printf("[ESP-NOW] OK: Batería=%.2fV, Modo=%s, Joystick=%d\n", 
                              current_battery_voltage, modeNames[currentMode], joystickValue);
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
        const char* modeNames[] = {"APAGADO", "SEGUIMIENTO", "PAUSA", "MANUAL"};
        Serial.printf("[STATS-Core1] Ciclos:%lu, Exitosos:%lu (%.1f%%), Resets:%lu, CARRO:%s, MODO:%s\n",
                      local_total_cycles, local_successful_cycles, success_rate, local_timeout_resets, 
                      local_carro_detected ? "CONECTADO" : "DESCONECTADO", modeNames[currentMode]);
        lastStats = currentTime;
    }
    
    // Pequeño delay para no saturar el Core 1
    delay(10);
}
