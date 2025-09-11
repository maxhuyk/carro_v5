#include <Arduino.h>
#include <Wire.h>
#include "DW3000.h"
#include "UWBCore.h"
#include <esp_now.h>
#include <WiFi.h>
#include "MotorController.h"
#include "control.h"
#include "config.h"




// Estructura de datos para ESP-NOW (debe coincidir con el TAG)
typedef struct {
    uint16_t batteryVoltage_mV; // Voltaje en milivoltios
    uint8_t modo;               // Modo (0-7)
    uint8_t joystick;           // Joystick (0-7)
    uint32_t timestamp;         // Timestamp
    // Quaternion del TAG (BNO080)
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
    uint8_t quatAccuracy;       // 0..3
    uint8_t bnoStability;       // Estado de estabilidad BNO080
    uint32_t stepCount;         // Conteo de pasos
} __attribute__((packed)) EspNowData;

// Variables globales para datos recibidos del TAG
volatile EspNowData receivedData = {0, 0, 0, 0};
volatile bool newDataReceived = false;
volatile unsigned long lastDataReceived = 0;
volatile unsigned long totalPacketsReceived = 0;

// Variables para el sistema de control sincronizado con UWB
CarroData sharedCarroData = {0};
SemaphoreHandle_t dataMutex = NULL;

// Declaraciones de funciones
void printControlStatus();
void updateSharedDataAndRunControl(float distances[NUM_ANCHORS], bool anchor_status[NUM_ANCHORS], unsigned long measurement_count);
void motorControlCallback(float velocidad_izq, float velocidad_der);

// Callback para recepción de datos ESP-NOW
void onDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(EspNowData)) {
        memcpy((void*)&receivedData, incomingData, sizeof(EspNowData));
        newDataReceived = true;
        lastDataReceived = millis();
        totalPacketsReceived++;
        
        // Solo mostrar cada 40 paquetes para no saturar consola (cada 2 segundos)
        if (totalPacketsReceived % 40 == 0) {
            float voltage = receivedData.batteryVoltage_mV / 1000.0;
            Serial.printf("[ESP-NOW] Recibido #%lu: Batería=%.2fV, Modo=%d, Joy=%d, qAcc=%d, Stab=%u, Steps=%lu\n", 
                          totalPacketsReceived, voltage, receivedData.modo, receivedData.joystick, receivedData.quatAccuracy,
                          (unsigned)receivedData.bnoStability, (unsigned long)receivedData.stepCount);
        }
    } else {
        Serial.printf("[ESP-NOW] Tamaño de datos incorrecto: %d bytes (esperado %d)\n", len, sizeof(EspNowData));
    }
}

// Función para inicializar ESP-NOW en el Carro
bool initESPNOW() {
    // Configurar WiFi en modo Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // Asegurar que no esté conectado a ninguna red
    
    // Mostrar MAC Address del Carro
    Serial.print("[ESP-NOW] CARRO MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("[ESP-NOW] Copiar esta MAC al TAG en carroMacAddress[]");
    
    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error inicializando ESP-NOW");
        return false;
    }
    
    // Registrar callback de recepción
    esp_now_register_recv_cb(onDataReceived);
    
    Serial.println("[ESP-NOW] Inicializado correctamente como receptor");
    Serial.println("[ESP-NOW] Esperando datos del TAG...");
    return true;
}

// Función de callback para el control de motores
void motorControlCallback(float velocidad_izq, float velocidad_der) {
    // Enviar velocidades directamente al controlador de motores
    motor_enviar_pwm(velocidad_izq, velocidad_der);
}

// Función que actualiza datos y ejecuta control inmediatamente después de mediciones UWB
void updateSharedDataAndRunControl(float distances[NUM_ANCHORS], bool anchor_status[NUM_ANCHORS], unsigned long measurement_count) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // Limpiar estructura
        memset(&sharedCarroData, 0, sizeof(CarroData));
        
        // Actualizar datos de sensores UWB (convertir de cm a mm)
        for (int i = 0; i < NUM_ANCHORS && i < 3; i++) {
            sharedCarroData.distancias[i] = anchor_status[i] ? (distances[i] * 10.0) : -1.0; // cm -> mm
        }
        
        // Actualizar datos del TAG si están disponibles
        bool tagDataValid = (millis() - lastDataReceived) < 1000;
        if (tagDataValid) {
            
            
            // Llenar datos de control: [BAT_TAG, MODO]
            sharedCarroData.control_data[0] = receivedData.batteryVoltage_mV / 1000.0; // BAT_TAG
            sharedCarroData.control_data[1] = receivedData.modo; // MODO
            
            // Llenar datos de botones: [JOY_D, JOY_A, JOY_I, JOY_T]
            sharedCarroData.buttons_data[0] = receivedData.joystick; // JOY_D (dirección)
            sharedCarroData.buttons_data[1] = 0.0; // JOY_A (no disponible)
            sharedCarroData.buttons_data[2] = 0.0; // JOY_I (no disponible)
            sharedCarroData.buttons_data[3] = 0.0; // JOY_T (no disponible)
            
            sharedCarroData.data_valid = true;
        } else {
            sharedCarroData.data_valid = false;
        }
        
        xSemaphoreGive(dataMutex);
        
        // *** EJECUTAR CONTROL INMEDIATAMENTE DESPUÉS DE MEDICIONES UWB ***
        // Esto garantiza que el control se ejecute a la misma frecuencia que las mediciones
        
        // En modo 0 (APAGADO), imprimir distancias DW3000 para diagnóstico
        if (tagDataValid && receivedData.modo == 0) {
            static unsigned long lastPrintMode0 = 0;
            if (millis() - lastPrintMode0 > 1000) { // Imprimir cada segundo en modo 0
                Serial.printf("[MODO_0_DEBUG] Distancias DW3000: ");
                for (int i = 0; i < NUM_ANCHORS; i++) {
                    if (anchor_status[i]) {
                        Serial.printf("A%d:%.1fcm ", i+1, distances[i]);
                    } else {
                        Serial.printf("A%d:FAIL ", i+1);
                    }
                }
                Serial.printf("| Count:%lu\n", measurement_count);
                lastPrintMode0 = millis();
            }
        }
        
        control_main(&sharedCarroData, motorControlCallback, [](){motor_enviar_pwm(0, 0);});
    }
}


void setup() {
  Serial.begin(500000);  // Reducido de 2000000 a 500000 para mejor estabilidad
    delay(1000);
  // Inicializar I2C
    Wire.begin(5, 4); // SDA=GPIO5, SCL=GPIO4
    Wire.setClock(500000);
     // Inicializar ESP-NOW PRIMERO
    if (!initESPNOW()) {
        Serial.println("ERROR: ESP-NOW initialization failed");
        return;
    }
    // Inicializar sistemas UWB (ahora dual-core)
    if (!DW3000Class::initMCP23008()) {
        Serial.println("ERROR: MCP23008 initialization failed");
        return;
    }

  ///////////////////////////////////////////////////////////////////////
  //                                                                   //
  //                             Seccion UWB                           //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////

  UWBCore_setup();
  UWBCore_startTask();
  
  setupMotorController();  //setup motores 
  
  ///////////////////////////////////////////////////////////////////////
  //                                                                   //
  //                         Seccion Control                           //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////
  
  // Crear mutex para datos compartidos
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
      Serial.println("ERROR: No se pudo crear el mutex para datos compartidos");
      return;
  }
  
  // Inicializar estructura de datos compartidos
  memset(&sharedCarroData, 0, sizeof(CarroData));
  
  // Configurar el control de motores directo
  setupMotorControlDirect();
  
  // Inicializar el sistema de control
  control_init();
  
  Serial.println("[CONTROL] Sistema de control inicializado - sincronizado con UWB");
}

void loop() {

  ///////////////////////////////////////////////////////////////////////
  //                                                                   //
  //                             Seccion UWB                           //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////

  // Actualizar datos UWB
  UWBCore_update();

  // Solo obtener distancias raw para enviar a la RPi
  float distances[NUM_ANCHORS];
  bool anchor_status[NUM_ANCHORS];
  UWBCore_getDistances(distances);
  UWBCore_getAnchorStatus(anchor_status);
// Mostrar información UWB en consola cada 5 segundos
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay > 5000) {
        unsigned long count = UWBCore_getMeasurementCount();

        Serial.printf("[UWB] Count: %lu | ", count);
        
        for (int i = 0; i < NUM_ANCHORS; i++) {
            Serial.printf("A%d:", i+1);
            if (anchor_status[i]) {
                Serial.printf("%.1fcm ", distances[i]);
            } else {
                Serial.print("FAIL ");
            }
        }
        Serial.println();
        lastDisplay = millis();
    }

  

  

  
  static unsigned long lastMeasurementCount = 0;
  unsigned long currentCount = UWBCore_getMeasurementCount();
  

  ///////////////////////////////////////////////////////////////////////
  //                                                                   //
  //                          Seccion ESP-NOW                          //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////



  // Procesar datos recibidos de ESP-NOW
    if (newDataReceived) {
        newDataReceived = false;
        
        // Aquí puedes procesar los datos recibidos del TAG
        float tagBatteryVoltage = receivedData.batteryVoltage_mV / 1000.0;
        uint8_t tagModo = receivedData.modo;
        uint8_t tagJoystick = receivedData.joystick;
        
        // Ejemplo: usar los datos recibidos para control del motor o envío a RPi
        // processTagCommand(tagModo, tagJoystick);
        // sendTagDataToRPi(tagBatteryVoltage, tagModo, tagJoystick);
    }
    
    // Verificar timeout de datos ESP-NOW
    static unsigned long lastEspNowStatus = 0;
    if (millis() - lastEspNowStatus >= 5000) { // Cada 5 segundos
        unsigned long timeSinceLastData = millis() - lastDataReceived;
        if (timeSinceLastData > 1000) { // Más de 1 segundo sin datos
            Serial.printf("[ESP-NOW] WARNING: Sin datos del TAG por %lu ms\n", timeSinceLastData);
        } else {
            Serial.printf("[ESP-NOW] OK: Último dato hace %lu ms, Total recibidos: %lu\n", 
                          timeSinceLastData, totalPacketsReceived);
        }
        lastEspNowStatus = millis();
    }

  ///////////////////////////////////////////////////////////////////////
  //                                                                   //
  //                   Seccion ESP-NOW / Raspberry                     //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////

  if (currentCount > lastMeasurementCount) {
      // *** NUEVA IMPLEMENTACIÓN: CONTROL SINCRONIZADO CON UWB ***
      // Actualizar datos compartidos Y ejecutar control inmediatamente
      updateSharedDataAndRunControl(distances, anchor_status, currentCount);
      

      
  // Verificar si tenemos datos válidos del TAG (menos de 1 segundo de antigüedad)
      bool tagDataValid = (millis() - lastDataReceived) < 1000;
      float tagBatteryVoltage = tagDataValid ? (receivedData.batteryVoltage_mV / 1000.0) : 0.0;
      uint8_t tagModo = tagDataValid ? receivedData.modo : 0;
      uint8_t tagJoystick = tagDataValid ? receivedData.joystick : 0;
  
    
      lastMeasurementCount = currentCount;
      
      // Mostrar información ocasionalmente (cada ~2 segundos a frecuencia UWB)
      static int send_count = 0;
      if (++send_count % 20 == 0) { // Ajustado para frecuencia real del UWB
          Serial.printf("[UWB+CONTROL] UWB[%.1f,%.1f,%.1f] TAG[%.2fV,%d] Valid:%s Control:SYNC\n", 
                        distances[0], distances[1], distances[2],
                        tagBatteryVoltage, tagModo, tagJoystick, 
                        tagDataValid ? "SI" : "NO");
      }
  }
  
  // Mostrar estado del sistema de control (cada 10 segundos)
  printControlStatus();
}

// Función para mostrar el estado del sistema de control (opcional)
void printControlStatus() {
    static unsigned long lastControlStatus = 0;
    if (millis() - lastControlStatus >= 10000) { // Cada 10 segundos
        Serial.println("\n========= ESTADO DEL SISTEMA DE CONTROL =========");
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.printf("[CONTROL] Distancias UWB: S1=%.1fmm, S2=%.1fmm, S3=%.1fmm\n", 
                         sharedCarroData.distancias[0], sharedCarroData.distancias[1], sharedCarroData.distancias[2]);
            Serial.printf("[CONTROL] TAG Batería: %.2fV, Modo: %.0f\n", 
                         sharedCarroData.control_data[0], sharedCarroData.control_data[1]);
            Serial.printf("[CONTROL] Joystick: %.0f, Datos válidos: %s\n", 
                         sharedCarroData.buttons_data[0], sharedCarroData.data_valid ? "SI" : "NO");
            xSemaphoreGive(dataMutex);
        } else {
            Serial.println("[CONTROL] ERROR: No se pudo acceder a datos compartidos");
        }
        
        // Obtener frecuencia real del UWB
        unsigned long uwbCount = UWBCore_getMeasurementCount();
        static unsigned long lastUWBCount = 0;
        static unsigned long lastFreqCheck = millis();
        unsigned long currentTime = millis();
        float uwbFreq = (uwbCount - lastUWBCount) * 1000.0 / (currentTime - lastFreqCheck);
        lastUWBCount = uwbCount;
        lastFreqCheck = currentTime;
        
        Serial.printf("[SISTEMA] Control sincronizado con UWB: %.1f Hz\n", uwbFreq);
        Serial.printf("[SISTEMA] Total mediciones UWB: %lu\n", uwbCount);
        Serial.printf("[SISTEMA] Memoria libre: %d bytes\n", esp_get_free_heap_size());
        Serial.println("================================================\n");
        
        lastControlStatus = millis();
    }
}

