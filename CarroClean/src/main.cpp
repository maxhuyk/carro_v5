#include <Arduino.h>
#include <Wire.h>
#include "DW3000.h"
#include "UWBCore.h"
#include <esp_now.h>
#include <WiFi.h>
#include "MotorController.h"
#include "RPiComm.h"
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

// Variables para el sistema de control en Core 1
TaskHandle_t controlTaskHandle = NULL;
CarroData sharedCarroData = {0};
volatile bool newUWBData = false;
SemaphoreHandle_t dataMutex = NULL;

// Declaraciones de funciones
void printControlStatus();
void updateSharedData(float distances[NUM_ANCHORS], bool anchor_status[NUM_ANCHORS], unsigned long measurement_count);
void controlTask(void* parameter);
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

// Tarea de control que se ejecuta en Core 1
void controlTask(void* parameter) {
    Serial.println("[CONTROL] Iniciando tarea de control en Core 1");
    
    // Configurar el control de motores directo
    setupMotorControlDirect();
    
    // Inicializar el sistema de control
    control_init();
    
    unsigned long lastControlTime = 0;
    const unsigned long CONTROL_INTERVAL = 10; // 100Hz para respuesta rápida
    
    while (true) {
        unsigned long currentTime = millis();
        
        if (currentTime - lastControlTime >= CONTROL_INTERVAL) {
            // Copiar datos compartidos de forma segura
            CarroData localData;
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                localData = sharedCarroData;
                xSemaphoreGive(dataMutex);
            } else {
                // Si no podemos obtener el mutex, usar datos anteriores
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            
            // Ejecutar el control principal
            control_main(&localData, motorControlCallback, [](){motor_enviar_pwm(0, 0);});
            
            lastControlTime = currentTime;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Pequeña pausa para no saturar el CPU
    }
}

// Función para actualizar los datos compartidos del carro
void updateSharedData(float distances[NUM_ANCHORS], bool anchor_status[NUM_ANCHORS], unsigned long measurement_count) {
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
            // Calcular yaw desde quaternion del TAG
            float yaw = atan2(2.0 * (receivedData.quatReal * receivedData.quatK + receivedData.quatI * receivedData.quatJ),
                             1.0 - 2.0 * (receivedData.quatJ * receivedData.quatJ + receivedData.quatK * receivedData.quatK));
            yaw = yaw * 180.0 / PI; // Convertir a grados
            
            // Llenar datos IMU: [PITCH, ROLL, YAW, MOV, VEL, ACCEL_Z]
            sharedCarroData.imu_data[0] = 0.0; // PITCH (no disponible)
            sharedCarroData.imu_data[1] = 0.0; // ROLL (no disponible)
            sharedCarroData.imu_data[2] = yaw; // YAW
            sharedCarroData.imu_data[3] = 0.0; // MOV (no disponible)
            sharedCarroData.imu_data[4] = 0.0; // VEL (no disponible)
            sharedCarroData.imu_data[5] = 0.0; // ACCEL_Z (no disponible)
            
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
        
        // Marcar que hay nuevos datos
        newUWBData = true;
        
        xSemaphoreGive(dataMutex);
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
  
  // Crear tarea de control en Core 1
  xTaskCreatePinnedToCore(
      controlTask,        // Función de la tarea
      "ControlTask",      // Nombre de la tarea
      8192,              // Tamaño del stack (8KB)
      NULL,              // Parámetros
      2,                 // Prioridad (mayor que loop)
      &controlTaskHandle, // Handle de la tarea
      1                  // Core 1
  );
  
  if (controlTaskHandle == NULL) {
      Serial.println("ERROR: No se pudo crear la tarea de control");
      return;
  }
  
  Serial.println("[CONTROL] Sistema de control inicializado en Core 1");
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

  

  ///////////////////////////////////////////////////////////////////////
  //                                                                   //
  //                          Seccion Raspberry                        //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////

  // Procesar comandos de la Raspberry Pi
  processSerialCommands();
  // Enviar datos a la RPi (datos en tiempo real desde UWBCore + TAG)
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
      // Hay nuevas mediciones, actualizar datos compartidos para el sistema de control
      updateSharedData(distances, anchor_status, currentCount);
      
      // Enviar a RPi con datos del TAG
      
  // Verificar si tenemos datos válidos del TAG (menos de 1 segundo de antigüedad)
      bool tagDataValid = (millis() - lastDataReceived) < 1000;
      float tagBatteryVoltage = tagDataValid ? (receivedData.batteryVoltage_mV / 1000.0) : 0.0;
      uint8_t tagModo = tagDataValid ? receivedData.modo : 0;
      uint8_t tagJoystick = tagDataValid ? receivedData.joystick : 0;
  // Quaternion
  float tagQuatI = tagDataValid ? receivedData.quatI : 0.0f;
  float tagQuatJ = tagDataValid ? receivedData.quatJ : 0.0f;
  float tagQuatK = tagDataValid ? receivedData.quatK : 0.0f;
  float tagQuatReal = tagDataValid ? receivedData.quatReal : 1.0f;
  uint8_t tagQuatAcc = tagDataValid ? receivedData.quatAccuracy : 0;
      
    // Enviar datos extendidos que incluyen información del TAG (incluye estabilidad y pasos)
      RPiComm_sendRawUWBDataWithTAG(distances, anchor_status, 20.0, currentCount,
              tagBatteryVoltage, tagModo, tagJoystick,
              tagQuatI, tagQuatJ, tagQuatK, tagQuatReal, tagQuatAcc,
              tagDataValid ? receivedData.bnoStability : 0,
              tagDataValid ? receivedData.stepCount : 0,
                    tagDataValid);
      
      lastMeasurementCount = currentCount;
      
      // Mostrar información ocasionalmente
      static int send_count = 0;
      if (++send_count % 40 == 0) { // Cada 2 segundos
          Serial.printf("[20Hz] Enviando: UWB[%.1f,%.1f,%.1f] TAG[%.2fV,%d,%d, qAcc=%d, stab=%u, steps=%lu] Valid:%s\n", 
                        distances[0], distances[1], distances[2],
                        tagBatteryVoltage, tagModo, tagJoystick, tagQuatAcc,
                        (unsigned)(tagDataValid ? receivedData.bnoStability : 0),
                        (unsigned long)(tagDataValid ? receivedData.stepCount : 0),
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
            Serial.printf("[CONTROL] Distancias UWB: S1=%.1fcm, S2=%.1fcm, S3=%.1fcm\n", 
                         sharedCarroData.distancias[0], sharedCarroData.distancias[1], sharedCarroData.distancias[2]);
            Serial.printf("[CONTROL] IMU Yaw: %.1f°\n", sharedCarroData.imu_data[2]);
            Serial.printf("[CONTROL] TAG Batería: %.2fV, Modo: %.0f\n", 
                         sharedCarroData.control_data[0], sharedCarroData.control_data[1]);
            Serial.printf("[CONTROL] Joystick: %.0f, Datos válidos: %s\n", 
                         sharedCarroData.buttons_data[0], sharedCarroData.data_valid ? "SI" : "NO");
            xSemaphoreGive(dataMutex);
        } else {
            Serial.println("[CONTROL] ERROR: No se pudo acceder a datos compartidos");
        }
        
        Serial.printf("[SISTEMA] Tarea de control: %s\n", 
                     (controlTaskHandle != NULL) ? "ACTIVA" : "INACTIVA");
        Serial.printf("[SISTEMA] Memoria libre: %d bytes\n", esp_get_free_heap_size());
        Serial.println("================================================\n");
        
        lastControlStatus = millis();
    }
}

