#include <Arduino.h>
#include <Wire.h>
#include "DW3000.h"
#include "UWBCore.h"
#include <esp_now.h>
#include <WiFi.h>
#include "MotorController.h"
#include "RPiComm.h"




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
} __attribute__((packed)) EspNowData;

// Variables globales para datos recibidos del TAG
volatile EspNowData receivedData = {0, 0, 0, 0};
volatile bool newDataReceived = false;
volatile unsigned long lastDataReceived = 0;
volatile unsigned long totalPacketsReceived = 0;

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
            Serial.printf("[ESP-NOW] Recibido #%lu: Batería=%.2fV, Modo=%d, Joystick=%d, QuatAcc=%d\n", 
                          totalPacketsReceived, voltage, receivedData.modo, receivedData.joystick, receivedData.quatAccuracy);
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
      // Hay nuevas mediciones, enviar a RPi con datos del TAG
      
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
      
      // Enviar datos extendidos que incluyen información del TAG
      RPiComm_sendRawUWBDataWithTAG(distances, anchor_status, 20.0, currentCount,
                    tagBatteryVoltage, tagModo, tagJoystick,
                    tagQuatI, tagQuatJ, tagQuatK, tagQuatReal, tagQuatAcc,
                    tagDataValid);
      
      lastMeasurementCount = currentCount;
      
      // Mostrar información ocasionalmente
      static int send_count = 0;
      if (++send_count % 40 == 0) { // Cada 2 segundos
          Serial.printf("[20Hz] Enviando: UWB[%.1f,%.1f,%.1f] TAG[%.2fV,%d,%d, qAcc=%d] Valid:%s\n", 
                        distances[0], distances[1], distances[2],
                        tagBatteryVoltage, tagModo, tagJoystick, tagQuatAcc,
                        tagDataValid ? "SI" : "NO");
      }
  }
}

