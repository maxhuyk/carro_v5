#include <Arduino.h>
#include <Wire.h>
#include "DW3000.h"
#include "UWBCore.h"

#define NUM_ANCHORS 3 

void setup() {
  Serial.begin(500000);  // Reducido de 2000000 a 500000 para mejor estabilidad
    delay(1000);
  // Inicializar I2C
    Wire.begin(5, 4); // SDA=GPIO5, SCL=GPIO4
    Wire.setClock(500000);
    
    // Inicializar sistemas UWB (ahora dual-core)
    if (!DW3000Class::initMCP23008()) {
        Serial.println("ERROR: MCP23008 initialization failed");
        return;
    }
  UWBCore_setup();
  UWBCore_startTask();

}

void loop() {

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
}

