#include "RPiComm.h"
#include <ArduinoJson.h>

// UART2 para comunicación con Raspberry Pi
HardwareSerial RPiSerial(2);

// Variables internas
static unsigned long lastHeartbeat = 0;
static const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 segundos

void RPiComm_setup() {
    RPiSerial.begin(RPI_UART_SPEED, SERIAL_8N1, RPI_UART_RX_PIN, RPI_UART_TX_PIN);
    
    // Configurar pines ADC específicamente
    pinMode(CURRENT_L_PIN, INPUT);
    pinMode(CURRENT_R_PIN, INPUT);
    pinMode(BATTERY_V_PIN, INPUT);
    
    // Configuración ADC más específica
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Verificar que los pines ADC son válidos
    Serial.printf("[RPiComm] Configurando ADC - Pin corriente L: %d, R: %d, Batería: %d\n", 
                 CURRENT_L_PIN, CURRENT_R_PIN, BATTERY_V_PIN);
    
    // Test inicial de lecturas ADC
    delay(100);
    int test_adc_l = analogRead(CURRENT_L_PIN);
    int test_adc_r = analogRead(CURRENT_R_PIN);
    int test_adc_bat = analogRead(BATTERY_V_PIN);
    
    Serial.printf("[RPiComm] Test ADC inicial - L:%d, R:%d, Bat:%d (rango esperado: 100-4000)\n", 
                 test_adc_l, test_adc_r, test_adc_bat);
    
    if (test_adc_l == 0 && test_adc_r == 0) {
        Serial.println("[RPiComm] WARNING: Sensores de corriente parecen desconectados (ADC=0)");
    }
    
    Serial.println("[RPiComm] UART2 ready");
}

// (RPiComm_sendSystemData eliminado: no usado)

// (RPiComm_sendUWBData eliminado: no usado)

// (RPiComm_sendRawUWBData eliminado: no usado)

void RPiComm_sendRawUWBDataWithTAG(float distances[3], bool anchor_status[3], 
                           float frequency, unsigned long count,
                           float tag_battery_voltage, uint8_t tag_modo, uint8_t tag_joystick,
                           float tag_q_i, float tag_q_j, float tag_q_k, float tag_q_real, uint8_t tag_q_acc,
                           bool tag_data_valid) {
    // NUEVO FORMATO EXTENDIDO: Lista [d1,d2,d3,voltaje_bateria_carro,corriente_ML,corriente_MR,voltaje_bateria_tag,modo_tag,joystick_tag, q_i, q_j, q_k, q_r, q_acc]
    // Valores NaN/inválidos se envían como -1
    
    // Leer sensores del carro
    float battery_v = RPiComm_readBatteryVoltage();
    float motor_l_current = RPiComm_readCurrentSensor(CURRENT_L_PIN);
    float motor_r_current = RPiComm_readCurrentSensor(CURRENT_R_PIN);
    
    // Construir lista compacta extendida
    RPiSerial.print("[");
    
    // Distancias (d1, d2, d3) - NaN se convierte a -1
    for (int i = 0; i < 3; i++) {
        if (anchor_status[i] && !isnan(distances[i])) {
            RPiSerial.print(distances[i], 1);  // 1 decimal
        } else {
            RPiSerial.print(-1);  // Valor inválido
        }
        RPiSerial.print(",");
    }
    
    // Voltaje batería del carro
    if (!isnan(battery_v)) {
        RPiSerial.print(battery_v, 2);  // 2 decimales
    } else {
        RPiSerial.print(-1);
    }
    RPiSerial.print(",");
    
    // Corriente motor izquierdo
    if (!isnan(motor_l_current)) {
        RPiSerial.print(motor_l_current, 3);  // 3 decimales
    } else {
        RPiSerial.print(-1);
    }
    RPiSerial.print(",");
    
    // Corriente motor derecho
    if (!isnan(motor_r_current)) {
        RPiSerial.print(motor_r_current, 3);  // 3 decimales
    } else {
        RPiSerial.print(-1);
    }
    RPiSerial.print(",");
    
    // DATOS DEL TAG via ESP-NOW
    // Voltaje batería del TAG
    if (tag_data_valid && !isnan(tag_battery_voltage)) {
        RPiSerial.print(tag_battery_voltage, 2);  // 2 decimales
    } else {
        RPiSerial.print(-1);
    }
    RPiSerial.print(",");
    
    // Modo del TAG
    if (tag_data_valid) {
        RPiSerial.print(tag_modo);
    } else {
        RPiSerial.print(-1);
    }
    RPiSerial.print(",");
    
    // Joystick del TAG
    if (tag_data_valid) {
        RPiSerial.print(tag_joystick);
    } else {
        RPiSerial.print(-1);
    }
    RPiSerial.print(",");

    // Quaternion del TAG
    if (tag_data_valid) {
        RPiSerial.print(tag_q_i, 6);
        RPiSerial.print(",");
        RPiSerial.print(tag_q_j, 6);
        RPiSerial.print(",");
        RPiSerial.print(tag_q_k, 6);
        RPiSerial.print(",");
        RPiSerial.print(tag_q_real, 6);
        RPiSerial.print(",");
        RPiSerial.print(tag_q_acc);
    } else {
        RPiSerial.print(-1); RPiSerial.print(",");
        RPiSerial.print(-1); RPiSerial.print(",");
        RPiSerial.print(-1); RPiSerial.print(",");
        RPiSerial.print(-1); RPiSerial.print(",");
        RPiSerial.print(-1);
    }
    
    RPiSerial.println("]");
}

bool RPiComm_receiveCommand(MotorCommand &cmd) {
    if (!RPiSerial.available()) {
        return false;
    }
    
    String line = RPiSerial.readStringUntil('\n');
    line.trim();
    
    if (line.length() == 0) {
        return false;
    }
    
    // NUEVO FORMATO: [enable,ML,MR] donde enable es 0/1, ML y MR son -255 a 255
    if (line.startsWith("[") && line.endsWith("]")) {
        // Remover corchetes
        line = line.substring(1, line.length() - 1);
        
        // Separar por comas
        int comma1 = line.indexOf(',');
        int comma2 = line.indexOf(',', comma1 + 1);
        
        if (comma1 > 0 && comma2 > 0) {
            // Parsear valores
            int enable = line.substring(0, comma1).toInt();
            int motor_left = line.substring(comma1 + 1, comma2).toInt();
            int motor_right = line.substring(comma2 + 1).toInt();
            
            // Validar rangos
            enable = constrain(enable, 0, 1);
            motor_left = constrain(motor_left, -255, 255);
            motor_right = constrain(motor_right, -255, 255);
            
            // Configurar comando
            cmd.command_type = 'M';
            cmd.motor_left_speed = motor_left;
            cmd.motor_right_speed = motor_right;
            cmd.emergency_stop = (enable == 0);
            
            return true;
        }
    }
    
    // Fallback para comandos JSON antiguos (compatibilidad temporal)
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, line);
    
    if (!error) {
        const char* type = doc["type"];
        if (!type) {
            return false;
        }
        
        if (strcmp(type, "motor_command") == 0) {
            cmd.command_type = 'M';
            cmd.motor_left_speed = doc["left_speed"] | 0;
            cmd.motor_right_speed = doc["right_speed"] | 0;
            cmd.emergency_stop = doc["emergency_stop"] | false;
            
            // Limitar velocidades
            cmd.motor_left_speed = constrain(cmd.motor_left_speed, -255, 255);
            cmd.motor_right_speed = constrain(cmd.motor_right_speed, -255, 255);
            
            return true;
        }
        else if (strcmp(type, "emergency_stop") == 0) {
            cmd.command_type = 'S';
            cmd.motor_left_speed = 0;
            cmd.motor_right_speed = 0;
            cmd.emergency_stop = true;
            
            return true;
        }
        else if (strcmp(type, "ping") == 0) {
            cmd.command_type = 'P';
            
            // Responder con pong
            JsonDocument response;
            response["type"] = "pong";
            response["timestamp"] = millis();
            String output;
            serializeJson(response, output);
            RPiSerial.println(output);
            
            return false; // No es un comando de motor
        }
    }
    
    return false;
}

float RPiComm_readBatteryVoltage() {
    // Leer ADC múltiples veces para mayor precisión
    long sum = 0;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        sum += analogRead(BATTERY_V_PIN);
        delay(1);
    }
    
    float adc_average = sum / (float)samples;
    
    // DEBUG: Imprimir valores ADC
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {  // Debug cada 5 segundos
        Serial.printf("[BATTERY] ADC raw: %.1f, ", adc_average);
        lastDebug = millis();
    }
    
    // Convertir ADC a voltaje del divisor
    float voltage_divider = (adc_average / ADC_RESOLUTION) * ADC_VREF;
    
    // Calcular voltaje real de la batería
    // V_battery = V_divider * (R1 + R2) / R2
    float battery_voltage = voltage_divider * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    
    // DEBUG: Continuar debug
    if (millis() - lastDebug < 100) {  // Solo si acabamos de hacer debug arriba
        Serial.printf("V_div: %.3fV, V_bat: %.2fV\n", voltage_divider, battery_voltage);
    }
    
    return battery_voltage;
}

float RPiComm_readCurrentSensor(int pin) {
    // Leer ADC múltiples veces para mayor precisión
    long sum = 0;
    const int samples = 5;
    
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delay(1);
    }
    
    float adc_average = sum / (float)samples;
    
    // Convertir ADC a voltaje
    float voltage = (adc_average / ADC_RESOLUTION) * ADC_VREF;
    
    // Convertir voltaje a corriente para DRV8701
    const float AV = 20.0;          // Ganancia del amplificador de shunt
    const float V_OFF = 1.65;       // Voltaje de offset teórico
    const float R_SENSE = 0.005;    // Resistencia de sensado en Ohms (5 miliohms)
    
    // Calcular corriente con fórmula original
    float current = (voltage - V_OFF) / (AV * R_SENSE);
    
    // SIMPLE: Sumar 16.5A para corregir offset (como pediste)
    current = current + 16.5;
    
    // DEBUG: Imprimir valores cada 5 segundos
    static unsigned long lastCurrentDebug = 0;
    if (millis() - lastCurrentDebug > 5000) {
        Serial.printf("[CURRENT] Pin=%d, ADC=%.1f, V=%.3f -> I=%.3fA\n", 
                     pin, adc_average, voltage, current);
        lastCurrentDebug = millis();
    }
    
    // Filtrar valores negativos y limitar máximo
    if (current < 0.0) current = 0.0;
    if (current > 25.0) current = 25.0;
    
    return current;
}

void RPiComm_sendHeartbeat() {
    unsigned long now = millis();
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        JsonDocument doc;
        doc["type"] = "heartbeat";
        doc["timestamp"] = now;
        doc["uptime"] = now;
        doc["free_heap"] = ESP.getFreeHeap();
        
        String output;
        serializeJson(doc, output);
        RPiSerial.println(output);
        
        lastHeartbeat = now;
    }
}

void RPiComm_sendJSON(const char* json) {
    RPiSerial.println(json);
}

// (RPiComm_available eliminado: no usado)
