#include "UWBCore.h"
#include "DW3000.h"
#include <Arduino.h>

// Configuración de pines del multiplexor MCP23008
#define ANCHOR1_CS 0     // GP0 del MCP23008
#define ANCHOR1_RST 4    // GP4 del MCP23008
#define ANCHOR1_IRQ 35   // GPIO35 del ESP32 (directo)

#define ANCHOR2_CS 1     // GP1 del MCP23008
#define ANCHOR2_RST 5    // GP5 del MCP23008
#define ANCHOR2_IRQ 27   // GPIO27 del ESP32 (directo)

#define ANCHOR3_CS 2     // GP2 del MCP23008
#define ANCHOR3_RST 6    // GP6 del MCP23008
#define ANCHOR3_IRQ 13   // GPIO13 del ESP32 (directo)

// Instancias DW3000
static DW3000Class anchor1(ANCHOR1_CS, ANCHOR1_RST, ANCHOR1_IRQ);
static DW3000Class anchor2(ANCHOR2_CS, ANCHOR2_RST, ANCHOR2_IRQ);
static DW3000Class anchor3(ANCHOR3_CS, ANCHOR3_RST, ANCHOR3_IRQ);

// Variables compartidas entre cores
volatile UWBRawData g_uwb_raw_data = {NAN, NAN, NAN, false, false, false, 0};
volatile bool g_uwb_data_ready = false;
volatile unsigned long g_uwb_measurement_count = 0;
volatile unsigned long g_uwb_last_measurement_time = 0;


// No se usa filtro ni buffer

// Semáforo binario para acceso thread-safe entre cores
static SemaphoreHandle_t uwbSemaphore = NULL;

// Handle de la tarea
static TaskHandle_t uwbTaskHandle = NULL;

// Contadores de fallo por anchor para watchdog de recuperación
static uint16_t failCountA1 = 0;
static uint16_t failCountA2 = 0;
static uint16_t failCountA3 = 0;
static unsigned long lastRecoverLog = 0;


// Función para procesar el buffer y obtener mediciones filtradas

// No se usa filtro ni buffer

void UWBCore_setup() {
    // Crear semáforo binario para sincronización entre cores
    uwbSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(uwbSemaphore); // Inicializar como disponible
    
    Serial.println("[UWBCore] Inicializando anchors con timings para multiplexor MCP23008...");
    
    // TIMING CRÍTICO: El MCP23008 introduce ~2-5ms de latencia en I2C
    // Necesitamos delays más largos entre operaciones SPI consecutivas
    
    // Inicializar Anchor 1
    Serial.println("[UWBCore] Inicializando Anchor 1...");
    anchor1.begin();
    delay(50);  // Estabilización multiplexor
    anchor1.hardReset();
    delay(500); // CRÍTICO: 500ms para reset a través de multiplexor
    while (!anchor1.checkForIDLE()) {
        delay(200); // Mayor tiempo entre chequeos
    }
    anchor1.softReset();
    delay(500); // CRÍTICO: 500ms después de soft reset
    anchor1.init();
    delay(100); // Estabilización post-init
    anchor1.setupGPIO();
    delay(50);  // Estabilización GPIO
    anchor1.configureAsTX();
    delay(50);  // Estabilización TX config
    anchor1.clearSystemStatus();
    delay(100); // Pausa antes del siguiente anchor

    // Inicializar Anchor 2
    Serial.println("[UWBCore] Inicializando Anchor 2...");
    anchor2.begin();
    delay(50);  // Estabilización multiplexor
    anchor2.hardReset();
    delay(500); // CRÍTICO: 500ms para reset a través de multiplexor
    while (!anchor2.checkForIDLE()) {
        delay(200); // Mayor tiempo entre chequeos
    }
    anchor2.softReset();
    delay(500); // CRÍTICO: 500ms después de soft reset
    anchor2.init();
    delay(100); // Estabilización post-init
    anchor2.setupGPIO();
    delay(50);  // Estabilización GPIO
    anchor2.configureAsTX();
    delay(50);  // Estabilización TX config
    anchor2.clearSystemStatus();
    delay(100); // Pausa antes del siguiente anchor

    // Inicializar Anchor 3
    Serial.println("[UWBCore] Inicializando Anchor 3...");
    anchor3.begin();
    delay(50);  // Estabilización multiplexor
    anchor3.hardReset();
    delay(500); // CRÍTICO: 500ms para reset a través de multiplexor
    while (!anchor3.checkForIDLE()) {
        delay(200); // Mayor tiempo entre chequeos
    }
    anchor3.softReset();
    delay(500); // CRÍTICO: 500ms después de soft reset
    anchor3.init();
    delay(100); // Estabilización post-init
    anchor3.setupGPIO();
    delay(50);  // Estabilización GPIO
    anchor3.configureAsTX();
    delay(50);  // Estabilización TX config
    anchor3.clearSystemStatus();
    delay(100); // Estabilización final
    
    Serial.println("[UWBCore] Todos los anchors inicializados con timings optimizados");
}

void UWBCore_task(void* parameter) {
    // Esperar 3 segundos antes de empezar mediciones para estabilización
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("[UWBCore] Iniciando bucle de mediciones UWB 20Hz (sin recovery, sin anti-interferencia)...");

    while (true) {
        // Delays fijos para multiplexor y 20Hz
        const int base_delay_us = 10000;         // 10ms entre TX y RX (ajustable)
        const int inter_anchor_delay_us = 15000; // 15ms entre anchors (ajustable)
        const int cycle_delay_ms = 15;           // 20Hz

        UWBRawData rawData;
        rawData.timestamp = millis();

    // Anchor 1
        float distance1 = NAN;
    int t_roundA = 0, t_replyA = 0, clock_offset = 0, ranging_time = 0;
        long long rx = 0, tx = 0;
    int rx1 = 0, rx2v = 0;

        anchor1.ds_sendFrame(1);
        tx = anchor1.readTXTimestamp();
        delayMicroseconds(base_delay_us);
    rx1 = anchor1.receivedFrameSucc();
        if (rx1 == 1) {
            anchor1.clearSystemStatus();
            rx = anchor1.readRXTimestamp();
            anchor1.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor1.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(base_delay_us);
            rx2v = anchor1.receivedFrameSucc();
            if (rx2v == 1) {
                anchor1.clearSystemStatus();
                clock_offset = anchor1.getRawClockOffset();
                ranging_time = anchor1.ds_processRTInfo(t_roundA, t_replyA, anchor1.read(0x12, 0x04), anchor1.read(0x12, 0x08), clock_offset);
                distance1 = anchor1.convertToCM(ranging_time);
                if (distance1 < 0 || distance1 > 10000) distance1 = NAN;
                // éxito -> resetear contador de fallas
                failCountA1 = 0;
            } else {
                // fallo en segunda RX -> limpiar estado para evitar latch de RX_ERR
                anchor1.clearSystemStatus();
                if (failCountA1 < 0xFFFF) failCountA1++;
            }
        } else {
            // fallo en primera RX -> limpiar estado
            anchor1.clearSystemStatus();
            if (failCountA1 < 0xFFFF) failCountA1++;
        }
        delayMicroseconds(inter_anchor_delay_us);

        // Anchor 2
        float distance2 = NAN;
        anchor2.ds_sendFrame(1);
        tx = anchor2.readTXTimestamp();
        delayMicroseconds(base_delay_us);
    rx1 = anchor2.receivedFrameSucc();
        if (rx1 == 1) {
            anchor2.clearSystemStatus();
            rx = anchor2.readRXTimestamp();
            anchor2.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor2.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(base_delay_us);
            rx2v = anchor2.receivedFrameSucc();
            if (rx2v == 1) {
                anchor2.clearSystemStatus();
                clock_offset = anchor2.getRawClockOffset();
                ranging_time = anchor2.ds_processRTInfo(t_roundA, t_replyA, anchor2.read(0x12, 0x04), anchor2.read(0x12, 0x08), clock_offset);
                distance2 = anchor2.convertToCM(ranging_time);
                if (distance2 < 0 || distance2 > 10000) distance2 = NAN;
                failCountA2 = 0;
            } else {
                anchor2.clearSystemStatus();
                if (failCountA2 < 0xFFFF) failCountA2++;
            }
        } else {
            anchor2.clearSystemStatus();
            if (failCountA2 < 0xFFFF) failCountA2++;
        }
        delayMicroseconds(inter_anchor_delay_us);

        // Anchor 3
        float distance3 = NAN;
        anchor3.ds_sendFrame(1);
        tx = anchor3.readTXTimestamp();
        delayMicroseconds(base_delay_us);
    rx1 = anchor3.receivedFrameSucc();
        if (rx1 == 1) {
            anchor3.clearSystemStatus();
            rx = anchor3.readRXTimestamp();
            anchor3.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor3.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(base_delay_us);
            rx2v = anchor3.receivedFrameSucc();
            if (rx2v == 1) {
                anchor3.clearSystemStatus();
                clock_offset = anchor3.getRawClockOffset();
                ranging_time = anchor3.ds_processRTInfo(t_roundA, t_replyA, anchor3.read(0x12, 0x04), anchor3.read(0x12, 0x08), clock_offset);
                distance3 = anchor3.convertToCM(ranging_time);
                if (distance3 < 0 || distance3 > 10000) distance3 = NAN;
                failCountA3 = 0;
            } else {
                anchor3.clearSystemStatus();
                if (failCountA3 < 0xFFFF) failCountA3++;
            }
        } else {
            anchor3.clearSystemStatus();
            if (failCountA3 < 0xFFFF) failCountA3++;
        }


        // Enviar la medición directamente a 20 Hz (cada ciclo)
        if (xSemaphoreTake(uwbSemaphore, portMAX_DELAY) == pdTRUE) {
            g_uwb_raw_data.distance1 = distance1;
            g_uwb_raw_data.distance2 = distance2;
            g_uwb_raw_data.distance3 = distance3;
            g_uwb_raw_data.valid1 = !isnan(distance1);
            g_uwb_raw_data.valid2 = !isnan(distance2);
            g_uwb_raw_data.valid3 = !isnan(distance3);
            g_uwb_raw_data.timestamp = millis();
            g_uwb_data_ready = true;
            g_uwb_measurement_count++;
            g_uwb_last_measurement_time = millis();
            xSemaphoreGive(uwbSemaphore);
        }

        // Watchdog de recuperación ligero: si un anchor falla sostenidamente, aplicar soft reset
        const uint16_t softThreshold = 200;  // ~10s si 20Hz
        auto maybeRecover = [&](DW3000Class &anc, uint16_t &fc, const char *name){
            if (fc >= softThreshold) {
                if (millis() - lastRecoverLog > 2000) {
                    Serial.printf("[UWBCore] Watchdog: %s falló %u ciclos. Soft reset...\n", name, fc);
                    lastRecoverLog = millis();
                }
                // Soft recovery: softReset + init secuencia mínima
                anc.softReset();
                delay(100);
                anc.init();
                delay(50);
                anc.setupGPIO();
                delay(10);
                anc.configureAsTX();
                delay(10);
                anc.clearSystemStatus();
                fc = 0; // reiniciar contador tras intento de recovery
            }
        };
        maybeRecover(anchor1, failCountA1, "Anchor1");
        maybeRecover(anchor2, failCountA2, "Anchor2");
        maybeRecover(anchor3, failCountA3, "Anchor3");

        vTaskDelay(pdMS_TO_TICKS(cycle_delay_ms));
    }
}

void UWBCore_startTask() {
    Serial.println("[UWBCore] Iniciando tarea UWB en Core 0...");
    
    // Crear tarea en Core 0 con alta prioridad
    BaseType_t result = xTaskCreatePinnedToCore(
        UWBCore_task,        // Función de la tarea
        "UWB_Core_Task",     // Nombre de la tarea
        4096,                // Tamaño del stack
        NULL,                // Parámetros
        2,                   // Prioridad (alta)
        &uwbTaskHandle,      // Handle de la tarea
        0                    // Core 0
    );
    
    if (result == pdPASS) {
        Serial.println("[UWBCore] Tarea UWB creada exitosamente en Core 0");
    } else {
        Serial.printf("[UWBCore] ERROR: No se pudo crear la tarea UWB (error %d)\n", result);
    }
}

bool UWBCore_getRawData(UWBRawData& data) {
    // Leer datos de forma thread-safe usando semáforo
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        data.distance1 = g_uwb_raw_data.distance1;
        data.distance2 = g_uwb_raw_data.distance2;
        data.distance3 = g_uwb_raw_data.distance3;
        data.valid1 = g_uwb_raw_data.valid1;
        data.valid2 = g_uwb_raw_data.valid2;
        data.valid3 = g_uwb_raw_data.valid3;
        data.timestamp = g_uwb_raw_data.timestamp;
        bool dataReady = g_uwb_data_ready;
        if (g_uwb_data_ready) {
            g_uwb_data_ready = false; // Marcar como leído solo si había datos nuevos
        }
        xSemaphoreGive(uwbSemaphore);
        return true;
    }
    
    // Si no se puede obtener el semáforo, devolver datos por defecto
    Serial.println("[UWBCore] ERROR: No se pudo obtener el semáforo UWB, devolviendo datos por defecto");
    data.distance1 = NAN;
    data.distance2 = NAN;
    data.distance3 = NAN;
    data.valid1 = false;
    data.valid2 = false;
    data.valid3 = false;
    data.timestamp = millis();
    
    return false;
}

void UWBCore_diagnostics() {
    Serial.println("\n=== DIAGNÓSTICO UWB ===");
    
    // Verificar conectividad SPI con cada módulo
    Serial.println("Verificando conectividad SPI...");
    
    // Anchor 1 - Leer registro de identificación
    uint32_t dev_id1 = anchor1.read(0x00, 0x04); // Registro DEV_ID
    Serial.printf("Anchor 1 - Device ID: 0x%08X %s\n", dev_id1, 
                 (dev_id1 == 0xDECA0302 || dev_id1 != 0x00000000) ? "(OK)" : "(ERROR - Sin respuesta SPI)");
    
    // Anchor 2  
    uint32_t dev_id2 = anchor2.read(0x00, 0x04);
    Serial.printf("Anchor 2 - Device ID: 0x%08X %s\n", dev_id2,
                 (dev_id2 == 0xDECA0302 || dev_id2 != 0x00000000) ? "(OK)" : "(ERROR - Sin respuesta SPI)");
    
    // Anchor 3
    uint32_t dev_id3 = anchor3.read(0x00, 0x04);
    Serial.printf("Anchor 3 - Device ID: 0x%08X %s\n", dev_id3,
                 (dev_id3 == 0xDECA0302 || dev_id3 != 0x00000000) ? "(OK)" : "(ERROR - Sin respuesta SPI)");
    
    // Estado actual de las mediciones - thread-safe
    UWBRawData current_data;
    bool data_available = UWBCore_getRawData(current_data);
    
    Serial.printf("\nEstado actual:\n");
    Serial.printf("- Total mediciones: %lu\n", g_uwb_measurement_count);
    Serial.printf("- Última medición: %lu ms ago\n", millis() - g_uwb_last_measurement_time);
    Serial.printf("- Datos disponibles: %s\n", data_available ? "SÍ" : "NO");
    Serial.printf("- Datos válidos: A1=%d, A2=%d, A3=%d\n", 
                 current_data.valid1, current_data.valid2, current_data.valid3);
    Serial.printf("- Distancias: A1=%.1f, A2=%.1f, A3=%.1f cm\n",
                 current_data.distance1, current_data.distance2, current_data.distance3);
    
    Serial.println("======================\n");
}

unsigned long UWBCore_getMeasurementCount() {
    // Thread-safe access al contador
    unsigned long count = 0;
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = g_uwb_measurement_count;
        xSemaphoreGive(uwbSemaphore);
    }
    return count;
}

// UWBCore_recoverAnchors eliminado (no usado)
