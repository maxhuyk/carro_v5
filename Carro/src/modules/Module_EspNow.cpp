#include "modules/Module_EspNow.h"
#include <Arduino.h>
#include <math.h>
#include "config/Config.h"

// ===== Migración literal desde CarroClean (main.cpp sección ESP-NOW) =====
// Estructura de datos recibidos
struct EspNowData {
  uint8_t  version;            // debe ser 2
  uint16_t batteryVoltage_mV;  // mV batería
  uint8_t  modo;               // modo 0..4
  uint16_t ax_raw;             // eje X crudo 0..4095
  uint16_t ay_raw;             // eje Y crudo 0..4095
  uint32_t timestamp;          // millis() del TAG
} __attribute__((packed));

static EspNowSharedState g_state; // estado compartido
EspNowSharedState& EspNow_getState(){ return g_state; }

static volatile EspNowData g_lastData = {0};
static volatile bool g_newData = false;

// Parámetros de mapping (literal CarroClean)
static constexpr int RAW_MIN    = 600;   // extremo negativo
static constexpr int RAW_MAX    = 3400;  // extremo positivo
static constexpr int RAW_CENTER = 2000;  // centro físico
static constexpr int RAW_DEAD   = 200;   // +/- ventana muerta

static float mapAxis(int raw){
  if (raw < RAW_CENTER - RAW_DEAD) {
    float norm = (float)(raw - (RAW_CENTER - RAW_DEAD)) / (float)(RAW_MIN - (RAW_CENTER - RAW_DEAD));
    if (norm < 0) norm = 0; if (norm > 1) norm = 1; return -norm; // negativo
  } else if (raw > RAW_CENTER + RAW_DEAD) {
    float norm = (float)(raw - (RAW_CENTER + RAW_DEAD)) / (float)(RAW_MAX - (RAW_CENTER + RAW_DEAD));
    if (norm < 0) norm = 0; if (norm > 1) norm = 1; return norm; // positivo
  } else { return 0.0f; }
}

static float clampf(float v, float mn, float mx){ return v < mn ? mn : (v > mx ? mx : v); }

static void onDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len){
  if(len != (int)sizeof(EspNowData)){
    LOGW("ESP","Tamaño inesperado %d (esperado %d)", len, sizeof(EspNowData));
    return;
  }
  EspNowData tmp; memcpy(&tmp, incomingData, sizeof(EspNowData));
  if(tmp.version != 2){ LOGW("ESP","Version desconocida %u (esperada 2)", tmp.version); return; }
  // Copia campo a campo porque g_lastData es volatile
  g_lastData.version = tmp.version;
  g_lastData.batteryVoltage_mV = tmp.batteryVoltage_mV;
  g_lastData.modo = tmp.modo;
  g_lastData.ax_raw = tmp.ax_raw;
  g_lastData.ay_raw = tmp.ay_raw;
  g_lastData.timestamp = tmp.timestamp;
  g_newData = true; g_state.totalPackets++; g_state.lastRxMs = millis();
  g_state.battery_mV = tmp.batteryVoltage_mV;
  g_state.modo = tmp.modo;
  // Calcular velocidades manuales proporcionales
  float xNorm = mapAxis(tmp.ax_raw);
  float yNorm = mapAxis(tmp.ay_raw);
  const float maxManual = (float)VELOCIDAD_MANUAL; // puede cambiarse a VELOCIDAD_MAXIMA si se desea
  float vBase = yNorm * maxManual;
  float vTurn = xNorm * maxManual;
  float vL = vBase - vTurn;
  float vR = vBase + vTurn;
  vL = clampf(vL, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
  vR = clampf(vR, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
  g_state.manualVelL = (int16_t)lroundf(vL);
  g_state.manualVelR = (int16_t)lroundf(vR);
  g_state.valid = true; // se validará edad en loop
  if(g_state.totalPackets % 40 == 0){
    float voltage = g_state.battery_mV / 1000.0f;
    LOGI("ESP","Recibido #%lu V=%.2f Modo=%u ax=%u ay=%u vL=%d vR=%d", (unsigned long)g_state.totalPackets, voltage, g_state.modo, (unsigned)tmp.ax_raw, (unsigned)tmp.ay_raw, (int)g_state.manualVelL, (int)g_state.manualVelR);
  }
}

static bool initESPNOW(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("[ESP-NOW] CARRO MAC Address: "); Serial.println(WiFi.macAddress());
  Serial.println("[ESP-NOW] Copiar esta MAC al TAG en carroMacAddress[]");
  if(esp_now_init()!=ESP_OK){ Serial.println("[ESP-NOW] Error inicializando ESP-NOW"); return false; }
  esp_now_register_recv_cb(onDataReceived);
  Serial.println("[ESP-NOW] Inicializado correctamente como receptor");
  Serial.println("[ESP-NOW] Esperando datos del TAG...");
  return true;
}

bool Module_EspNow::setup(){
  LOGI("ESP","Inicializando ESP-NOW (literal)");
  if(!initESPNOW()) return false;
  return true;
}

void Module_EspNow::loop(){
  // Validar edad de datos (<1s) para marcar valid flag
  if(g_state.valid){
    if(millis() - g_state.lastRxMs > 1000){ g_state.valid = false; }
  }
  // Log periódico de estado cada 5s
  static unsigned long lastPrint=0; if(millis()-lastPrint>5000){ lastPrint=millis();
    LOGI("ESP","Status packets=%lu lastAge=%lums modo=%u valid=%d vL=%d vR=%d", (unsigned long)g_state.totalPackets, (unsigned long)(millis()-g_state.lastRxMs), g_state.modo, (int)g_state.valid, (int)g_state.manualVelL, (int)g_state.manualVelR);
  }
}

// Registrar el módulo (solo una vez aquí)
REGISTER_MODULE(Module_EspNow);
