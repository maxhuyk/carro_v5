#include "EspNowReceiver.h"
#include <esp_wifi.h>
#include "monitoreo/LogMacros.h"

namespace comms {
using monitoreo::Logger;
using config::LogLevel;

// Estructura empaquetada versión 2 como en CarroClean
#pragma pack(push,1)
struct EspNowDataV2 {
    uint8_t  version;           // debe ser 2
    uint16_t batteryVoltage_mV; // mV
    uint8_t  modo;              // modo 0..4
    uint16_t ax_raw;            // 0..4095
    uint16_t ay_raw;            // 0..4095
    uint32_t timestamp;         // millis() emisor
};
#pragma pack(pop)

EspNowReceiver* EspNowReceiver::instancia_activa_ = nullptr;

bool EspNowReceiver::iniciar() {
    instancia_activa_ = this;
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        LOG_ERROR("ESPNOW", "Fallo init esp_now");
        return false;
    }
    // (Opcional) Configurar canal fijo: omitido si símbolo no disponible en entorno Arduino genérico.
    esp_now_register_recv_cb(EspNowReceiver::onDataRecv);
    LOG_INFO("ESPNOW", "Inicializado canal=%u MAC=%s", canal_, WiFi.macAddress().c_str());
    return true;
}

void EspNowReceiver::actualizar() {
    if (millis() - last_log_ > 5000) {
    LOG_DEBUG("ESPNOW", "Paquetes=%lu", (unsigned long)paquetes_);
        last_log_ = millis();
    }
}

void EspNowReceiver::onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (!instancia_activa_) return;
    instancia_activa_->handlePacket(incomingData, len);
}

void EspNowReceiver::handlePacket(const uint8_t* data, int len) {
    if (len != sizeof(EspNowDataV2)) {
    LOG_WARN("ESPNOW", "Tam inesperado=%d", len);
        return;
    }
    EspNowDataV2 pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.version != 2) {
    LOG_WARN("ESPNOW", "Version desconocida=%u", pkt.version);
        return;
    }
    paquetes_++;
    ManualCommand cmd{ millis(), pkt.modo, pkt.ax_raw, pkt.ay_raw, pkt.batteryVoltage_mV };
    if (callback_) callback_(cmd);
}

} // namespace comms
