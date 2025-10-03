#include "EspNowReceiver.h"

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
    esp_wifi_set_channel(canal_, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        Logger::instancia().logf(LogLevel::ERROR, "ESPNOW", "Fallo init esp_now");
        return false;
    }
    esp_now_register_recv_cb(EspNowReceiver::onDataRecv);
    Logger::instancia().logf(LogLevel::INFO, "ESPNOW", "Inicializado canal=%u MAC=%s", canal_, WiFi.macAddress().c_str());
    return true;
}

void EspNowReceiver::actualizar() {
    if (millis() - last_log_ > 5000) {
        Logger::instancia().logf(LogLevel::DEBUG, "ESPNOW", "Paquetes=%lu", (unsigned long)paquetes_);
        last_log_ = millis();
    }
}

void EspNowReceiver::onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (!instancia_activa_) return;
    instancia_activa_->handlePacket(incomingData, len);
}

void EspNowReceiver::handlePacket(const uint8_t* data, int len) {
    if (len != sizeof(EspNowDataV2)) {
        Logger::instancia().logf(LogLevel::WARN, "ESPNOW", "Tam inesperado=%d", len);
        return;
    }
    EspNowDataV2 pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.version != 2) {
        Logger::instancia().logf(LogLevel::WARN, "ESPNOW", "Version desconocida=%u", pkt.version);
        return;
    }
    paquetes_++;
    ManualCommand cmd{ millis(), pkt.modo, pkt.ax_raw, pkt.ay_raw, pkt.batteryVoltage_mV };
    if (callback_) callback_(cmd);
}

} // namespace comms
