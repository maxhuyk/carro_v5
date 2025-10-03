#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "core/Modulo.h"
#include "monitoreo/LoggerReinicios.h"

namespace comms {

struct ManualCommand {
    uint32_t timestamp_ms;
    uint8_t modo;
    uint16_t ax_raw;
    uint16_t ay_raw;
    uint16_t battery_mV;
};

using ManualCallback = std::function<void(const ManualCommand&)>;

class EspNowReceiver : public core::Modulo {
public:
    explicit EspNowReceiver(uint8_t canal_wifi = 1) : canal_(canal_wifi) {}

    bool iniciar() override;
    void actualizar() override;
    void detener() override {}

    void setCallback(ManualCallback cb) { callback_ = cb; }

private:
    static void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);
    void handlePacket(const uint8_t* data, int len);

    uint8_t canal_;
    ManualCallback callback_{};
    static EspNowReceiver* instancia_activa_;
    uint32_t paquetes_ = 0;
    unsigned long last_log_ = 0;
};

} // namespace comms
