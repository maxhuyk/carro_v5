/**
 * @file UWBCore.h
 * @brief Módulo de ranging UWB multi-anchor (3 anclas) ejecutado en una tarea dedicada.
 *
 * Replica la lógica de CarroClean: realiza ciclos de medición secuenciales sobre 3 anclas
 * DW3000 y produce distancias (cm) + timestamp. Implementa contadores y mecanismos de
 * recuperación (soft/hard reset) para cada ancla según estados de timeout o errores.
 * El resultado se entrega al `FollowController` para el pipeline de filtrado → trilateración.
 */
#pragma once
#include <Arduino.h>
#include "core/Modulo.h"
#include "control/autopilot/FollowController.h"
#include "DW3000.h"

namespace uwb {

/**
 * @brief Datos crudos de UWB producidos por la tarea (distancias en cm).
 * @note Las banderas v1..v3 indican validez de cada distancia en el ciclo actual.
 */
struct UWBRawData {
    float d1; float d2; float d3; // cm (como CarroClean)
    bool v1; bool v2; bool v3;
    unsigned long timestamp; // millis
};

/** @brief Contadores globales de runtime usados para diagnóstico. */
struct UWBCoreRuntimeCounters {
    uint32_t measurement_count = 0;
};

/**
 * @brief Configuración básica expuesta (delays y timeouts) manteniendo fidelidad.
 */
struct UWBCoreConfig {
    uint16_t rx_timeout_ms = 30;
    uint16_t inter_anchor_delay_us = 1000; // CarroClean: 1000us
    uint16_t cycle_delay_ms = 1;           // CarroClean: 1ms
};

/**
 * Tarea de ranging UWB (multi-anchor) que corre en Core0 y entrega
 * mediciones mm al FollowController. Implementa recuperación soft/hard.
 */
class UWBCore : public core::Modulo {
public:
    UWBCore(const UWBCoreConfig& cfg, control::FollowController& follow)
        : cfg_(cfg), follow_(follow) {}

    /**
     * @brief Inicializa pines, crea semáforo y lanza la tarea en Core0.
     * @return true si la tarea y recursos se crearon correctamente.
     */
    bool iniciar() override;
    /**
     * @brief Revisa si la tarea produjo un nuevo paquete válido y lo pasa al FollowController.
     */
    void actualizar() override; // toma lectura lista y la pasa al FollowController
    void detener() override {}

private:
    UWBCoreConfig cfg_;
    control::FollowController& follow_;

    // Raw data compartida (simulando el esquema multi-core de CarroClean)
    volatile UWBRawData raw_{NAN,NAN,NAN,false,false,false,0};
    volatile bool data_ready_ = false;
    volatile uint32_t measurement_count_ = 0;
    SemaphoreHandle_t sem_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;

    // Anchor DW3000 instances (pines idénticos)
    DW3000Class a1_{0,4,35};
    DW3000Class a2_{1,5,27};
    DW3000Class a3_{2,6,13};

    /**
     * @brief Estado y contadores de salud de un ancla.
     */
    struct AnchorState {
        uint16_t consecNotIdle=0;       ///< Veces consecutivas sin estado IDLE
        uint16_t consecTimeouts=0;      ///< Timeouts consecutivos
        uint16_t consecErrors=0;        ///< Errores consecutivos de frame
        unsigned long lastSuccessMs=0;  ///< Timestamp última medición válida
        uint8_t softResets=0;           ///< Re-inicializaciones suaves
        uint8_t hardResets=0;           ///< Re-inicializaciones completas
    };
    AnchorState s1_, s2_, s3_;

    // Task static trampolines
    static void taskEntry(void* self);
    void taskLoop();

    // Internos
    /** Ejecuta un ciclo double-sided ranging (simplificado). */
    float dsr_once(DW3000Class& D, uint16_t& failCount, unsigned long rxTimeoutMs);
    /** Reinicialización parcial manteniendo pines y SPI. */
    void anchor_soft_reinit(DW3000Class& D);
    /** Lógica de recuperación (soft/hard) según contadores y tiempos. */
    void try_anchor_recovery(DW3000Class& D, AnchorState& S, const char* name);
};

} // namespace uwb
