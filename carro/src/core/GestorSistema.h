/**
 * @file GestorSistema.h
 * @brief Gestor central que orquesta el ciclo de vida de los módulos registrados.
 */
#pragma once
#include <Arduino.h>
#include <vector>
#include "Modulo.h"

namespace core {

/** Gestor central: registra módulos y coordina su ciclo de vida (iniciar/loop/detener). */
class GestorSistema {
public:
    static GestorSistema& instancia();

    void registrar(Modulo* m);
    bool iniciar();
    void loop();
    void detener();

    unsigned long tiempoArranque() const { return t_arranque_ms; }

private:
    GestorSistema() = default;
    std::vector<Modulo*> modulos;
    bool iniciado = false;
    unsigned long t_arranque_ms = 0;
};

} // namespace core
