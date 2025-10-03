#pragma once

#include <Arduino.h>

namespace core {

// Clase base para todos los módulos del sistema. Permite un ciclo de vida homogéneo.
class Modulo {
public:
    virtual ~Modulo() = default;

    // Se invoca una sola vez durante el arranque del sistema.
    virtual bool iniciar() { return true; }

    // Se invoca en cada iteración del bucle principal.
    virtual void actualizar() {}

    // Se invoca antes de un apagado controlado o reinicio planificado.
    virtual void detener() {}
};

} // namespace core
