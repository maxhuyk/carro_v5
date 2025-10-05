#include "GestorSistema.h"

namespace core {

GestorSistema& GestorSistema::instancia() {
    static GestorSistema g; return g;
}

void GestorSistema::registrar(Modulo* m) {
    modulos.push_back(m);
}

bool GestorSistema::iniciar() {
    if (iniciado) return true;
    t_arranque_ms = millis();
    for (auto* m : modulos) {
        if (m && !m->iniciar()) {
            return false; // aborta si un módulo falla
        }
    }
    iniciado = true;
    return true;
}

void GestorSistema::loop() {
    for (auto* m : modulos) {
        if (m) m->actualizar();
    }
}

void GestorSistema::detener() {
    for (auto it = modulos.rbegin(); it != modulos.rend(); ++it) {
        if (*it) (*it)->detener();
    }
    iniciado = false;
}

} // namespace core
