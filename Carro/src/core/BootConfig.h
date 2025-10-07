#pragma once
#include <Arduino.h>

// Encapsula la lógica de configuración de arranque
namespace BootConfig {
  void applyDefaults();      // setea claves por defecto en ConfigStore
  void applyModuleEnables(); // habilita/deshabilita módulos según claves enable.*
  void applyRuntime();       // aplica configuraciones dinámicas (nivel de log, etc.)
  inline void applyAll(){ applyDefaults(); applyModuleEnables(); applyRuntime(); }
}
