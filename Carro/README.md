# Proyecto Carro (Esqueleto Modular)

Este es un esqueleto limpio para reconstruir el sistema mediante módulos desacoplados.

## Filosofía
Cada módulo implementa 3 fases bien definidas:

1. `configure()`  -> Cargar/derivar parámetros (no tocar hardware).
2. `setup()`      -> Reservar recursos / inicializar hardware. Devuelve `bool`.
3. `loop()`       -> Lógica periódica rápida (sin bloqueos largos).

Características clave incluidas:
- Activación/desactivación por módulo (en memoria).
- Orden de inicialización determinista con soporte de dependencias.
- Métricas básicas (tiempo de setup y loop acumulado) para diagnóstico.
- Registro central de módulos (`ModuleManager`).
- Macro de registro para reducir repetición.

## Estructura
```
src/
  core/
    Module.h            (clase base)
    ModuleManager.h/.cpp
    ConfigStore.h/.cpp  (stub para futura carga persistente)
    Log.h               (logging mínimo)
  modules/
    Module_EspNow.*     (stub)
    Module_Motor.*      (stub)
    Module_Emergency.*  (stub)
    Module_Uwb.*        (placeholder)
    Module_Control.*    (stub control de alto nivel)
  main.cpp
```

## Flujo de arranque
1. Registrar todos los módulos.
2. Aplicar configuración (enable/disable).
3. `configureAll()`
4. `setupAll()` (respeta dependencias y marca fallos sin abortar el resto).
5. Bucle principal llama `loopAll()`.

## Dependencias
Al registrar un módulo se puede declarar la lista de nombres de los módulos de los cuales depende. Si alguno falla en `setup()`, el dependiente queda deshabilitado automáticamente.

## Extender con un módulo nuevo
```cpp
class Module_MiSensor : public Module {
public:
  Module_MiSensor(): Module("MiSensor", {"Motor"}) {}
  void configure() override { sampleIntervalMs_ = 50; }
  bool setup() override { lastMs_ = millis(); return true; }
  void loop() override {
    if (millis()-lastMs_ >= sampleIntervalMs_) { lastMs_=millis(); /* leer sensor */ }
  }
private:
  uint32_t lastMs_ = 0; uint32_t sampleIntervalMs_ = 50;
};
REGISTER_MODULE(Module_MiSensor);
```

## Sugerencias robustas
1. **No bloquear en loop()**: usar lapsos cortos y `millis()`.
2. **Instrumentar latencias**: revisar `ModuleManager::dumpStatus()` serial cada X segundos.
3. **Fallbacks**: si un módulo crítico falla en `setup()`, dejar constancia y seguir (no reset total).
4. **Config layering**: futuro: capas (defaults, archivo, OTA) → aplicar antes de `configureAll()`.
5. **Hot-disable**: puedes llamar `manager.disableModule("Uwb")` en runtime si se detecta error recurrente.
6. **Watchdog interno**: usar contador de ciclos sin eventos dentro de módulos para auto-recuperación.
7. **Separar ISR**: cualquier interrupción debe poner flags y el procesamiento pesado en loop().

## Próximos pasos recomendados
1. Implementar logger configurable (niveles + sink serial / archivo).
2. Añadir JSON simple (ArduinoJson) en `ConfigStore` para persistir.
3. Reintroducir lógica real de cada subsistema incrementalmente (motores, esp-now, emergencia, UWB).

---
Este esqueleto está listo para ampliarse sin arrastrar la complejidad previa.
