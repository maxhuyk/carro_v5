# Proyecto Carro (Arquitectura Modular)

## Objetivo

Este proyecto es la evolución modular del firmware del carro autónomo. El objetivo es separar responsabilidades para que cada subsistema (UWB, control, motores, comunicaciones, monitoreo) pueda habilitarse o deshabilitarse según la aplicación, reutilizando la misma base sin arrastrar dependencias innecesarias.

### Metas principales

- **Composición flexible:** poder compilar perfiles que sólo usen UWB, sólo motores o toda la pila completa.
- **Aislamiento de dependencia:** cada módulo expone contratos claros (structs, callbacks, interfaces) y depende sólo de capas que estén por debajo.
- **Observabilidad incorporada:** registrar reinicios y eventos críticos dentro de `monitoring` sin mezclar la lógica con los demás módulos.
- **Documentación viva:** mantener este documento sincronizado con los cambios para que sirva como guía del sistema.

## Mapa de módulos

La siguiente tabla resume los módulos planificados, su propósito y principales interfaces:

| Módulo | Namespace / Carpeta | Responsabilidad | Entradas | Salidas | Dependencias opcionales |
|--------|---------------------|-----------------|----------|---------|-------------------------|
| Núcleo | `core` | Gestión de ciclo de vida, registro de módulos, timing común | Lista de módulos, configuración general | Llamadas `begin/loop/end`, servicios compartidos | `monitoring` (para loggear eventos del núcleo) |
| Configuración | `config` | Cargar perfiles y parámetros (constantes, overrides) | Storage (NVS/LittleFS), defaults | Structs `CommsConfig`, `UwbConfig`, `ControlConfig`, `MotorConfig` | `monitoring` (para reportar cambios) |
| Monitoreo | `monitoring` | Logging persistente, métricas, reinicios | Reset reason, timestamps | Entradas en log persistente, estadísticas | `core::SystemManager` |
| Comunicaciones | `comms::espnow` | Recepción ESP-NOW y decodificación de comandos manuales | Configuración, callbacks de aplicación | `ManualCommand`, métricas | `monitoring` (registro de reconexiones) |
| UWB | `uwb` | Control DW3000, filtrado primario y estado de anchors | Configuración UWB | `Measurement` (distancias + estado) | `utils::math` |
| Control manual | `control::manual` | Convertir ejes analógicos/comandos en velocidades | `ManualCommand`, `ControlConfig` | `MotionCommand` | — |
| Control autónomo | `control::autopilot` | Seguimiento de TAG usando mediciones UWB | `Measurement`, modo actual, `ManualCommand` opcional | `MotionCommand` | `control::manual`, `monitoring` |
| Motores | `motion::motors` | PWM/enable/diagnóstico del DRV8701 | `MotorConfig`, `MotionCommand` | Señales PWM, `FaultEvent` | `monitoring` |
| Utilidades | `utils::math`, `utils::concurrency` | Filtros, trilateración, wrappers FreeRTOS | Datos primarios | Resultados matemáticos o sincronización | — |

## Flujo de datos

1. `core::SystemManager` inicializa `config`, `monitoring` y cada módulo registrado.
2. `comms::EspNowReceiver` entrega `ManualCommand` a quien se suscriba (por defecto `control::ManualController`).
3. `uwb::UWBCoreFacade` realiza mediciones y emite `Measurement` a `control::FollowController`.
4. `control::FollowController` fusiona datos (auto + manual) y genera un `MotionCommand`.
5. `motion::MotorDriver` aplica el comando, monitorea fallas y notifica a `monitoring` o `core` si detecta problemas.
6. `monitoring::RestartLogger` registra cada reinicio, tiempos de recuperación y eventos críticos (timeouts, fallos de anchor, etc.).

Cada bloque puede funcionar aislado: por ejemplo, para pruebas de motores se instancia sólo `core + config + motion`, y se alimenta con comandos manuales desde un stub. Para un logger de reinicios basta con `core + monitoring`.

## Contratos principales

### ManualCommand
```cpp
struct ManualCommand {
    uint32_t timestamp;
    uint8_t  modo;
    uint16_t ax_raw;
    uint16_t ay_raw;
    int16_t  vel_left_hint;  // opcional (preprocesado)
    int16_t  vel_right_hint; // opcional (preprocesado)
    uint16_t battery_mV;
};
```

### Measurement
```cpp
struct Measurement {
    float distances_mm[3];
    bool  anchor_ok[3];
    uint32_t count;
    uint32_t timestamp_ms;
};
```

### MotionCommand
```cpp
struct MotionCommand {
    int16_t left_pwm;   // -100..100 (escala lógica)
    int16_t right_pwm;  // -100..100
    enum class Source { Manual, Autopilot, Safety } provenance;
};
```

## Estrategia de migración

1. **Crear esqueleto** (esta etapa): carpetas, `SystemManager`, structs base y documentación.
2. **Extraer utilidades comunes** desde `CarroClean`: filtros, trilateración, configuraciones.
3. **Migrar módulos uno por uno**:
   - `utils` + `config`
   - `monitoring`
   - `comms::espnow`
   - `uwb`
   - `control` (manual → autopilot)
   - `motion`
4. **Composición en `main.cpp`** con perfiles de build:
   - Perfil `full` (todo)
   - Perfil `uwb_only`
   - Perfil `motor_test`
5. **Validación continua:** tras cada migración, compilar y (cuando sea posible) ejecutar pruebas unitarias o de integración.

## Próximos pasos inmediatos

- Subir el esqueleto básico (`core::SystemManager`, interfaces) a `src`.
- Preparar archivo `platformio.ini` apuntando al mismo board que `CarroClean` para mantener compatibilidad.
- Copiar la configuración base (`config`) y la lógica auxiliar (`utils`) desde `CarroClean`, asegurando que los namespaces coincidan con esta documentación.

Mantendremos este README actualizado conforme se vayan migrando módulos y cerrando dependencias.

## Generar documentación (Doxygen)

La documentación HTML se genera localmente usando únicamente el `Doxyfile` del directorio `docs`.

1. Instalar Doxygen (Windows): descargar instalador desde https://www.doxygen.nl y asegurarse de que `doxygen.exe` quede en el PATH.
2. (Opcional) Verificar versión: `doxygen -v`.
3. Desde la carpeta `Carro/docs` ejecutar:

```bash
doxygen Doxyfile
```

La salida se ubicará en `Carro/docs/build/html/index.html`.

### Hoja de estilo

Se incluye un tema ligero (`doxygen-awesome.css` reducido) que se aplica automáticamente mediante la directiva `HTML_EXTRA_STYLESHEET` en `Doxyfile`.

### Actualizar comentarios

Todos los headers en `src/` poseen ahora bloques `@file` y descripciones. Para ver advertencias de documentación incompleta puedes cambiar `WARN_IF_UNDOCUMENTED = YES` temporalmente en `docs/Doxyfile`.

### Limpieza

Para regenerar desde cero simplemente borrar `docs/build` antes de ejecutar Doxygen nuevamente.
