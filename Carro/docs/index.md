# Carro – Documentación Técnica

Autor: **Máximo Huykman**  
Repositorio: `carro`

## Visión General
Plataforma embarcada para un carro seguidor basado en ESP32 + DW3000 (UWB) que permite:
- Ranging UWB multi‑anchor (3 anchors) con recuperación de fallos.
- Control de seguimiento (autopilot) con filtrado (media móvil + Kalman) y heurística de velocidad.
- Control manual diferencial vía ESP‑NOW.
- Seguridad (signal lost, fallback, recovery Kalman, deadzone motores, timeouts).
- Arquitectura modular (módulos registrables) con logging unificado y niveles.

## Arquitectura de Alto Nivel
```text
┌─────────────┐    UWB (Core0)       ┌────────────────────┐
│ Anchors DW  │ ───────────────────► │ UWBCore (tarea RT) │
└─────┬───────┘                      └───────┬────────────┘
      │ cm                                   │ mm (pushMeasurement)
      ▼                                      ▼
                                  ┌────────────────────┐
ESP-NOW  ───────►  EspNowReceiver │ FollowController    │ ──► DriverMotores ─► PWM
(control manual)                  │  (filtros, PID ang) │
                                  └─────────┬──────────┘
                                            │ logs
                                      Logger / SPIFFS
```

## Módulos Clave
| Módulo | Propósito | Archivo principal |
|--------|-----------|-------------------|
| core::GestorSistema | Ciclo de vida (iniciar/actualizar) | `core/GestorSistema.*` |
| monitoreo::Logger | Logging niveles + archivo | `monitoreo/LoggerReinicios.*` |
| uwb::UWBCore | Tarea ranging DW3000 multi‑anchor | `uwb/UWBCore.*` |
| control::FollowController | Autopilot (filtros, PID angular, safety) | `control/autopilot/FollowController.*` |
| control::ControlManual | Normaliza joystick y aplica diferencial | `control/manual/ControlManual.*` |
| motion::DriverMotores | Deadzone, mapping PWM, safety timeout | `motion/DriverMotores.*` |
| utils::* | Filtros (media, Kalman), trilateración, helpers | `utils/Filtros.h` |

## Flujo de Control (Autopilot)
1. `UWBCore` obtiene distancias (cm) y las publica.
2. `FollowController::pushMeasurement()` recibe (mm) y guarda timestamp.
3. En cada ciclo `FollowController::actualizar()`:
   - Valida distancias / limitador variación.
   - Media móvil + Kalman.
   - Trilateración -> posición -> ángulo relativo.
   - PID angular si supera umbral dinámico.
   - Heurística de velocidad por error de distancia.
   - Diferencial (sin invertir ruedas) y aplicación a motores (modo 1).
   - Safety: fallback si señal perdida.

## Modos de Operación
| Modo | Descripción |
|------|-------------|
| 0 | Stop / pausa (pipeline corre, no mueve) |
| 1 | Autopilot (follow activo) |
| 3 | Manual (usa joystick, follow no envía PWM) |

## Seguridad
- Timeout señal UWB (>1s): fallback velocidad reducida + última corrección hasta `timeout_fallback_s`.
- Distancia menor a `distancia_fallback`: stop inmediato.
- Recovery Kalman: `Q` temporal alta, `P_init` grande, reseed media.
- Motores: timeout de comandos → stop + enable LOW.

## Parámetros Relevantes (extracto)
| Parámetro | Valor defecto | Archivo |
|-----------|---------------|---------|
| PID angular (Kp,Ki,Kd) | 1.2,0.01,0.3 | `ConfigControl.h` |
| Distancia objetivo | 1500 mm | `ConfigControl.h` |
| Velocidad máxima | 75 | `ConfigControl.h` |
| Deadzone motores | 60% | `ConfigMotores.h` |
| Joystick deadzone | 200 (crudo) | `ConfigPerfil.h` |
| Recovery Q temp | 1.0 | `ConfigControl.h` |

## Estilo de Código
- Nombres en español para reflejar intención original.
- Comentarios concisos, preferencia por funciones puras en utilidades.
- Lógica crítica (safety/follow) sin macros ocultas.

## Autoría
Desarrollo y migración modular: **Máximo Huykman**.  
Optimización, refactor y documentación: soporte asistido (IA).  
Por favor referenciar este repositorio al reutilizar fragmentos.

## Próximas Mejoras Sugeridas
- Métricas runtime (frecuencia UWB y control) publicadas vía logging.
- Tests unitarios mínimos de trilateración y filtros.
- Integración continua para compilación y artefactos de firmware.

---
> Última actualización generada automáticamente.
