# Carro  Documentación Técnica

[Documentación (GitHub Pages)](https://maxhuyk.github.io/carro_v5/)  
![Docs](https://github.com/maxhuyk/carro_v5/actions/workflows/docs.yml/badge.svg)

## Visión General
Plataforma embarcada para un carro seguidor basado en ESP32 + DW3000 (UWB) que permite:
- Ranging UWB multianchor (3 anchors) con recuperación de fallos.
- Control de seguimiento (autopilot) con filtrado (media móvil + Kalman) y heurística de velocidad.
- Control manual diferencial vía ESPNOW.
- Seguridad (signal lost, fallback, recovery Kalman, deadzone motores, timeouts).
- Arquitectura modular (módulos registrables) con logging unificado y niveles.

## Arquitectura de Alto Nivel
`	ext
    UWB (Core0)       
 Anchors DW     UWBCore (tarea RT) 
                      
       cm                                    mm (pushMeasurement)
                                            
                                  
ESP-NOW    EspNowReceiver  FollowController      DriverMotores  PWM
(control manual)                    (filtros, PID ang) 
                                  
                                             logs
                                      Logger / SPIFFS
`

## Módulos Clave
| Módulo | Propósito | Archivo principal |
|--------|-----------|-------------------|
| core::GestorSistema | Ciclo de vida (iniciar/actualizar) | core/GestorSistema.* |
| monitoreo::Logger | Logging niveles + archivo | monitoreo/LoggerReinicios.* |
| uwb::UWBCore | Tarea ranging DW3000 multianchor | uwb/UWBCore.* |
| control::FollowController | Autopilot (filtros, PID angular, safety) | control/autopilot/FollowController.* |
| control::ControlManual | Normaliza joystick y aplica diferencial | control/manual/ControlManual.* |
| motion::DriverMotores | Deadzone, mapping PWM, safety timeout | motion/DriverMotores.* |
| utils::* | Filtros (media, Kalman), trilateración, helpers | utils/Filtros.h |

## Flujo de Control (Autopilot)
1. UWBCore obtiene distancias (cm) y las publica.
2. FollowController::pushMeasurement() recibe (mm) y guarda timestamp.
3. En cada ciclo FollowController::actualizar():
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
- Timeout señal UWB (>1s): fallback velocidad reducida + última corrección hasta 	imeout_fallback_s.
- Distancia menor a distancia_fallback: stop inmediato.
- Recovery Kalman: Q temporal alta, P_init grande, reseed media.
- Motores: timeout de comandos  stop + enable LOW.

## Parámetros Relevantes (extracto)
| Parámetro | Valor defecto | Archivo |
|-----------|---------------|---------|
| PID angular (Kp,Ki,Kd) | 1.2,0.01,0.3 | ConfigControl.h |
| Distancia objetivo | 1500 mm | ConfigControl.h |
| Velocidad máxima | 75 | ConfigControl.h |
| Deadzone motores | 60% | ConfigMotores.h |
| Joystick deadzone | 200 (crudo) | ConfigPerfil.h |
| Recovery Q temp | 1.0 | ConfigControl.h |

## Estilo de Código
- Comentarios concisos, preferencia por funciones puras en utilidades.
- Lógica crítica (safety/follow) sin macros ocultas.

---
> Última actualización generada automáticamente. Para ver la doc renderizada online usar el enlace de GitHub Pages.

(La documentación se genera desde carpeta carro/docs con Doxygen.)
