#include "config.h"
#include "calculos.h"
#include "MotorController.h"
#include "control.h"
#include <Arduino.h>
#include <cmath>

// Variables estáticas para mantener estado entre llamadas
static bool control_initialized = false;
static PIDController pid;
static PIDController pid_distancia;
static float distancias_previas[3] = {0, 0, 0};
static bool distancias_previas_valid = false;
static float angulo_anterior = 0;
static float y_anterior = 0.0;
static float velocidad_actual = 0;
static float correccion_anterior = 0; // Para suavizado de correcciones PID
static int ciclo = 0;

// Variables para sistema de seguridad por pérdida de señal
static bool signal_lost = false;
static unsigned long signal_lost_time = 0;
static float last_valid_distance = 0;
static float last_valid_correction = 0;
static int last_valid_velocity = 0;
static bool was_moving_when_lost = false;
// Variables de recuperación rápida
static bool in_recovery = false;
static unsigned long recovery_start_ms = 0;
static int recovery_cycles = 0;
static float saved_normal_Q = KALMAN_Q; // Para restaurar Q tras recuperación

// Variables para debug de frecuencia
static unsigned long control_execution_count = 0;
static unsigned long last_frequency_check = 0;

uint8_t modo = 0;            // modo actual del TAG
static int manual_vel_L = 0; // velocidad manual izquierda (-/+)
static int manual_vel_R = 0; // velocidad manual derecha (-/+)

// Función de inicialización del sistema de control
void control_init() {
    // Inicializar sistema de control de motores
    setupMotorControlDirect();
    
    // Inicializar filtros
    media_movil_init(3, MEDIA_MOVIL_VENTANA);
    kalman_init(3, KALMAN_Q, KALMAN_R);
    
    // Inicializar PIDs
    pid_init(&pid, 1.2, 0.01, 0.3, 0.0, PID_ALPHA, PID_SALIDA_MAX, INTEGRAL_MAX);
    pid_init(&pid_distancia, PID_DISTANCIA_KP, PID_DISTANCIA_KI, PID_DISTANCIA_KD, 
             DISTANCIA_OBJETIVO, PID_DISTANCIA_ALPHA, 
             VELOCIDAD_MAXIMA, PID_DISTANCIA_INTEGRAL_MAX);
    
    // Reset variables de estado
    distancias_previas_valid = false;
    angulo_anterior = 0;
    y_anterior = 0.0;
    velocidad_actual = 0;
    correccion_anterior = 0; // Reset corrección PID anterior
    ciclo = 0;
    
    control_initialized = true;
    Serial.println("[CONTROL] Sistema de control inicializado");
}

// Procesa UNA iteración del control - debe ser llamada desde loop() del main.cpp
void control_main(CarroData* data, PWMCallback enviar_pwm, StopCallback detener) {
    // Contar ejecuciones para debug de frecuencia
    control_execution_count++;
    unsigned long current_time = millis();
    if (current_time - last_frequency_check >= 5000) { // Cada 5 segundos
        float frequency = control_execution_count / 5.0;
        Serial.printf("[CONTROL_FREQ] Ejecutándose a %.1f Hz\n", frequency);
        control_execution_count = 0;
        last_frequency_check = current_time;
    }
    
    if (!control_initialized) {
        Serial.println("[CONTROL] ERROR: Debe llamar control_init() primero");
        return;
    }
    
    // ###########################################################
    // PASO 0: Verificar MODO del TAG - CONTROL BASADO EN MODO
    // ###########################################################
    if (data != nullptr && data->data_valid) {
        modo = (uint8_t)data->control_data[1];
        manual_vel_L = (int)data->buttons_data[2];
        manual_vel_R = (int)data->buttons_data[3];
        Serial.printf("[MODO] Modo actual del TAG: %d\n", modo);
    }
        
        // ###########################################################
        // PASO 1: Verificar datos válidos y sistema de seguridad
        // ###########################################################
        if (data != nullptr && data->data_valid) {
            //Serial.println("Datos completos recibidos");
            float* distancias = data->distancias;
            
            // SISTEMA DE SEGURIDAD: Verificar si hay valores NaN o señales perdidas
            bool signal_valid = true;
            for(int i = 0; i < 3; i++) {
                if(isnan(distancias[i]) || distancias[i] <= 0 || distancias[i] > 40000) {
                    signal_valid = false;
                    Serial.printf("[SIGNAL_LOST] Sensor %d: %.1f (NaN o fuera de rango)\n", i, distancias[i]);
                    break;
                }
            }
            
            if (!signal_valid) {
                // SEÑAL PERDIDA - Activar sistema de seguridad
                if (!signal_lost) {
                    // Primera vez que se pierde la señal
                    signal_lost = true;
                    signal_lost_time = millis();
                    last_valid_distance = (distancias_previas_valid) ? 
                        (distancias_previas[0] + distancias_previas[1]) / 2.0 : DISTANCIA_FALLBACK + 1000;
                    was_moving_when_lost = (velocidad_actual > 0);
                    
                    Serial.printf("[SIGNAL_LOST] Señal perdida. Última distancia: %.1f mm\n", last_valid_distance);
                }
                
                // Determinar comportamiento según distancia
                if (last_valid_distance < DISTANCIA_FALLBACK) {
                    // CERCA: Detenerse inmediatamente
                    Serial.println("[SAFETY] Cerca del objetivo - DETENIENDO INMEDIATAMENTE");
                    motor_enviar_pwm(0, 0);
                    velocidad_actual = 0;
                    return;
                } else {
                    // LEJOS: Continuar en la misma dirección por TIMEOUT_FALLBACK segundos
                    unsigned long time_since_lost = millis() - signal_lost_time;
                    if (time_since_lost < (TIMEOUT_FALLBACK * 1000) && was_moving_when_lost) {
                        // Continuar con última corrección y velocidad reducida
                        Serial.printf("[SAFETY] Lejos - Continuando %.1fs más (%.1f/%.1f)\n", 
                                     (TIMEOUT_FALLBACK * 1000 - time_since_lost) / 1000.0,
                                     time_since_lost / 1000.0, TIMEOUT_FALLBACK);
                        
                        // Aplicar control de emergencia con valores guardados
                        if (VELOCIDAD_FALLBACK > 0.0) {
                            float vel_izq, vel_der, giro_normalizado;
                            auto velocidades_result = calcular_velocidades_diferenciales(
                                VELOCIDAD_FALLBACK,
                                last_valid_correction,
                                VELOCIDAD_MAXIMA
                            );
                            vel_izq = velocidades_result.vel_izq;
                            vel_der = velocidades_result.vel_der;
                            
                            Serial.printf("[EMERGENCY] Velocidades: L=%d, R=%d\n", (int)vel_izq, (int)vel_der);
                            // Gating por modo (faltaba en esta rama)
                            if (modo == 1) {
                                enviar_pwm(vel_izq, vel_der);
                            } else if (modo == 3) {
                                int vL = constrain(manual_vel_L, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                                int vR = constrain(manual_vel_R, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                                Serial.printf("[MANUAL] (emergencia) L=%d R=%d\n", vL, vR);
                                enviar_pwm(vL, vR);
                            } else {
                                detener();
                            }
                        } else {
                            detener();
                        }
                        return;
                    } else {
                        // Timeout alcanzado - Detenerse
                        Serial.println("[SAFETY] Timeout alcanzado sin recuperar señal - DETENIENDO");
                        motor_enviar_pwm(0, 0);
                        velocidad_actual = 0;
                        return;
                    }
                }
            } else {
                // SEÑAL VÁLIDA - Restablecer sistema de seguridad si estaba activo
                if (signal_lost) {
                    Serial.println("[SIGNAL_RECOVERED] Señal recuperada - Reanudando control normal");
                    signal_lost = false;
                    // ===== Inicio de secuencia de recuperación rápida =====
                    // 1. Tomar primera medición válida y replicarla
                    float primera_medicion[3];
                    for (int i = 0; i < 3; i++) {
                        // Si la medición actual no es válida, usar distancias_previas
                        primera_medicion[i] = distancias[i];
                    }
                    // Sembrar buffers de media móvil copiando el valor en toda la ventana
                    // (Reiniciamos media y Kalman)
                    media_movil_init(3, MEDIA_MOVIL_VENTANA);
                    // Rellenar manualmente ejecutando tantas veces como ventana para fijar estado
                    float dummy[3];
                    for (int r = 0; r < MEDIA_MOVIL_VENTANA; r++) {
                        media_movil_actualizar(primera_medicion, dummy);
                    }
                    // 2. Kalman: estado = medición, P grande, Q temporal alta
                    saved_normal_Q = kalman_get_Q();
                    kalman_set_Q(RECOVERY_Q_TEMP);
                    kalman_force_state(primera_medicion, RECOVERY_P_INIT);
                    // 3. Bandera de recuperación para saltar/relajar limitador
                    in_recovery = true;
                    recovery_start_ms = millis();
                    recovery_cycles = 0;
                    Serial.printf("[RECOVERY] Iniciada. Q=%.3f -> %.3f, P=%.1f, ventana=%d\n", 
                                  saved_normal_Q, RECOVERY_Q_TEMP, (double)RECOVERY_P_INIT, MEDIA_MOVIL_VENTANA);
                }
            }
            
            // DEBUG: Verificar si las distancias son válidas (check adicional)
            bool distancias_validas = true;
            for(int i = 0; i < 3; i++) {
                if(distancias[i] <= 0 || distancias[i] > 40000) { // Fuera de rango razonable
                    distancias_validas = false;
                    break;
                }
            }
            
            if(!distancias_validas) {
                Serial.printf("[CONTROL_ERROR] Distancias inválidas: %.1f,%.1f,%.1f - Deteniendo\n", 
                              distancias[0], distancias[1], distancias[2]);
                motor_enviar_pwm(0, 0);
                return;
            }
            
            // Paso 1.5: Limito variación entre ciclos (anti-salto)
            if (distancias_previas_valid) {
                bool saltar_limitador = false;
                if (in_recovery) {
                    unsigned long elapsed = millis() - recovery_start_ms;
                    if (elapsed < RECOVERY_GRACE_MS) {
                        saltar_limitador = true; // Permitimos salto inicial para converger rápido
                    } else {
                        // Después de la gracia, aplicar limitador ampliado algunos ciclos
                        saltar_limitador = false;
                    }
                }
                if (!saltar_limitador) {
                    float distancias_limitadas[3];
                    float delta = (in_recovery ? RECOVERY_DELTA_POS : MAX_DELTA_POS);
                    limitar_variacion(distancias, distancias_previas, distancias_limitadas, 3, delta);
                    for(int i = 0; i < 3; i++) {
                        distancias[i] = distancias_limitadas[i];
                    }
                }
            }

            // Actualizo distancias_previas para el próximo ciclo
            for(int i = 0; i < 3; i++) {
                distancias_previas[i] = distancias[i]; // distancias_previas = distancias.copy()
            }
            distancias_previas_valid = true;
            //Serial.printf("Las distancias ORIGINALES en mm son: %f,%f,%f\n", distancias[0], distancias[1], distancias[2]);
            
            // ###########################################################
            // PASO 2: FILTRO DE MEDIA MOVIL
            // ###########################################################
            //distancias_media = media_movil.actualizar(distancias_previas)
            
            float distancias_media[3];
            media_movil_actualizar(distancias, distancias_media);
            //Serial.printf("Las distancias POR MEDIA en mm son: %f,%f,%f\n", distancias_media[0], distancias_media[1], distancias_media[2]);
            
            // ###########################################################
            // PASO 3: FILTRO KALMAN
            // ###########################################################
            float distancias_kalman[3];
            kalman_filtrar(distancias_media, distancias_kalman);
            Serial.printf("Las distancias POR KALMAN en mm son: %.1f,%.1f,%.1f\n", distancias_kalman[0], distancias_kalman[1], distancias_kalman[2]);

            if (in_recovery) {
                recovery_cycles++;
                // Condición para finalizar recuperación: suficientes ciclos y pasó la gracia
                if (recovery_cycles >= RECOVERY_K_CYCLES && (millis() - recovery_start_ms) > RECOVERY_GRACE_MS) {
                    // Restaurar Q normal
                    kalman_set_Q(saved_normal_Q);
                    in_recovery = false;
                    Serial.printf("[RECOVERY_DONE] Q restaurada a %.4f tras %d ciclos (%lums)\n", saved_normal_Q, recovery_cycles, millis()-recovery_start_ms);
                }
            }
            
            
            // ###########################################################
            // PASO 4: Mediante trilateración obtengo la ubicación del tag
            // ###########################################################
            
            float pos_tag3d_kalman[3];
            trilateracion_3d(SENSOR_POSICIONES, distancias_kalman, pos_tag3d_kalman);

            
            // ###########################################################
            // PASO 5: Calculo del ángulo a partir de trilateración
            // -180° hacia la izquierda y 180° hacia la derecha
            // ###########################################################
            float angulo_actual = angulo_direccion_xy(pos_tag3d_kalman);
    
            float angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual); // evita el salto entre -179 y 179
            float angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, MAX_DELTA);  // Limita los cambios bruscos
            angulo_anterior = angulo_relativo;  // Actualizamos para la próxima iteración
            Serial.printf("ÁNGULO : %.2f\n", angulo_relativo);

            // ###########################################################
            // PASO 6: CALCULAR DISTANCIA AL TAG PARA USO MÚLTIPLE
            // ###########################################################
            // Usar promedio de sensores frontales (1 y 2) para mejor precisión
            float distancia_al_tag = (distancias_kalman[0] + distancias_kalman[1]) / 2.0;
            
            // ###########################################################
            // PASO 7: UMBRAL DINÁMICO BASADO EN DISTANCIA
            // ###########################################################
            // Calcular umbral dinámico: cerca = más tolerante, lejos = más estricto
            float umbral_dinamico = calcular_umbral_dinamico(distancia_al_tag, 
                                                            UMBRAL_MINIMO, UMBRAL_MAXIMO,
                                                            DISTANCIA_UMBRAL_MIN, DISTANCIA_UMBRAL_MAX);
            
            float correccion;
            if (debe_corregir(angulo_relativo, umbral_dinamico)) {
                // ###########################################################
                // PASO 8: CONTROL PID ANGULAR
                // ###########################################################
                correccion = pid_update(&pid, angulo_relativo);
                Serial.printf("DIST:%.0fmm UMBRAL:%.1f° PID:%.2f\n", distancia_al_tag, umbral_dinamico, correccion);
            } else {
                correccion = 0;  // No se aplica corrección
                Serial.printf("DIST:%.0fmm UMBRAL:%.1f° PID:%.2f (ignorado)\n", distancia_al_tag, umbral_dinamico, correccion);
            }
            
            // ###########################################################
            // PASO 9: Control PID de velocidad por distancia - MEJORADO
            // ###########################################################
            // Control progresivo de velocidad para mantener distancia objetivo
            // (distancia_al_tag ya calculada arriba)
            
            // El PID calcula la velocidad basada en el error de distancia
            float velocidad_pid = pid_update(&pid_distancia, distancia_al_tag);
            
            // Error de distancia: positivo = lejos, negativo = cerca
            float error_distancia = distancia_al_tag - DISTANCIA_OBJETIVO;
            
            // Calcular velocidad objetivo basada en la distancia (SOLO AVANCE)
            int velocidad_objetivo;
            
            if (error_distancia > 5000) {
                // Muy lejos (>500mm): velocidad alta constante
                velocidad_objetivo = VELOCIDAD_MAXIMA;
            }
            else if (error_distancia > 100) {
                // Lejos (100-500mm): velocidad proporcional
                float velocidad_proporcional = 20 + (error_distancia - 100) * 0.016; // Base + proporcional
                velocidad_objetivo = constrain(velocidad_proporcional, 20, VELOCIDAD_MAXIMA);
            } 
            else if (error_distancia < 50) {
                // Está MUY cerca (>300mm más cerca del objetivo): parar completamente
                velocidad_objetivo = 0;
            }
            else {
                // En rango aceptable (-300mm a +100mm): velocidad muy baja para mantenimiento
                velocidad_objetivo = constrain(8 + error_distancia * 0.05, 0, 25); // Incrementado velocidades
            }
            
            // Aplicar suavizado de velocidad con aceleración y desaceleración separadas
            velocidad_actual = suavizar_velocidad_avanzada(velocidad_actual, velocidad_objetivo, 
                                                           ACELERACION_MAXIMA, DESACELERACION_MAXIMA);
            int velocidad_avance = (int)velocidad_actual;
            
            Serial.printf("Distancia al tag: %.1fmm, Objetivo: %.1fmm, Error: %.1fmm\n", 
                          distancia_al_tag, (float)DISTANCIA_OBJETIVO, error_distancia);
            Serial.printf("PID Distancia: %.2f -> Vel Objetivo: %d -> Vel Suavizada: %d\n", 
                          velocidad_pid, velocidad_objetivo, velocidad_avance);
            
            // ###########################################################
            // PASO 9  Control diferencial y pwm 
            // Un valor como: PWM:7,91 indica que la instrucción para 
            // el motor izquierdo es 7 y para el derecho es 91 con lo
            // que giraría hacia la izquierda para que el TAG regrese 
            // a ángulo cero
            // ###########################################################
            if (velocidad_avance > 0.0) {
                float vel_izq, vel_der, giro_normalizado;
                auto velocidades_result = calcular_velocidades_diferenciales(
                    velocidad_avance,  // v_lineal
                    correccion,        // angulo_relativo
                    VELOCIDAD_MAXIMA   // max_v
                );
                vel_izq = velocidades_result.vel_izq;
                vel_der = velocidades_result.vel_der;
                giro_normalizado = velocidades_result.giro_normalizado;

                // Las velocidades ya vienen como PWM de calcular_velocidades_diferenciales
                Serial.printf("[MOTOR_OUTPUT] Enviando velocidades: L=%d, R=%d (max_permitida=%d)\n", 
                              (int)vel_izq, (int)vel_der, VELOCIDAD_MAXIMA);
                //enviar_pwm(vel_izq, vel_der);
                if (modo == 1) {
                    enviar_pwm(vel_izq, vel_der);
                } else if (modo == 3) {
                    int vL = constrain(manual_vel_L, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                    int vR = constrain(manual_vel_R, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                    Serial.printf("[MANUAL] L=%d R=%d (override auto)\n", vL, vR);
                    enviar_pwm(vL, vR);
                } else {
                    detener();
                }
                // Guardar valores válidos para sistema de seguridad
                last_valid_distance = distancia_al_tag;
                last_valid_correction = correccion;
                last_valid_velocity = velocidad_avance;
            } else {
                if (modo == 3) {
                    int vL = constrain(manual_vel_L, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                    int vR = constrain(manual_vel_R, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                    Serial.printf("[MANUAL] (idle) L=%d R=%d\n", vL, vR);
                    enviar_pwm(vL, vR);
                } else {
                    detener();
                }
                
                // Guardar valores válidos para sistema de seguridad (velocidad = 0)
                last_valid_distance = distancia_al_tag;
                last_valid_correction = correccion;
                last_valid_velocity = 0;
            }
            
        } else {
            // NO HAY DATOS VÁLIDOS - También activar sistema de seguridad
            Serial.println("No hay datos válidos disponibles");
            
            if (!signal_lost) {
                // Primera vez que se pierden los datos
                signal_lost = true;
                signal_lost_time = millis();
                last_valid_distance = (distancias_previas_valid) ? 
                    (distancias_previas[0] + distancias_previas[1]) / 2.0 : DISTANCIA_FALLBACK + 1000;
                was_moving_when_lost = (velocidad_actual > 0);
                
                Serial.printf("[DATA_LOST] Datos perdidos. Última distancia: %.1f mm\n", last_valid_distance);
            }
            
            // Aplicar la misma lógica de seguridad
            if (last_valid_distance < DISTANCIA_FALLBACK) {
                Serial.println("[SAFETY] Sin datos y cerca - DETENIENDO");
                motor_enviar_pwm(0, 0);
                velocidad_actual = 0;
            } else {
                unsigned long time_since_lost = millis() - signal_lost_time;
                if (time_since_lost < (TIMEOUT_FALLBACK * 1000) && was_moving_when_lost) {
                    Serial.printf("[SAFETY] Sin datos pero lejos - Continuando %.1fs más\n", 
                                 (TIMEOUT_FALLBACK * 1000 - time_since_lost) / 1000.0);
                    
                    if (VELOCIDAD_FALLBACK > 0.0) {
                        float vel_izq, vel_der;
                        auto velocidades_result = calcular_velocidades_diferenciales(
                            VELOCIDAD_FALLBACK,
                            last_valid_correction,
                            VELOCIDAD_MAXIMA
                        );
                        vel_izq = velocidades_result.vel_izq;
                        vel_der = velocidades_result.vel_der;
                       if (modo == 1) {
                            enviar_pwm(vel_izq, vel_der);
                        } else if(modo == 3) {
                            int vL = constrain(manual_vel_L, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                            int vR = constrain(manual_vel_R, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
                            Serial.printf("[MANUAL] (lost continue) L=%d R=%d\n", vL, vR);
                            enviar_pwm(vL, vR);
                        } else {
                            detener();
                        }
                    } else {
                        detener();
                    }
                } else {
                    Serial.println("[SAFETY] Timeout sin datos - DETENIENDO");
                    motor_enviar_pwm(0, 0);
                    velocidad_actual = 0;
                }
            }
        }
}

    
