#ifndef CONTROL_NAVEGACION_H
#define CONTROL_NAVEGACION_H

#include <Arduino.h>
#include "ControlConfig.h"
#include "Filtros.h"
#include "PIDController.h"
#include "AlgoritmosNavegacion.h"

// Declaraciones de funciones de control de motores (del sistema existente)
extern void setMotorL(int pwm, bool dir);
extern void setMotorR(int pwm, bool dir);

class ControlNavegacion {
private:
    // Filtros para las distancias
    FiltroHibrido<MEDIA_MOVIL_VENTANA> filtros_distancia[3];
    
    // PIDs
    PIDController pid_angulo;
    PIDController pid_distancia;
    
    // Variables de estado
    float posicion_x, posicion_y;
    float posicion_x_anterior, posicion_y_anterior;
    float angulo_objetivo;
    float distancia_al_objetivo;
    
    // Variables de fallback
    unsigned long tiempo_inicio_fallback;
    bool en_fallback;
    float ultima_posicion_valida_x, ultima_posicion_valida_y;
    
    // Control de aceleración
    int velocidad_anterior_ml, velocidad_anterior_mr;
    
    // Modo de operación
    int modo;

public:
    ControlNavegacion() 
        : pid_angulo(PID_KP, PID_KI, PID_KD, PID_SETPOINT, PID_ALPHA, INTEGRAL_MAX, PID_SALIDA_MAX),
          pid_distancia(PID_DISTANCIA_KP, PID_DISTANCIA_KI, PID_DISTANCIA_KD, DISTANCIA_OBJETIVO, 
                       PID_DISTANCIA_ALPHA, PID_DISTANCIA_INTEGRAL_MAX, VELOCIDAD_MAXIMA),
          posicion_x(NAN), posicion_y(NAN), posicion_x_anterior(NAN), posicion_y_anterior(NAN),
          angulo_objetivo(0), distancia_al_objetivo(0), tiempo_inicio_fallback(0), en_fallback(false),
          ultima_posicion_valida_x(NAN), ultima_posicion_valida_y(NAN),
          velocidad_anterior_ml(0), velocidad_anterior_mr(0), modo(0) {
        
        // Inicializar filtros
        for(int i = 0; i < 3; i++) {
            filtros_distancia[i] = FiltroHibrido<MEDIA_MOVIL_VENTANA>(KALMAN_Q, KALMAN_R);
        }
    }

    void setModo(int nuevo_modo) {
        if (nuevo_modo >= 0 && nuevo_modo <= 4) {
            modo = nuevo_modo;
            pid_angulo.reset();
            pid_distancia.reset();
            en_fallback = false;
        }
    }

    // Procesar datos directos (sin estructuras complejas)
    void procesar(const float distancias_raw[3], float tilt_forward = 0, float tilt_turn = 0) {
        unsigned long tiempo_actual = millis();
        
        // Filtrar distancias
        float distancias_filtradas[3];
        for(int i = 0; i < 3; i++) {
            distancias_filtradas[i] = filtros_distancia[i].update(distancias_raw[i]);
        }
        
        // Procesar según el modo
        Serial.printf("[DEBUG] ControlNavegacion modo: %d\n", modo);
        switch(modo) {
            case 1: // Modo UWB
                Serial.println("[DEBUG] Procesando modo UWB");
                procesarModoUWB(distancias_filtradas, tiempo_actual);
                break;
            case 2: // Modo TILT
                procesarModoTilt(tilt_forward, tilt_turn);
                break;
            case 3: // Modo Manual
                procesarModoManual();
                break;
            default:
                Serial.printf("[DEBUG] Modo %d - deteniendo motores\n", modo);
                detenerMotores();
                break;
        }
    }

    // Nuevo método para procesamiento de seguimiento (modo 1) usando lógica Python
    void procesarSeguimiento(float dist1, float dist2, float dist3) {
        float distancias[3] = {dist1, dist2, dist3};
        unsigned long tiempo_actual = millis();
        
        // Filtrar distancias
        float distancias_filtradas[3];
        for(int i = 0; i < 3; i++) {
            distancias_filtradas[i] = filtros_distancia[i].update(distancias[i]);
        }
        
        // Llamar al método corregido de modo UWB
        procesarModoUWB(distancias_filtradas, tiempo_actual);
    }

    // Método ejecutarControlNormal ahora público para compatibilidad
    void ejecutarControlNormal() {
        // *** LÓGICA CORREGIDA SEGÚN PYTHON ejecutar_modo_seguimiento ***
        
        // 1. Calcular ángulo direccional (como angulo_direccion_xy en Python)
        angulo_objetivo = CalculadorAngulo::calcular(posicion_x, posicion_y, 0, 0);
        
        // 2. Calcular distancia al objetivo  
        distancia_al_objetivo = sqrt(posicion_x * posicion_x + posicion_y * posicion_y);
        
        Serial.printf("[UWB] Pos:(%.1f,%.1f), Ang:%.1f°, Dist:%.1f\n", 
                     posicion_x, posicion_y, angulo_objetivo, distancia_al_objetivo);
        
        // 3. PID de ángulo con umbral (como en Python debe_corregir)
        float correccion_giro = 0;
        if(abs(angulo_objetivo) > UMBRAL) {  // UMBRAL en Python = 10°
            correccion_giro = pid_angulo.update(angulo_objetivo);
        } else {
            correccion_giro = 0;  // No corregir si está dentro del umbral
        }
        
        // 4. PID de distancia (como en Python)
        float velocidad_pid = pid_distancia.update(distancia_al_objetivo);
        float error_distancia = distancia_al_objetivo - DISTANCIA_OBJETIVO;
        
        int velocidad_objetivo = 0;
        if(abs(error_distancia) > 200) {  // Umbral de 200mm como Python
            if(error_distancia > 0) {
                // Muy lejos, usar velocidad calculada por PID
                velocidad_objetivo = constrain(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA);
            } else {
                // Demasiado cerca, no moverse
                velocidad_objetivo = 0;
            }
        } else {
            // Dentro del rango aceptable, no moverse
            velocidad_objetivo = 0;
        }
        
        // 5. Aplicar suavizado de aceleración (como en Python)
        velocidad_anterior_ml = suavizarVelocidad(velocidad_anterior_ml, velocidad_objetivo, ACELERACION_MAXIMA);
        int velocidad_avance = velocidad_anterior_ml;
        
        Serial.printf("[UWB] Vel_PID:%.1f, Error_Dist:%.1f, Vel_Obj:%d, Vel_Final:%d, Correcc:%.1f\n",
                     velocidad_pid, error_distancia, velocidad_objetivo, velocidad_avance, correccion_giro);
        
        // 6. Calcular velocidades diferenciales (como calcular_velocidades_diferenciales en Python)
        if(velocidad_avance > 0) {
            int ml = velocidad_avance - (int)correccion_giro;
            int mr = velocidad_avance + (int)correccion_giro;
            
            // Limitar a rango válido
            ml = constrain(ml, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
            mr = constrain(mr, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
            
            Serial.printf("[UWB] Motores: ML=%d, MR=%d\n", ml, mr);
            setMotorSpeeds(ml, mr);
        } else {
            Serial.println("[UWB] Deteniendo motores (velocidad = 0)");
            setMotorSpeeds(0, 0);
        }
    }

    // Método activarFallback (público)
    void activarFallback(unsigned long tiempo_actual) {
        if(!en_fallback) {
            en_fallback = true;
            tiempo_inicio_fallback = tiempo_actual;
        }
        
        // Verificar timeout de fallback
        float tiempo_transcurrido = (tiempo_actual - tiempo_inicio_fallback) / 1000.0;
        if(tiempo_transcurrido > TIMEOUT_FALLBACK) {
            detenerMotores();
            return;
        }
        
        // Verificar si hay última posición válida
        if(isnan(ultima_posicion_valida_x) || isnan(ultima_posicion_valida_y)) {
            detenerMotores();
            return;
        }
        
        // Calcular distancia a la última posición válida
        float distancia_fallback = sqrt(ultima_posicion_valida_x * ultima_posicion_valida_x + 
                                       ultima_posicion_valida_y * ultima_posicion_valida_y);
        
        if(distancia_fallback < DISTANCIA_FALLBACK) {
            detenerMotores();
        } else {
            // Moverse hacia la última posición conocida con velocidad reducida
            float angulo_fallback = CalculadorAngulo::calcular(ultima_posicion_valida_x, ultima_posicion_valida_y, 0, 0);
            
            int ml = VELOCIDAD_FALLBACK;
            int mr = VELOCIDAD_FALLBACK;
            
            // Aplicar corrección básica de dirección
            if(abs(angulo_fallback) > 10) {
                if(angulo_fallback > 0) {
                    ml = VELOCIDAD_FALLBACK / 2;
                } else {
                    mr = VELOCIDAD_FALLBACK / 2;
                }
            }
            
            setMotorSpeeds(ml, mr);
        }
    }

private:
    // Función auxiliar para suavizado de aceleración (como suavizar_velocidad en Python)
    int suavizarVelocidad(int velocidad_actual, int velocidad_objetivo, int aceleracion_max) {
        int diferencia = velocidad_objetivo - velocidad_actual;
        if(abs(diferencia) <= aceleracion_max) {
            return velocidad_objetivo;
        } else {
            return velocidad_actual + (diferencia > 0 ? aceleracion_max : -aceleracion_max);
        }
    }

    // Control directo de motores
    void setMotorSpeeds(int ml, int mr) {
        // Limitar a rango válido
        ml = constrain(ml, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
        mr = constrain(mr, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
        
        // Aplicar límite de aceleración
        ml = limitarAceleracion(velocidad_anterior_ml, ml);
        mr = limitarAceleracion(velocidad_anterior_mr, mr);
        
        // Actualizar velocidades anteriores
        velocidad_anterior_ml = ml;
        velocidad_anterior_mr = mr;
        
        // Controlar motores con dirección correcta
        Serial.printf("[DEBUG] setMotorSpeeds - ml: %d, mr: %d\n", ml, mr);
        setMotorL(abs(ml), ml >= 0);
        setMotorR(abs(mr), mr >= 0);
    }
    
    int limitarAceleracion(int velocidad_anterior, int velocidad_objetivo) {
        int diferencia = velocidad_objetivo - velocidad_anterior;
        
        if(abs(diferencia) <= ACELERACION_MAXIMA) {
            return velocidad_objetivo;
        }
        
        return velocidad_anterior + (diferencia > 0 ? ACELERACION_MAXIMA : -ACELERACION_MAXIMA);
    }
    
    void detenerMotores() {
        setMotorSpeeds(0, 0);
    }

    void procesarModoUWB(const float distancias[3], unsigned long tiempo_actual) {
        // Contar sensores válidos
        int sensores_validos = ValidadorDatos::contarSensoresValidos(distancias);
        
        bool posicion_calculada = false;
        float nueva_x = NAN, nueva_y = NAN;
        
        if(sensores_validos >= 3) {
            // Trilateración
            if(Trilateracion::calcular(distancias, nueva_x, nueva_y)) {
                if(LimitadorDistancia::validar(nueva_x, nueva_y, posicion_x_anterior, posicion_y_anterior)) {
                    posicion_calculada = true;
                }
            }
        } else if(sensores_validos == 2) {
            // Bilateración
            int indices[2];
            float valores[2];
            int count;
            ValidadorDatos::obtenerSensoresValidos(distancias, indices, valores, count);
            
            if(count == 2) {
                if(Bilateracion::calcular(valores, indices, nueva_x, nueva_y, posicion_x, posicion_y)) {
                    if(LimitadorDistancia::validar(nueva_x, nueva_y, posicion_x_anterior, posicion_y_anterior)) {
                        posicion_calculada = true;
                    }
                }
            }
        }
        
        if(posicion_calculada) {
            // Actualizar posición
            posicion_x_anterior = posicion_x;
            posicion_y_anterior = posicion_y;
            posicion_x = nueva_x;
            posicion_y = nueva_y;
            
            // Guardar como última posición válida
            ultima_posicion_valida_x = nueva_x;
            ultima_posicion_valida_y = nueva_y;
            
            // Salir del modo fallback
            en_fallback = false;
            
            // Calcular control normal
            ejecutarControlNormal();
            
        } else {
            // No se pudo calcular posición - activar fallback
            activarFallback(tiempo_actual);
        }
    }
    
    void procesarModoTilt(float tilt_forward, float tilt_turn) {
        // Configurar ejes según la configuración
        float forward = TILT_FORWARD_AXIS_ROLL ? tilt_forward : tilt_turn;
        float turn = TILT_TURN_AXIS_PITCH ? tilt_turn : tilt_forward;
        
        // Aplicar signos de corrección
        forward *= TILT_FORWARD_SIGN;
        turn *= TILT_TURN_SIGN;
        
        // Convertir a velocidades de motor
        int vel_base = (int)(forward * 2.0); // Escalado para velocidad base
        int vel_turn = (int)(turn * 1.5);    // Escalado para giro
        
        int ml = constrain(vel_base - vel_turn, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
        int mr = constrain(vel_base + vel_turn, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
        
        setMotorSpeeds(ml, mr);
    }
    
    void procesarModoManual() {
        setMotorSpeeds(VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
    }

public:
    // Getters para debug/monitoreo
    float getPosicionX() { return posicion_x; }
    float getPosicionY() { return posicion_y; }
    float getAnguloObjetivo() { return angulo_objetivo; }
    float getDistanciaObjetivo() { return distancia_al_objetivo; }
    bool estaEnFallback() { return en_fallback; }
    int getModo() { return modo; }
};

#endif // CONTROL_NAVEGACION_H
