#include "config.h"
#include "calculos.h"
#include "MotorController.h"
#include "control.h"
#include <Arduino.h>

// Variables estáticas para mantener estado entre llamadas
static bool control_initialized = false;
static PIDController pid;
static PIDController pid_distancia;
static float distancias_previas[3] = {0, 0, 0};
static bool distancias_previas_valid = false;
static float angulo_anterior = 0;
static float y_anterior = 0.0;
static float velocidad_actual = 0;
static int ciclo = 0;

// Variables para debug de frecuencia
static unsigned long control_execution_count = 0;
static unsigned long last_frequency_check = 0;

// Función para procesar control por joystick (MODO 3)
void processJoystickControl(uint8_t joystick_value) {
    const int velocidad_manual = VELOCIDAD_MANUAL; // Usar valor del config.h
    
    int vel_izq = 0;
    int vel_der = 0;
    
    // Mapeo basado en el control.py de la Raspberry (valores 0-7)
    switch (joystick_value) {
        case 0: // (0000) - Sin movimiento
            vel_izq = 0;
            vel_der = 0;
            break;
            
        case 1: // (0100) - Solo adelante
            vel_izq = velocidad_manual;
            vel_der = velocidad_manual;
            break;
            
        case 2: // (0110) - Adelante + Izquierda
            vel_izq = velocidad_manual / 2;  // Rueda izquierda más lenta
            vel_der = velocidad_manual;      // Rueda derecha más rápida
            break;
            
        case 3: // (0010) - Solo izquierda (giro en el lugar)
            vel_izq = -velocidad_manual / 2; // Rueda izquierda atrás
            vel_der = velocidad_manual / 2;  // Rueda derecha adelante
            break;
            
        case 4: // (0011) - Izquierda + Atrás
            vel_izq = -velocidad_manual / 2; // Rueda izquierda reversa lenta
            vel_der = -velocidad_manual;     // Rueda derecha reversa rápida
            break;
            
        case 5: // (0001) - Solo atrás
            vel_izq = -velocidad_manual;
            vel_der = -velocidad_manual;
            break;
            
        case 6: // (1001) - Derecha + Atrás
            vel_izq = -velocidad_manual;     // Rueda izquierda reversa rápida
            vel_der = -velocidad_manual / 2; // Rueda derecha reversa lenta
            break;
            
        case 7: // (1000) - Solo derecha (giro en el lugar)
            vel_izq = velocidad_manual / 2;  // Rueda izquierda adelante
            vel_der = -velocidad_manual / 2; // Rueda derecha atrás
            break;
            
        default:
            Serial.printf("[JOYSTICK] Valor inválido: %d, deteniendo motores\n", joystick_value);
            vel_izq = 0;
            vel_der = 0;
            break;
    }
    
    // Para valores negativos, necesitamos manejar la dirección
    // motor_enviar_pwm espera valores 0-100, así que convertimos valores negativos
    int pwm_izq, pwm_der;
    
    if (vel_izq >= 0) {
        pwm_izq = vel_izq;
    } else {
        // Para reversa, necesitamos usar valores negativos o una lógica especial
        // Por ahora, limitamos a solo adelante por seguridad
        pwm_izq = 0;
        Serial.println("[JOYSTICK] WARNING: Reversa deshabilitada por seguridad");
    }
    
    if (vel_der >= 0) {
        pwm_der = vel_der;
    } else {
        // Para reversa, necesitamos usar valores negativos o una lógica especial
        // Por ahora, limitamos a solo adelante por seguridad
        pwm_der = 0;
        Serial.println("[JOYSTICK] WARNING: Reversa deshabilitada por seguridad");
    }
    
    Serial.printf("[JOYSTICK] Joy=%d -> L=%d, R=%d\n", joystick_value, pwm_izq, pwm_der);
    
    // Enviar comando a los motores
    motor_enviar_pwm(pwm_izq, pwm_der);
}

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
        uint8_t modo = (uint8_t)data->control_data[1]; // MODO está en control_data[1]
        uint8_t joystick = (uint8_t)data->buttons_data[0]; // JOYSTICK está en buttons_data[0]
        
        Serial.printf("[MODE_CONTROL] Modo=%d, Joystick=%d\n", modo, joystick);
        
        switch (modo) {
            case 0: // APAGADO
                Serial.println("[MODE_CONTROL] MODO 0: APAGADO - Deteniendo motores");
                motor_enviar_pwm(0, 0);
                return; // Salir de la función, no procesar más control
                
            case 1: // SEGUIMIENTO (control normal)
                Serial.println("[MODE_CONTROL] MODO 1: SEGUIMIENTO - Ejecutando control normal");
                // Continuar con el control normal (no return aquí)
                break;
                
            case 2: // PAUSA
                Serial.println("[MODE_CONTROL] MODO 2: PAUSA - Deteniendo motores");
                motor_enviar_pwm(0, 0);
                return; // Salir de la función, no procesar más control
                
            case 3: // MANUAL (control por joystick)
                Serial.printf("[MODE_CONTROL] MODO 3: MANUAL - Control por joystick (valor=%d)\n", joystick);
                processJoystickControl(joystick);
                return; // Salir de la función, el joystick ya controló los motores
                
            default:
                Serial.printf("[MODE_CONTROL] MODO DESCONOCIDO: %d - Deteniendo motores por seguridad\n", modo);
                motor_enviar_pwm(0, 0);
                return; // Salir de la función por seguridad
        }
    }
        
        // ###########################################################
        // PASO 1: Verificar datos válidos (SOLO para modo 1 - SEGUIMIENTO)
        // ###########################################################
        if (data != nullptr && data->data_valid) {
            //Serial.println("Datos completos recibidos");
            float* distancias = data->distancias;
            
            // DEBUG: Verificar si las distancias son válidas
            bool distancias_validas = true;
            for(int i = 0; i < 3; i++) {
                if(distancias[i] <= 0 || distancias[i] > 10000) { // Fuera de rango razonable
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
                float distancias_limitadas[3];
                limitar_variacion(distancias, distancias_previas, distancias_limitadas, 3, MAX_DELTA);
                // Copiar resultado de vuelta a distancias
                for(int i = 0; i < 3; i++) {
                    distancias[i] = distancias_limitadas[i];
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
            
            // ###########################################################
            // MOSTRAR DATOS ADICIONALES DEL ESP32 (FORMATO CORRECTO)
            // ###########################################################
            // Obtener datos directamente de la estructura
            float* power_data = data->power_data;      // [BAT_C, ML_C, MR_C]
            float* imu_data = data->imu_data;          // [PITCH, ROLL, YAW, MOV, VEL, ACCEL_Z]
            float* tag_sensors = data->tag_sensors;    // [S1, S2, S3]
            float* control_data = data->control_data;  // [BAT_TAG, MODO]
            float* buttons_data = data->buttons_data;  // [JOY_D, JOY_A, JOY_I, JOY_T]
            
            Serial.printf(" CARRO: BAT_C:%.2fV | ML_C:%.2fA | MR_C:%.2fA\n", power_data[0], power_data[1], power_data[2]);
            Serial.printf(" IMU: Pitch:%.1f° | Roll:%.1f° | Yaw:%.1f° | MOV:%.0f | VEL:%.2f | ACCEL_Z:%.2f\n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
            Serial.printf(" TAG: S1:%.1f | S2:%.1f | S3:%.1f | BAT_TAG:%.2fV | MODO:%.0f\n", tag_sensors[0], tag_sensors[1], tag_sensors[2], control_data[0], control_data[1]);
            Serial.printf(" JOYSTICK: D:%.0f A:%.0f I:%.0f T:%.0f\n", buttons_data[0], buttons_data[1], buttons_data[2], buttons_data[3]);
            
            // ###########################################################
            // PASO 4: Mediante trilateración obtengo la ubicación del tag
            // ###########################################################
            float pos_tag3d[3];
            trilateracion_3d(SENSOR_POSICIONES, distancias, pos_tag3d);
            float pos_tag3d_media[3];
            trilateracion_3d(SENSOR_POSICIONES, distancias_media, pos_tag3d_media);
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
            // PASO 6: UMBRAL 
            // ###########################################################
            float correccion;
            if (debe_corregir(angulo_relativo, UMBRAL)) {
                // ###########################################################
                // PASO 7: CCNTROL PID 
                // ###########################################################
                correccion = pid_update(&pid, angulo_relativo);
                Serial.printf("PID %.2f\n", correccion);
            } else {
                //Serial.printf("Corrección ignorada: ángulo de %.2f° está dentro del umbral\n", angulo_relativo);
                correccion = 0;  // No se aplica corrección
                Serial.printf("PID %.2f\n", correccion);
            }
            
            // ###########################################################
            // PASO 8  Control PID de velocidad por distancia
            // ###########################################################
            // Control progresivo de velocidad para mantener distancia objetivo
            // Usar promedio de sensores frontales (1 y 2) para mejor precisión
            float distancia_al_tag = (distancias_kalman[0] + distancias_kalman[1]) / 2.0;
            
            // El PID calcula cuánto debe acelerar/desacelerar para alcanzar la distancia objetivo
            float velocidad_pid = pid_update(&pid_distancia, distancia_al_tag);
            
            // Lógica correcta del PID:
            // Si error > 0 (lejos): PID da salida NEGATIVA para acercarse
            // Si error < 0 (cerca): PID da salida POSITIVA para alejarse
            // Como queremos seguir al tag, invertimos la lógica:
            float error_distancia = distancia_al_tag - DISTANCIA_OBJETIVO;
            
            // Calcular velocidad objetivo basada en la distancia
            int velocidad_objetivo;
            if (abs(error_distancia) > 200) {  // Fuera del rango objetivo: mover
                if (error_distancia > 0) {  // Lejos: acelerar
                    velocidad_objetivo = constrain(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA); // int(np.clip(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA))
                } else {  // Muy cerca: retroceder (o parar si no quieres reversa)
                    velocidad_objetivo = 0;  // Cambia esto si quieres que retroceda
                }
            } else {  // En rango aceptable (±200mm): PARAR
                velocidad_objetivo = 0;
            }
            
            // Aplicar suavizado de velocidad SOLO para arranque y frenado
            velocidad_actual = suavizar_velocidad(velocidad_actual, velocidad_objetivo, ACELERACION_MAXIMA);
            int velocidad_avance = (int)velocidad_actual;
            
            Serial.printf("Distancia al tag: %.1fmm, Objetivo: %.1fmm\n", distancia_al_tag, (float)DISTANCIA_OBJETIVO);
            Serial.printf("PID Distancia: %.2f -> Vel Objetivo: %d -> Vel Suavizada: %d\n", velocidad_pid, velocidad_objetivo, velocidad_avance);
            
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
                enviar_pwm(vel_izq, vel_der);
            } else {
                detener();
            }
            
        } else {
            Serial.println("Aún no hay datos válidos");
        }
}

    
