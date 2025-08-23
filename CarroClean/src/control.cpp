#include "config.h"
#include <Arduino.h>



// Loop principal inicial
void main() {

    UARTDataReceiver data_receiver(DATA_PORT, BAUDRATE);
    UARTMotorController motor_controller(DATA_PORT, BAUDRATE);
    PWMManager pwm_manager(motor_controller);
    MediaMovil media_movil(3, MEDIA_MOVIL_VENTANA);
    SimpleKalman kalman(3, KALMAN_Q, KALMAN_R);
    
    // Variables para datos de gráfico distancias
    float tiempos[1000];
    float distancias_hist[1000][3];
    float distancias_media_hist[1000][3];
    float distancias_kalman_hist[1000][3];
    float postag_hist[1000];
    unsigned long start_time = millis();
    float angulos_hist[1000];
    float tag2d[1000][2];
    int ciclo = 0;
    float distancia_s1_hist[1000];

    int N_CICLOS = 100;  // Número de ciclos a graficar
    
    if (!data_receiver.connect()) {
        Serial.printf("Error: No se pudo conectar al receptor de datos en %d\n", DATA_PORT);
        return;
    }
    
    if (!motor_controller.connect()) {
        Serial.printf(" Error: No se pudo conectar al controlador de motores en %d\n", DATA_PORT);
        data_receiver.disconnect();
        return;
    }
    
    // PID para control de ángulo
    PIDController pid(1.2, 0.01, 0.3, 0.0, PID_ALPHA, PID_SALIDA_MAX, INTEGRAL_MAX);
    
    // PID para control de distancia (seguimiento a distancia fija)
    PIDController pid_distancia(PID_DISTANCIA_KP, PID_DISTANCIA_KI, PID_DISTANCIA_KD, 
                                DISTANCIA_OBJETIVO, PID_DISTANCIA_ALPHA, 
                                VELOCIDAD_MAXIMA, PID_DISTANCIA_INTEGRAL_MAX);
    
    ciclo = 0;
    float distancias_previas[3] = {0, 0, 0}; // None
    bool distancias_previas_valid = false;
    float angulo_anterior = 0;  // Inicializamos la variable
    float y_anterior = 0.0;  // Inicialización del valor Y anterior
    float velocidad_actual = 0;  // Para suavizado de velocidad

    while (true) {
        void* data = data_receiver.read_data();
        
        // ###########################################################
        // PASO 1: Obtengo datos
        // ###########################################################
        if (data != nullptr) {
            //Serial.println("Datos completos en cm:", data, type(data));
            float* distancias = data_receiver.get_distances_array();
            
            // Paso 1.5: Limito variación entre ciclos (anti-salto)
            if (distancias_previas_valid) {
                distancias = limitar_variacion(distancias, distancias_previas, MAX_DELTA);
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
            
            float* distancias_media = media_movil.actualizar(distancias);
            //Serial.printf("Las distancias POR MEDIA en mm son: %f,%f,%f\n", distancias_media[0], distancias_media[1], distancias_media[2]);
            
            // ###########################################################
            // PASO 3: FILTRO KALMAN
            // ###########################################################
            float* distancias_kalman = kalman.filtrar(distancias_media);
            Serial.printf("Las distancias POR KALMAN en mm son: %.1f,%.1f,%.1f\n", distancias_kalman[0], distancias_kalman[1], distancias_kalman[2]);
            
            // ###########################################################
            // MOSTRAR DATOS ADICIONALES DEL ESP32 (FORMATO CORRECTO)
            // ###########################################################
            // Obtener datos con el mapeo correcto del formato real
            float* power_data = data_receiver.get_power_data_array();      // [BAT_C, ML_C, MR_C] pos 3-5
            float* imu_data = data_receiver.get_imu_data_array();          // [PITCH, ROLL, YAW, MOV, VEL, ACCEL_Z] pos 6-11
            float* tag_sensors = data_receiver.get_tag_sensors_array();    // [S1, S2, S3] pos 12-14
            float* control_data = data_receiver.get_control_data_array();  // [BAT_TAG, MODO] pos 15-16
            float* buttons_data = data_receiver.get_buttons_data_array();  // [JOY_D, JOY_A, JOY_I, JOY_T] pos 17-20
            
            Serial.printf(" CARRO: BAT_C:%.2fV | ML_C:%.2fA | MR_C:%.2fA\n", power_data[0], power_data[1], power_data[2]);
            Serial.printf(" IMU: Pitch:%.1f° | Roll:%.1f° | Yaw:%.1f° | MOV:%.0f | VEL:%.2f | ACCEL_Z:%.2f\n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
            Serial.printf(" TAG: S1:%.1f | S2:%.1f | S3:%.1f | BAT_TAG:%.2fV | MODO:%.0f\n", tag_sensors[0], tag_sensors[1], tag_sensors[2], control_data[0], control_data[1]);
            Serial.printf(" JOYSTICK: D:%.0f A:%.0f I:%.0f T:%.0f\n", buttons_data[0], buttons_data[1], buttons_data[2], buttons_data[3]);
            
            // ###########################################################
            // PASO 4: Mediante trilateración obtengo la ubicación del tag
            // ###########################################################
            float* pos_tag3d = trilateracion_3d(SENSOR_POSICIONES, distancias);
            float* pos_tag3d_media = trilateracion_3d(SENSOR_POSICIONES, distancias_media);
            float* pos_tag3d_kalman = trilateracion_3d(SENSOR_POSICIONES, distancias_kalman);

            // ###########################################################
            // GUARDAR PARA GRAFICAR #####################################
            // ###########################################################
            tiempos[ciclo] = (millis() - start_time) / 1000.0; // tiempos.append(time.time() - start_time)
            for(int i = 0; i < 3; i++) {
                distancias_hist[ciclo][i] = distancias_previas[i];      // distancias_hist.append(distancias_previas.copy())
                distancias_media_hist[ciclo][i] = distancias_media[i];  // distancias_media_hist.append(distancias_media.copy())
                distancias_kalman_hist[ciclo][i] = distancias_kalman[i]; // distancias_kalman_hist.append(distancias_kalman.copy())
            }
            postag_hist[ciclo] = pos_tag3d_kalman[1]; // postag_hist.append(pos_tag3d_kalman[1].copy())
            // posiciones_tag = np.array([[0.0, y, 0.0] for y in postag_hist])

            distancia_s1_hist[ciclo] = distancias_kalman[0]; // distancia_s1_hist.append(distancias_kalman[0])
            
            // ###########################################################
            // PASO 5: Calculo del ángulo a partir de trilateración
            // -180° hacia la izquierda y 180° hacia la derecha
            // ###########################################################
            float angulo_actual = angulo_direccion_xy(pos_tag3d_kalman);
    
            float angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual); // evita el salto entre -179 y 179
            float angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, MAX_DELTA);  // Limita los cambios bruscos
            angulo_anterior = angulo_relativo;  // Actualizamos para la próxima iteración
            angulos_hist[ciclo] = angulo_actual; // Para graficar
            Serial.printf("ÁNGULO : %.2f\n", angulo_relativo);

            // ###########################################################
            // PASO 6: UMBRAL 
            // ###########################################################
            float correccion;
            if (debe_corregir(angulo_relativo, UMBRAL)) {
                // ###########################################################
                // PASO 7: CCNTROL PID 
                // ###########################################################
                correccion = pid.update(angulo_relativo);
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
            float velocidad_pid = pid_distancia.update(distancia_al_tag);
            
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
            velocidad_actual = suavizar_velocidad(velocidad_actual, velocidad_objetivo, aceleracion_maxima=ACELERACION_MAXIMA);
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
                pwm_manager.enviar_pwm(vel_izq, vel_der);
            } else {
                pwm_manager.detener();
            }
            
        } else {
            Serial.println("Aún no hay datos válidos");
        }

        

        
    }
}

    
