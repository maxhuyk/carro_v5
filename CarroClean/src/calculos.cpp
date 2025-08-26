#include "calculos.h"
#include <Arduino.h>
#include <math.h>

// Variables estáticas para la media móvil (mantienen estado entre llamadas)
static float buffers[3][10]; // Máximo 3 sensores, ventana máxima 10
static int buffer_indices[3] = {0, 0, 0};
static int buffer_counts[3] = {0, 0, 0};
static int ventana_size = 5; // Por defecto
static int num_sensores = 3; // Por defecto

void media_movil_init(int n_sensores, int ventana) {
    num_sensores = n_sensores;
    ventana_size = ventana;
    
    // Limpiar buffers
    for (int i = 0; i < n_sensores; i++) {
        buffer_indices[i] = 0;
        buffer_counts[i] = 0;
        for (int j = 0; j < ventana; j++) {
            buffers[i][j] = 0.0;
        }
    }
}

void media_movil_actualizar(float* nuevos_valores, float* resultado) {
    for (int i = 0; i < num_sensores; i++) {
        // Agregar nuevo valor al buffer circular
        buffers[i][buffer_indices[i]] = nuevos_valores[i];
        buffer_indices[i] = (buffer_indices[i] + 1) % ventana_size;
        
        // Incrementar contador hasta llenar la ventana
        if (buffer_counts[i] < ventana_size) {
            buffer_counts[i]++;
        }
        
        // Calcular promedio
        float suma = 0.0;
        for (int j = 0; j < buffer_counts[i]; j++) {
            suma += buffers[i][j];
        }
        resultado[i] = suma / buffer_counts[i];
    }
}

// Variables estáticas para el filtro Kalman
static float kalman_x[3] = {0.0, 0.0, 0.0};  // Estado estimado
static float kalman_P[3] = {1.0, 1.0, 1.0};  // Covarianza estimada
static float kalman_Q = 0.01;  // Varianza del proceso
static float kalman_R = 1.0;   // Varianza de la medición
static int kalman_num_sensores = 3;

void kalman_init(int n_sensores, float Q, float R) {
    kalman_num_sensores = n_sensores;
    kalman_Q = Q;
    kalman_R = R;
    
    // Inicializar estado y covarianza
    for (int i = 0; i < n_sensores; i++) {
        kalman_x[i] = 0.0;
        kalman_P[i] = 1.0;
    }
}

void kalman_filtrar(float* mediciones, float* resultado) {
    for (int i = 0; i < kalman_num_sensores; i++) {
        // Predicción
        kalman_P[i] += kalman_Q;
        
        // Ganancia de Kalman
        float K = kalman_P[i] / (kalman_P[i] + kalman_R);
        
        // Actualización
        kalman_x[i] = kalman_x[i] + K * (mediciones[i] - kalman_x[i]);
        kalman_P[i] = (1 - K) * kalman_P[i];
        
        resultado[i] = kalman_x[i];
    }
}

// ===== Funciones auxiliares para gestión dinámica del Kalman (recuperación) =====
void kalman_force_state(float* mediciones, float P_init) {
    // Fija el estado a las mediciones actuales y pone una covarianza inicial alta
    for (int i = 0; i < kalman_num_sensores; i++) {
        kalman_x[i] = mediciones[i];
        kalman_P[i] = P_init; // Covarianza grande -> ganancia cercana a 1 en primeras iteraciones
    }
}

void kalman_set_Q(float Q) {
    kalman_Q = Q;
}

float kalman_get_Q() { return kalman_Q; }

// Funciones para controlador PID
void pid_init(PIDController* pid, float kp, float ki, float kd, float setpoint, float alpha, float salida_maxima, float integral_maxima) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->alpha = alpha;
    pid->error_anterior = 0.0;
    pid->integral = 0.0;
    pid->salida_maxima = salida_maxima;
    pid->integral_maxima = integral_maxima;
}

float pid_update(PIDController* pid, float medida_actual) {
    // Error actual
    float error_actual = pid->setpoint - medida_actual;
    
    // Filtro exponencial del error
    float error_filtrado = pid->alpha * error_actual + (1 - pid->alpha) * pid->error_anterior;
    
    // Derivada
    float derivada = error_filtrado - pid->error_anterior;
    
    // Acumulación de la integral
    pid->integral += error_filtrado;
    
    // Clamping de la integral
    if (pid->integral_maxima != 0.0) {
        if (pid->integral > pid->integral_maxima) pid->integral = pid->integral_maxima;
        if (pid->integral < -pid->integral_maxima) pid->integral = -pid->integral_maxima;
    }
    
    // PID completo
    float salida = pid->kp * error_filtrado + pid->ki * pid->integral + pid->kd * derivada;
    
    // Saturación de la salida
    if (pid->salida_maxima != 0.0) {
        if (salida > pid->salida_maxima) salida = pid->salida_maxima;
        if (salida < -pid->salida_maxima) salida = -pid->salida_maxima;
    }
    
    // Actualizar error anterior
    pid->error_anterior = error_filtrado;
    
    return salida;
}

// Función para limitar variación de distancias
void limitar_variacion(float* dist_actual, float* dist_anterior, float* resultado, int n_sensores, float max_delta) {
    for (int i = 0; i < n_sensores; i++) {
        float delta = dist_actual[i] - dist_anterior[i];
        if (delta > max_delta) {
            delta = max_delta;
        } else if (delta < -max_delta) {
            delta = -max_delta;
        }
        resultado[i] = dist_anterior[i] + delta;
    }
}

// Funciones auxiliares para operaciones vectoriales
float vector_norm(float* v, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += v[i] * v[i];
    }
    return sqrt(sum);
}

void vector_subtract(float* a, float* b, float* result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = a[i] - b[i];
    }
}

void vector_add(float* a, float* b, float* result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = a[i] + b[i];
    }
}

void vector_scale(float* v, float scale, float* result, int size) {
    for (int i = 0; i < size; i++) {
        result[i] = v[i] * scale;
    }
}

float vector_dot(float* a, float* b, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

void vector_cross_3d(float* a, float* b, float* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// Función para trilateración 3D
void trilateracion_3d(const float sensor_posiciones[3][3], float* distancias, float* resultado) {
    float P1[3] = {sensor_posiciones[0][0], sensor_posiciones[0][1], sensor_posiciones[0][2]};
    float P2[3] = {sensor_posiciones[1][0], sensor_posiciones[1][1], sensor_posiciones[1][2]};
    float P3[3] = {sensor_posiciones[2][0], sensor_posiciones[2][1], sensor_posiciones[2][2]};
    
    float r1 = distancias[0];
    float r2 = distancias[1];
    float r3 = distancias[2];

    // Vector X: de P1 a P2
    float p2_p1[3];
    vector_subtract(P2, P1, p2_p1, 3);
    float norm_p2_p1 = vector_norm(p2_p1, 3);
    float ex[3];
    vector_scale(p2_p1, 1.0/norm_p2_p1, ex, 3);

    // Vector Y: lo definimos manualmente como hacia adelante
    float ey_dir[3] = {0, 1, 0};  // Y hacia adelante en sistema global
    float norm_ey_dir = vector_norm(ey_dir, 3);
    float ey[3];
    vector_scale(ey_dir, 1.0/norm_ey_dir, ey, 3);

    // Vector Z: lo obtenemos como ex × ey → apuntará hacia arriba
    float ez[3];
    vector_cross_3d(ex, ey, ez);

    // Ajustamos el plano de trabajo
    float d = norm_p2_p1;
    float p3_p1[3];
    vector_subtract(P3, P1, p3_p1, 3);
    float i = vector_dot(ex, p3_p1, 3);
    float j = vector_dot(ey, p3_p1, 3);

    float x = (r1*r1 - r2*r2 + d*d) / (2 * d);
    float y = (r1*r1 - r3*r3 + i*i + j*j) / (2 * j) - (i / j) * x;
    float z2 = r1*r1 - x*x - y*y;

    float z;
    if (z2 < 0) {
        z = 0.0;
    } else {
        z = sqrt(z2);
    }

    // resultado = P1 + x * ex + y * ey + z * ez
    float temp1[3], temp2[3], temp3[3];
    vector_scale(ex, x, temp1, 3);
    vector_scale(ey, y, temp2, 3);
    vector_scale(ez, z, temp3, 3);
    
    vector_add(P1, temp1, resultado, 3);
    vector_add(resultado, temp2, resultado, 3);
    vector_add(resultado, temp3, resultado, 3);
}

// Funciones para cálculo y manejo de ángulos
float angulo_direccion_xy(float* pos_tag) {
    /*
    Calcula el ángulo entre el vector desde el origen al tag y el eje Y positivo.
    Ángulo positivo en sentido antihorario, negativo en sentido horario.
    Retorna ángulo en grados entre -180 y +180.
    */
    float x = pos_tag[0];
    float y = pos_tag[1];

    // Vector desde el origen hacia el tag
    // Ángulo respecto al eje Y: usamos atan2 para obtener signo
    float angulo_rad = atan2(x, y);  // Invierte los parámetros para que 0° sea en eje Y
    float angulo_grados = angulo_rad * 180.0 / M_PI;  // Convertir a grados

    return angulo_grados;
}

// Esto evita saltos bruscos entre ±180° y mantiene la continuidad angular
float desenrollar_angulo(float previo, float actual) {
    float delta = actual - previo;
    if (delta > 180) {
        delta -= 360;
    } else if (delta < -180) {
        delta += 360;
    }
    return previo + delta;
}

// Si el tag se mueve rápidamente, incluso con desenrollado, el ángulo puede cambiar abruptamente. 
// Para evitar valores "no reales", limitamos cuán rápido puede variar el ángulo entre frames.
// Esto evita que el sistema salte de, digamos, 30° a 150°, cuando quizás solo debería haber cambiado a 50°.
float limitar_cambio(float previo, float actual, float max_delta) {
    float delta = actual - previo;
    if (delta > max_delta) {
        delta = max_delta;
    } else if (delta < -max_delta) {
        delta = -max_delta;
    }
    return previo + delta;
}

// Función de validación/umbral
bool debe_corregir(float angulo, float umbral) {
    /*
    Evalúa si el ángulo es suficientemente grande para justificar una corrección.
    
    Parámetros:
    - angulo: float, en grados
    - umbral: float, umbral mínimo (ej. 1.0°)
    
    Retorna:
    - bool: True si se debe corregir, False si se ignora
    */
    return abs(angulo) > umbral;
}

// Función para calcular umbral dinámico basado en distancia
float calcular_umbral_dinamico(float distancia_mm, float umbral_min, float umbral_max, 
                               float dist_min, float dist_max) {

    
    // Limitar distancia al rango válido
    if (distancia_mm <= dist_min) {
        return umbral_min;
    } else if (distancia_mm >= dist_max) {
        return umbral_max;
    } else {
        // Interpolación lineal entre dist_min y dist_max
        float factor = (distancia_mm - dist_min) / (dist_max - dist_min);
        float umbral_dinamico = umbral_min + factor * (umbral_max - umbral_min);
        return umbral_dinamico;
    }
}

// Función para suavizar velocidad (control de aceleración)
float suavizar_velocidad(float velocidad_actual, float velocidad_objetivo, float aceleracion_maxima) {
    // CASO 1: Arranque desde parado (0 -> velocidad)
    if (velocidad_actual == 0 && velocidad_objetivo > 0) {
        return min(velocidad_objetivo, aceleracion_maxima);
    }
    
    // CASO 2: Frenado hacia parado (velocidad -> 0)
    else if (velocidad_actual > 0 && velocidad_objetivo == 0) {
        return max(0.0f, velocidad_actual - aceleracion_maxima);
    }
    
    // CASO 3: Cambios normales (velocidad -> velocidad diferente)
    // No aplicar filtro para no afectar las curvas
    else {
        return velocidad_objetivo;
    }
}

// Nueva función con aceleración y desaceleración separadas
float suavizar_velocidad_avanzada(float velocidad_actual, float velocidad_objetivo, 
                                  float aceleracion_maxima, float desaceleracion_maxima) {
    float diferencia = velocidad_objetivo - velocidad_actual;
    
    if (diferencia > 0) {
        // Necesita acelerar
        float incremento = min(diferencia, aceleracion_maxima);
        return velocidad_actual + incremento;
    }
    else if (diferencia < 0) {
        // Necesita frenar/desacelerar
        float decremento = min(abs(diferencia), desaceleracion_maxima);
        return velocidad_actual - decremento;
    }
    else {
        // Ya está en la velocidad objetivo
        return velocidad_objetivo;
    }
}

// Función auxiliar para clipping
float clip(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Función para calcular velocidades diferenciales
VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lineal, float angulo_relativo, float max_v) {
    /*
    Calcula las velocidades para las ruedas izquierda y derecha en base a
    la velocidad lineal y el ángulo de giro relativo.

    Parámetros:
    - v_lineal (float): velocidad deseada en línea recta (PWM)
    - angulo_relativo (float): ángulo entre el frente del robot y el tag (grados)
    - max_v (float): velocidad máxima (PWM)

    Retorna:
    - VelocidadesDiferenciales: velocidades para rueda izquierda y derecha, y giro normalizado
    */
    
    VelocidadesDiferenciales resultado;
    
    // Escalado del ángulo a [-1.0, 1.0] (control de curvatura)
    resultado.giro_normalizado = clip(angulo_relativo / 30.0, -1.0, 1.0);

    // Velocidades diferenciales
    float vel_der_float = clip(v_lineal * (1.0 - resultado.giro_normalizado), 0, max_v);
    float vel_izq_float = clip(v_lineal * (1.0 + resultado.giro_normalizado), 0, max_v);
    
    resultado.vel_der = (int)vel_der_float;
    resultado.vel_izq = (int)vel_izq_float;

    return resultado;
}

// Función para limitar velocidad angular (suavizado de correcciones PID)
float limitar_velocidad_angular(float correccion_actual, float correccion_anterior, 
                                float max_cambio_por_ciclo, float factor_suavizado) {
    /*
    Limita la velocidad de cambio de la corrección angular para evitar movimientos bruscos.
    
    Parámetros:
    - correccion_actual: corrección que quiere aplicar el PID
    - correccion_anterior: corrección aplicada en el ciclo anterior
    - max_cambio_por_ciclo: máximo cambio permitido por ciclo (ej. 3°)
    - factor_suavizado: factor de suavizado (0.0-1.0, donde 1.0 = sin suavizado)
    
    Retorna:
    - float: corrección limitada y suavizada
    */
    
    // Calcular el cambio deseado
    float cambio_deseado = correccion_actual - correccion_anterior;
    
    // Limitar el cambio máximo por ciclo
    float cambio_limitado;
    if (cambio_deseado > max_cambio_por_ciclo) {
        cambio_limitado = max_cambio_por_ciclo;
    } else if (cambio_deseado < -max_cambio_por_ciclo) {
        cambio_limitado = -max_cambio_por_ciclo;
    } else {
        cambio_limitado = cambio_deseado;
    }
    
    // Aplicar suavizado adicional
    float cambio_suavizado = cambio_limitado * factor_suavizado;
    
    // Calcular nueva corrección
    float correccion_suavizada = correccion_anterior + cambio_suavizado;
    
    return correccion_suavizada;
}

// Función para calcular salida máxima del PID basada en velocidad actual
float calcular_pid_salida_max_por_velocidad(float velocidad_actual, 
                                            float vel_baja, float vel_alta,
                                            float salida_max_baja, float salida_max_alta) {
    /*
    Calcula la salida máxima del PID angular basada en la velocidad actual del vehículo.
    A menor velocidad, permite correcciones más agresivas.
    A mayor velocidad, limita las correcciones para seguridad.
    
    Parámetros:
    - velocidad_actual: velocidad actual del vehículo (PWM)
    - vel_baja: velocidad considerada "baja" (ej. 10 PWM)
    - vel_alta: velocidad considerada "alta" (ej. 40 PWM)
    - salida_max_baja: máxima corrección a velocidad baja (ej. 15°)
    - salida_max_alta: máxima corrección a velocidad alta (ej. 3°)
    
    Retorna:
    - float: salida máxima calculada dinámicamente
    
    Lógica:
    - A velocidad ≤ vel_baja: salida_max_baja (corrección agresiva)
    - A velocidad ≥ vel_alta: salida_max_alta (corrección suave)
    - Entre vel_baja y vel_alta: interpolación lineal
    */
    
    // Limitar velocidad al rango válido
    if (velocidad_actual <= vel_baja) {
        return salida_max_baja;
    } else if (velocidad_actual >= vel_alta) {
        return salida_max_alta;
    } else {
        // Interpolación lineal entre vel_baja y vel_alta
        float factor = (velocidad_actual - vel_baja) / (vel_alta - vel_baja);
        float salida_max_dinamica = salida_max_baja - factor * (salida_max_baja - salida_max_alta);
        return salida_max_dinamica;
    }
}
