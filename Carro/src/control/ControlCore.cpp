#include "ControlCore.h"
#include "motor/MotorController.h"
#include "core/Log.h"

// =====================================================================================
// CONTROL CORE - Migración literal desde CarroClean/control.cpp
// =====================================================================================
// Este archivo encapsula la lógica original del control adaptada al modelo modular.
// Se mantienen las fases numeradas, comentarios y semántica original.
// Diferencias mínimas:
//  - Uso de macros LOG* en lugar de Serial.printf directo
//  - Fuente de 'modo' y velocidades manuales provendrá luego del módulo EspNow
//  - Encapsulación en ControlCore_run en vez de control_main con callbacks
//  - PID angular ahora toma sus valores de Config.h (ajustados a los originales)
// =====================================================================================

// ===== Estado global migrado literal (A-E) =====
static bool control_initialized = false;
static PIDController pid_ang;              // pid angular (pid original 'pid')
// Eliminado PID de distancia; se usa perfil proporcional directo
static float distancias_previas[3] = {0,0,0};
static bool dist_prev_valid = false;
static float angulo_anterior = 0;
static float velocidad_actual = 0;
static float correccion_anterior = 0; // (reservado si se necesitara suavizado)
static int ciclo = 0;

// Seguridad / pérdida de señal
static bool signal_lost = false;
static unsigned long signal_lost_time = 0;
static float last_valid_distance = 0;
static float last_valid_correction = 0;
static int last_valid_velocity = 0;
static bool was_moving_when_lost = false;

// Recuperación rápida
static bool in_recovery = false;
static unsigned long recovery_start_ms = 0;
static int recovery_cycles = 0;
static float saved_normal_Q = KALMAN_Q;

// Frecuencia
static unsigned long control_execution_count = 0;
static unsigned long last_frequency_check = 0;

// Modo y manual (por ahora placeholders hasta que se integre módulo EspNow)
static uint8_t modo = 0;          // 0..N
static int manual_vel_L = 0;      // -VELOCIDAD_MANUAL..VELOCIDAD_MANUAL
static int manual_vel_R = 0;

// ===== Inicialización =====
void ControlCore_init(){
  if(control_initialized) return;
  setupMotorControlDirect();
  media_movil_init(3, MEDIA_MOVIL_VENTANA);
  kalman_init(3, KALMAN_Q, KALMAN_R);
  // Gains literales (ojo: en CarroClean pid angular estaba hardcodeado a 1.2/0.01/0.3, pero ahora usamos los defines configurables)
  pid_init(&pid_ang, PID_KP, PID_KI, PID_KD, PID_SETPOINT, PID_ALPHA, PID_SALIDA_MAX, INTEGRAL_MAX);
  dist_prev_valid = false;
  angulo_anterior = 0;
  velocidad_actual = 0;
  correccion_anterior = 0;
  ciclo = 0;
  control_initialized = true;
  LOGI("CTRL","ControlCore inicializado (migración literal)");
}

// Helper para enviar PWM según modo
static void aplicarSalidaMotores(int pwmL, int pwmR){
  if(modo == 1){
    motor_enviar_pwm(pwmL, pwmR);
  } else if(modo == 3){
    int vL = constrain(manual_vel_L, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
    int vR = constrain(manual_vel_R, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
    LOGD("CTRL","[MANUAL] override L=%d R=%d", vL, vR);
    motor_enviar_pwm(vL, vR);
  } else {
    motor_enviar_pwm(0,0);
  }
}

void ControlCore_run(const CarroData& data){
  if(!control_initialized) return;

  // ###########################################################
  // PASO FREQ: Métrica de frecuencia de ejecución (cada 5s)
  // ###########################################################
  // Frecuencia (mantener lógica original cada 5s)
  control_execution_count++;
  unsigned long now = millis();
  if(now - last_frequency_check >= 5000){
    float freq = control_execution_count / 5.0f;
    LOGI("CTRL","[CONTROL_FREQ] %.1f Hz", freq);
    control_execution_count = 0;
    last_frequency_check = now;
  }

  // ###########################################################
  // PASO 0: Verificación de datos válidos (data_valid) - rama seguridad si faltan
  // ###########################################################
  if(!data.data_valid){
    // NO hay datos válidos -> rama seguridad similar a original (parte final)
    LOGW("CTRL","No hay datos válidos disponibles");
    if(!signal_lost){
      signal_lost = true; signal_lost_time = now;
      last_valid_distance = dist_prev_valid ? (distancias_previas[0]+distancias_previas[1])/2.0f : DISTANCIA_FALLBACK + 1000;
      was_moving_when_lost = (velocidad_actual > 0);
      LOGW("CTRL","[DATA_LOST] Ultima distancia=%.1f", last_valid_distance);
    }
    if(last_valid_distance < DISTANCIA_FALLBACK){
      LOGW("CTRL","[SAFETY] cerca sin datos -> stop");
      motor_enviar_pwm(0,0); velocidad_actual=0; return;
    } else {
      unsigned long elapsed = now - signal_lost_time;
      if(elapsed < (unsigned long)(TIMEOUT_FALLBACK*1000) && was_moving_when_lost){
        LOGW("CTRL","[SAFETY] lejos sin datos, continuando %.2fs", (TIMEOUT_FALLBACK*1000 - elapsed)/1000.0f);
        if(VELOCIDAD_FALLBACK > 0){
          auto vres = calcular_velocidades_diferenciales(VELOCIDAD_FALLBACK, last_valid_correction, VELOCIDAD_MAXIMA);
          aplicarSalidaMotores(vres.vel_izq, vres.vel_der);
        } else {
          motor_enviar_pwm(0,0);
        }
      } else {
        LOGW("CTRL","[SAFETY] timeout sin datos -> stop");
        motor_enviar_pwm(0,0); velocidad_actual=0;
      }
      return;
    }
  }

  // ###########################################################
  // PASO 0.5: MODO y control manual (se poblará desde EspNow luego)
  // ###########################################################
  modo = (uint8_t)data.control_data[1];
  manual_vel_L = (int)data.buttons_data[2];
  manual_vel_R = (int)data.buttons_data[3];
  LOGD("CTRL","[MODO] %u", (unsigned)modo);

  float distancias[3] = { data.distancias[0], data.distancias[1], data.distancias[2] };

  // ###########################################################
  // PASO 1: Validación básica de señal (NaN / fuera de rango)
  // ###########################################################
  // Esto decide si entramos a sistema de seguridad por pérdida de señal.
  bool signal_valid = true;
  for(int i=0;i<3;i++){
    if(isnan(distancias[i]) || distancias[i] <= 0 || distancias[i] > 40000){
      signal_valid = false;
      LOGW("CTRL","[SIGNAL_LOST] sensor %d=%.1f", i, distancias[i]);
      break;
    }
  }

  // ###########################################################
  // PASO 1.1: Rama de pérdida de señal (SIGNAL_LOST)
  // ###########################################################
  if(!signal_valid){
    if(!signal_lost){
      signal_lost = true; signal_lost_time = now;
      last_valid_distance = dist_prev_valid ? (distancias_previas[0]+distancias_previas[1])/2.0f : DISTANCIA_FALLBACK + 1000;
      was_moving_when_lost = (velocidad_actual > 0);
      LOGW("CTRL","[SIGNAL_LOST] activado ultimaDist=%.1f", last_valid_distance);
    }
    if(last_valid_distance < DISTANCIA_FALLBACK){
      LOGI("CTRL","[SAFETY] cerca -> stop"); motor_enviar_pwm(0,0); velocidad_actual=0; return; }
    unsigned long elapsed = now - signal_lost_time;
    if(elapsed < (unsigned long)(TIMEOUT_FALLBACK*1000) && was_moving_when_lost){
      LOGI("CTRL","[SAFETY] continuar %.2fs", (TIMEOUT_FALLBACK*1000 - elapsed)/1000.0f);
      if(VELOCIDAD_FALLBACK > 0){
        auto vres = calcular_velocidades_diferenciales(VELOCIDAD_FALLBACK, last_valid_correction, VELOCIDAD_MAXIMA);
        aplicarSalidaMotores(vres.vel_izq, vres.vel_der);
      } else motor_enviar_pwm(0,0);
    } else {
      LOGI("CTRL","[SAFETY] timeout -> stop"); motor_enviar_pwm(0,0); velocidad_actual=0; }
    return;
  } else {
    // ###########################################################
    // PASO 1.2: Recuperación de señal
    // ###########################################################
    if(signal_lost){
      LOGI("CTRL","[SIGNAL_RECOVERED] reanudando");
      signal_lost = false;
      float primera[3]; for(int i=0;i<3;i++) primera[i]=distancias[i];
      media_movil_init(3, MEDIA_MOVIL_VENTANA);
      float dummy[3]; for(int r=0;r<MEDIA_MOVIL_VENTANA;r++) media_movil_actualizar(primera, dummy);
      saved_normal_Q = kalman_get_Q();
      kalman_set_Q(RECOVERY_Q_TEMP);
      kalman_force_state(primera, RECOVERY_P_INIT);
      in_recovery = true; recovery_start_ms = now; recovery_cycles = 0;
      LOGI("CTRL","[RECOVERY] Q %.4f->%.4f P=%.1f", saved_normal_Q, (double)RECOVERY_Q_TEMP, (double)RECOVERY_P_INIT);
    }
  }

  // ###########################################################
  // PASO 1.3: Chequeo adicional de distancias (rango razonable)
  // ###########################################################
  bool dist_ok = true; for(int i=0;i<3;i++){ if(distancias[i]<=0 || distancias[i]>40000){ dist_ok=false; break; }}
  if(!dist_ok){ LOGE("CTRL","[CONTROL_ERROR] dist invalid %.1f %.1f %.1f", distancias[0], distancias[1], distancias[2]); motor_enviar_pwm(0,0); return; }

  // ###########################################################
  // PASO 1.5: Limitador de variación inter-frame (anti-salto)
  // ###########################################################
  if(dist_prev_valid){
    bool saltar_limitador=false;
    if(in_recovery){ unsigned long elapsed = now - recovery_start_ms; if(elapsed < RECOVERY_GRACE_MS) saltar_limitador=true; }
    if(!saltar_limitador){
      float dist_lim[3]; float delta = in_recovery ? RECOVERY_DELTA_POS : MAX_DELTA_POS; limitar_variacion(distancias, distancias_previas, dist_lim, 3, delta); for(int i=0;i<3;i++) distancias[i]=dist_lim[i];
    }
  }
  for(int i=0;i<3;i++) distancias_previas[i]=distancias[i]; dist_prev_valid=true;

  // ###########################################################
  // PASO 2: Filtro de MEDIA MÓVIL
  // ###########################################################
  float dist_media[3]; media_movil_actualizar(distancias, dist_media);
  // ###########################################################
  // PASO 3: Filtro KALMAN
  // ###########################################################
  float dist_kalman[3]; kalman_filtrar(dist_media, dist_kalman);
  LOGD("CTRL","KALMAN %.1f %.1f %.1f", dist_kalman[0], dist_kalman[1], dist_kalman[2]);
  if(in_recovery){
    recovery_cycles++;
    if(recovery_cycles >= RECOVERY_K_CYCLES && (now - recovery_start_ms) > RECOVERY_GRACE_MS){
      kalman_set_Q(saved_normal_Q); in_recovery=false; LOGI("CTRL","[RECOVERY_DONE] ciclos=%d", recovery_cycles);
    }
  }

  // ###########################################################
  // PASO 4: TRILATERACIÓN 3D -> pos_tag3d_kalman
  // ###########################################################
  float pos[3]; trilateracion_3d(SENSOR_POSICIONES, dist_kalman, pos);
  // ###########################################################
  // PASO 5: ÁNGULO (desenrollado + limitador de delta)
  // ###########################################################
  float angulo_actual = angulo_direccion_xy(pos);
  float angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual);
  float angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, MAX_DELTA);
  angulo_anterior = angulo_relativo;
  LOGD("CTRL","ANG %.2f", angulo_relativo);

  // ###########################################################
  // PASO 6: Distancia al TAG (promedio sensores frontales)
  // ###########################################################
  float distancia_al_tag = (dist_kalman[0] + dist_kalman[1]) * 0.5f;
  // ###########################################################
  // PASO 7: UMBRAL dinámico (tolerancia angular dependiente de la distancia)
  // ###########################################################
  float umbral = calcular_umbral_dinamico(distancia_al_tag, UMBRAL_MINIMO, UMBRAL_MAXIMO, DISTANCIA_UMBRAL_MIN, DISTANCIA_UMBRAL_MAX);

  float correccion;
  // ###########################################################
  // PASO 8: PID ANGULAR (si excede umbral)
  // ###########################################################
  if(debe_corregir(angulo_relativo, umbral)){
    correccion = pid_update(&pid_ang, angulo_relativo);
    LOGD("CTRL","DIST:%.0f UMB:%.1f PID:%.2f", distancia_al_tag, umbral, correccion);
  } else {
    correccion = 0; LOGD("CTRL","DIST:%.0f UMB:%.1f PID:0 (ign)", distancia_al_tag, umbral);
  }

  // ###########################################################
  // PASO 9: Perfil de velocidad progresivo (sin PID de distancia)
  // ###########################################################
  float error_distancia = distancia_al_tag - DISTANCIA_OBJETIVO;
  int velocidad_objetivo;
  if(error_distancia > 5000) velocidad_objetivo = VELOCIDAD_MAXIMA;
  else if(error_distancia > 100){
    float velocidad_proporcional = 20 + (error_distancia - 100) * 0.016f;
    if(velocidad_proporcional < 20) velocidad_proporcional = 20;
    if(velocidad_proporcional > VELOCIDAD_MAXIMA) velocidad_proporcional = VELOCIDAD_MAXIMA;
    velocidad_objetivo = (int)velocidad_proporcional;
  } else if(error_distancia < 50){ velocidad_objetivo = 0; }
  else {
    float vtmp = 8 + error_distancia * 0.05f; if(vtmp<0) vtmp=0; if(vtmp>25) vtmp=25; velocidad_objetivo = (int)vtmp;
  }
  velocidad_actual = suavizar_velocidad_avanzada(velocidad_actual, (float)velocidad_objetivo, ACELERACION_MAXIMA, DESACELERACION_MAXIMA);
  int velocidad_avance = (int)velocidad_actual;
  LOGD("CTRL","DIST %.1f OBJ %.1f ERR %.1f vObj %d vSuav %d (sin PID dist)", distancia_al_tag, (float)DISTANCIA_OBJETIVO, error_distancia, velocidad_objetivo, velocidad_avance);

  // ###########################################################
  // PASO 10: Control diferencial y envío de PWM
  // ###########################################################
  if(velocidad_avance > 0){
    auto velocidades_result = calcular_velocidades_diferenciales(velocidad_avance, correccion, VELOCIDAD_MAXIMA);
    LOGD("CTRL","[MOTOR_OUTPUT] L=%d R=%d", velocidades_result.vel_izq, velocidades_result.vel_der);
    aplicarSalidaMotores(velocidades_result.vel_izq, velocidades_result.vel_der);
    last_valid_distance = distancia_al_tag;
    last_valid_correction = correccion;
    last_valid_velocity = velocidad_avance;
  } else {
    if(modo == 3){
      int vL = constrain(manual_vel_L, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
      int vR = constrain(manual_vel_R, -VELOCIDAD_MANUAL, VELOCIDAD_MANUAL);
      LOGD("CTRL","[MANUAL idle] L=%d R=%d", vL, vR);
      motor_enviar_pwm(vL, vR);
    } else motor_enviar_pwm(0,0);
    last_valid_distance = distancia_al_tag; last_valid_correction = correccion; last_valid_velocity = 0;
  }
}
