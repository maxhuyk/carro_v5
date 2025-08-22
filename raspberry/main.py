from src.config.variables import (
NUM_CICLOS, 
TIEMPO_ESPERA,  
DATA_PORT,
BAUDRATE,
N_SENSORES,
SENSOR_POSICIONES, 
MEDIA_MOVIL_VENTANA,
KALMAN_Q,
KALMAN_R,
MAX_DELTA,
MAX_DELTA_POS,
UMBRAL,
PID_KP, PID_KI, PID_KD, PID_SETPOINT,
PID_ALPHA, PID_SALIDA_MAX, INTEGRAL_MAX,
DISTANCIA_OBJETIVO,
PID_DISTANCIA_KP, PID_DISTANCIA_KI, PID_DISTANCIA_KD,
PID_DISTANCIA_ALPHA, PID_DISTANCIA_INTEGRAL_MAX,
VELOCIDAD_MAXIMA,
ACELERACION_MAXIMA,
VELOCIDAD_MANUAL,
DISTANCIA_FALLBACK,
VELOCIDAD_FALLBACK,
TIMEOUT_FALLBACK
)
from src.utils.lectores.uart_data_receiver import UARTDataReceiver
from src.utils.auxiliares.mediam import MediaMovil
from src.utils.auxiliares.mikalman import SimpleKalman
from src.utils.auxiliares.trilateracion import trilateracion_3d
from src.utils.auxiliares.angulo_direccion import angulo_direccion_xy, desenrollar_angulo, limitar_cambio
from src.utils.auxiliares.limitador_distancia import limitar_variacion
from src.utils.auxiliares.control_diferencial import calcular_velocidades_diferenciales
from src.utils.auxiliares.validador import debe_corregir
from src.utils.auxiliares.limite_aceleracion import suavizar_velocidad
#from src.utils.auxiliares.limitar_tag_y import limitar_variacion_y
from src.utils.controladores.pid_controller import PIDController
from src.utils.controladores.pwm_manager import PWMManager
from src.utils.controladores.control import ControlModos
from src.utils.controladores.uart_motor_controller import UARTMotorController

from time import sleep
import numpy as np
import time


def quat_to_pitch_roll(qi, qj, qk, qr):
    """Retorna (pitch, roll) en grados a partir de un cuaternión (convención Tait-Bryan)."""
    # roll (x-axis)
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))
    # pitch (y-axis)
    sinp = 2.0 * (qr * qj - qk * qi)
    pitch = np.degrees(np.arcsin(np.clip(sinp, -1.0, 1.0)))
    return float(pitch), float(roll)

def quat_normalize(qi, qj, qk, qr):
    n = np.sqrt(qi*qi + qj*qj + qk*qk + qr*qr)
    if n == 0:
        return 0.0, 0.0, 0.0, 1.0
    return qi/n, qj/n, qk/n, qr/n

def quat_conjugate(q):
    qi, qj, qk, qr = q
    return (-qi, -qj, -qk, qr)

def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    # Hamilton product (w + xi + yj + zk)
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (x, y, z, w)

# Flags de orientación: aplicar rotaciones de 180° sobre ejes del TAG
# Ajusta signos de pitch/roll para coincidir con el montaje físico
# Nota:
#  - Z_180 invierte ambos (pitch y roll)
#  - X_180 invierte solo pitch
#  - Y_180 invierte solo roll
ORIENT_Z_180 = False  # Invertido en eje Z 180° (ambos ejes se invierten)
ORIENT_X_180 = False  # Invertido 180° adicional (si el frente está 180° al revés)
ORIENT_Y_180 = False  # Usualmente no necesario; habilitar si se detecta inversión en Y

def ajustar_orientacion_tilt(pitch_deg: float, roll_deg: float):
    """Aplica correcciones de orientación de 180° a (pitch, roll)."""
    p, r = pitch_deg, roll_deg
    if ORIENT_Z_180:
        p, r = -p, -r
    if ORIENT_X_180:
        p = -p
    if ORIENT_Y_180:
        r = -r
    return p, r

def tilt_to_pwm(pitch_deg, roll_deg, dead=7.5, max_tilt=25.0, max_pwm=255):
    """Mapea inclinación directamente a PWM -255..255 para cada motor."""
    def norm(v):
        if abs(v) <= dead:
            return 0.0
        return float(np.clip(v / max_tilt, -1.0, 1.0))
    F = norm(pitch_deg)   # +: adelante, -: atrás
    T = norm(roll_deg)    # +: derecha, -: izquierda
    l = float(np.clip(F + T, -1.0, 1.0))
    r = float(np.clip(F - T, -1.0, 1.0))
    to_pwm = lambda x: int(np.clip(round(x * max_pwm), -max_pwm, max_pwm))
    return to_pwm(l), to_pwm(r)

# Configuración de ejes para TILT (permite adaptar sin tocar fórmulas)
# Seleccionar cuál eje controla avance/retroceso y cuál controla giro
# Valores posibles para *_AXIS: 'pitch' o 'roll'; *_SIGN: 1 o -1
TILT_FORWARD_AXIS = 'pitch'   # 'pitch' o 'roll'
TILT_FORWARD_SIGN = -1         # 1 o -1
TILT_TURN_AXIS = 'roll'       # 'pitch' o 'roll'
TILT_TURN_SIGN = 1            # 1 o -1


def es_modo_valido(modo):
    """
    Verificar si el modo recibido es válido
    """
    return modo in [0, 1, 2, 3, 4]



def ejecutar_modo_seguimiento(data_receiver, motor_controller, pwm_manager, control_modos, error_logger=None):
    """
    MODO 1: Seguimiento continuo con pipeline estable de v5, manteniendo extras v7.
    """
    media_movil = MediaMovil(n_sensores=3, ventana=MEDIA_MOVIL_VENTANA)
    kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)

    # PID de v5 para ángulo
    pid = PIDController(kp=1.2, ki=0.01, kd=0.3, setpoint=0.0, alpha=PID_ALPHA,
                        salida_maxima=PID_SALIDA_MAX, integral_maxima=INTEGRAL_MAX)

    # PID de distancia como v5
    pid_distancia = PIDController(kp=PID_DISTANCIA_KP, ki=PID_DISTANCIA_KI, kd=PID_DISTANCIA_KD,
                                  setpoint=DISTANCIA_OBJETIVO, alpha=PID_DISTANCIA_ALPHA,
                                  salida_maxima=VELOCIDAD_MAXIMA, integral_maxima=PID_DISTANCIA_INTEGRAL_MAX)

    distancias_previas = None
    angulo_anterior = 0
    velocidad_actual = 0
    
    # Variables para modo fallback
    ultimo_angulo_valido = 0
    ultima_distancia_valida = DISTANCIA_OBJETIVO
    primer_nan_tiempo = None
    en_modo_fallback = False

    while True:
        data = data_receiver.read_data()

        if data is not None:
            # Cambio de modo con confirmación breve (anti-jitter)
            modo_recibido = int(data_receiver.get_control_data_array()[1])
            if es_modo_valido(modo_recibido) and modo_recibido != 1:
                t0 = time.time()
                confirmado = True
                while time.time() - t0 < 0.05:
                    d2 = data_receiver.read_data()
                    if d2 is None:
                        continue
                    if int(data_receiver.get_control_data_array()[1]) != modo_recibido:
                        confirmado = False
                        break
                if confirmado:
                    print(f" Cambiando de MODO 1 a MODO {modo_recibido}")
                    return

            # v5: adquisición + limitar variación
            distancias = data_receiver.get_distances_array()
            
            # Verificar si hay valores NaN en las distancias
            hay_nan = any(np.isnan(d) for d in distancias)
            
            if hay_nan:
                # Manejo de modo fallback cuando hay NaN
                tiempo_actual = time.time()
                
                if not en_modo_fallback:
                    # Primera detección de NaN
                    primer_nan_tiempo = tiempo_actual
                    en_modo_fallback = True
                    print("[SEGUIMIENTO] NaN detectado - Activando modo fallback")
                
                tiempo_transcurrido = tiempo_actual - (primer_nan_tiempo or tiempo_actual)
                
                if tiempo_transcurrido > TIMEOUT_FALLBACK:
                    # Más de 1 segundo con NaN - detener completamente
                    print("[SEGUIMIENTO] Timeout fallback alcanzado - Deteniendo motores")
                    pwm_manager.detener()
                else:
                    # Dentro del tiempo límite - usar lógica fallback
                    if ultima_distancia_valida < DISTANCIA_FALLBACK:
                        # Distancia menor a umbral - no moverse
                        print(f"[SEGUIMIENTO] Fallback: Distancia {ultima_distancia_valida:.1f} < {DISTANCIA_FALLBACK} - No moverse")
                        pwm_manager.detener()
                    else:
                        # Distancia mayor a umbral - moverse hacia última posición conocida
                        print(f"[SEGUIMIENTO] Fallback: Moviéndose hacia última posición (dist={ultima_distancia_valida:.1f}, ang={ultimo_angulo_valido:.1f})")
                        
                        # Calcular velocidades usando último ángulo conocido
                        if abs(ultimo_angulo_valido) > UMBRAL:
                            correccion_fallback = ultimo_angulo_valido
                        else:
                            correccion_fallback = 0
                        
                        vel_izq, vel_der, _ = calcular_velocidades_diferenciales(
                            v_lineal=VELOCIDAD_FALLBACK, angulo_relativo=correccion_fallback, max_v=VELOCIDAD_MAXIMA)
                        pwm_manager.enviar_pwm(vel_izq, vel_der)
                
                # Saltar el resto del procesamiento normal
                if TIEMPO_ESPERA:
                    sleep(TIEMPO_ESPERA)
                continue
            else:
                # Datos válidos - salir del modo fallback si estaba activo
                if en_modo_fallback:
                    print("[SEGUIMIENTO] Señal recuperada - Saliendo del modo fallback")
                    en_modo_fallback = False
                    primer_nan_tiempo = None
            
            if distancias_previas is not None:
                distancias = limitar_variacion(distancias, distancias_previas, max_delta=MAX_DELTA)
            distancias_previas = distancias.copy()

            # media móvil y kalman (v5)
            distancias_media = media_movil.actualizar(distancias)
            distancias_kalman = kalman.filtrar(distancias_media)

            # trilateración y ángulo (v5)
            pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)
            angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
            angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual)
            angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, max_delta=MAX_DELTA)
            angulo_anterior = angulo_relativo
            
            # Guardar último ángulo válido para fallback
            ultimo_angulo_valido = angulo_relativo

            # PID ángulo con umbral (v5)
            if debe_corregir(angulo_relativo, umbral=UMBRAL):
                correccion = pid.update(angulo_relativo)
            else:
                correccion = 0

            # PID distancia y selección de velocidad objetivo (v5)
            distancia_al_tag = (distancias_kalman[0] + distancias_kalman[1]) / 2.0
            
            # Guardar última distancia válida para fallback
            ultima_distancia_valida = distancia_al_tag
            
            velocidad_pid = pid_distancia.update(distancia_al_tag)
            error_distancia = distancia_al_tag - DISTANCIA_OBJETIVO
            if abs(error_distancia) > 200:
                if error_distancia > 0:
                    velocidad_objetivo = int(np.clip(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA))
                else:
                    velocidad_objetivo = 0
            else:
                velocidad_objetivo = 0

            # suavizado de aceleración (v5)
            velocidad_actual = suavizar_velocidad(velocidad_actual, velocidad_objetivo, aceleracion_maxima=ACELERACION_MAXIMA)
            velocidad_avance = int(velocidad_actual)

            # enviar PWM como v5
            if velocidad_avance > 0.0:
                vel_izq, vel_der, _ = calcular_velocidades_diferenciales(
                    v_lineal=velocidad_avance, angulo_relativo=correccion, max_v=VELOCIDAD_MAXIMA)
                pwm_manager.enviar_pwm(vel_izq, vel_der)
            else:
                pwm_manager.detener()

        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

def ejecutar_modo_pausa(data_receiver, motor_controller, pwm_manager, control_modos, error_logger=None):
    """
    MODO 2: Pausa - procesa sensores pero no mueve motores
    """
    media_movil = MediaMovil(n_sensores=3, ventana=MEDIA_MOVIL_VENTANA)
    kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)
    distancias_previas = None
    
    
    
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            # Cambio de modo seguro
            control_data = data_receiver.get_control_data_array()
            modo_recibido = int(control_data[1])
            
            # Cambio de modo con pequeña confirmación
            if  es_modo_valido(modo_recibido) and modo_recibido != 2:
                t0 = time.time()
                confirmado = True
                while time.time() - t0 < 0.05:
                    d2 = data_receiver.read_data()
                    if d2 is None:
                        continue
                    modo2 = int(data_receiver.get_control_data_array()[1])
                    if modo2 != modo_recibido:
                        confirmado = False
                        break
                if confirmado:
                    print(f" Cambiando de MODO 2 a MODO {modo_recibido}")
                    return
            
            # Procesar sensores (mismo que seguimiento) pero SIN mover motores
            distancias = data_receiver.get_distances_array()
            
            if distancias_previas is not None:
                distancias = limitar_variacion(distancias, distancias_previas, max_delta=MAX_DELTA)
            distancias_previas = distancias.copy()
            
            distancias_media = media_movil.actualizar(distancias)
            distancias_kalman = kalman.filtrar(distancias_media)
            
            # Calcular pero NO enviar a motores
            pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)
            angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
            
            # Mantener motores detenidos
            pwm_manager.detener()
            
        else:
            pass
        
        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)


def ejecutar_modo_manual(data_receiver, motor_controller, pwm_manager, control_modos, error_logger=None):
    """
    MODO 3: Control manual por joystick
    """
    

    while True:
        data = data_receiver.read_data()

        if data is not None:
            # Verificar si el modo cambió
            control_data = data_receiver.get_control_data_array()
            modo_recibido = int(control_data[1])

            if es_modo_valido(modo_recibido) and modo_recibido != 3:
                t0 = time.time()
                confirmado = True
                while time.time() - t0 < 0.05:
                    d2 = data_receiver.read_data()
                    if d2 is None:
                        continue
                    modo2 = int(data_receiver.get_control_data_array()[1])
                    if modo2 != modo_recibido:
                        confirmado = False
                        break
                if confirmado:
                    print(f" Cambiando de MODO 3 a MODO {modo_recibido}")
                    return

            # Control manual con joystick (valor único 0-7)
            joy_valor = data_receiver.get_joystick_as_int()

            if isinstance(joy_valor, (int, np.integer)) and 0 <= joy_valor <= 7:
                vel_izq, vel_der = control_modos.control_manual_joystick_valor(joy_valor)
            else:
                vel_izq, vel_der = 0, 0

            # Enviar PWM a motores
            if vel_izq != 0 or vel_der != 0:
                pwm_manager.enviar_pwm(vel_izq, vel_der)
            else:
                pwm_manager.detener()
        else:
            pass

        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

def mostrar_datos_esp32(data_receiver):
    """
    Función auxiliar para mostrar datos del ESP32 (FORMATO NUEVO SIN IMU)
    """
    power_data = data_receiver.get_power_data_array()        # [BAT_C, ML_C, MR_C] pos 3-5
    anchor_status = data_receiver.get_anchor_status_array()  # [S1, S2, S3] pos 6-8
    control_data = data_receiver.get_control_data_array()    # [BAT_TAG, MODO] pos 9-10
    buttons_data = data_receiver.get_buttons_data_array()    # [D,A,I,T] (solo debug)
    joy_valor = data_receiver.get_joystick_as_int()          # Valor crudo 0-7 del TAG
    
    print(f" CARRO: BAT_C:{power_data[0]:.2f}V | ML_C:{power_data[1]:.2f}A | MR_C:{power_data[2]:.2f}A")
    print(f" ANCHORS: S1:{anchor_status[0]:.0f} | S2:{anchor_status[1]:.0f} | S3:{anchor_status[2]:.0f} (1=OK, 0=FAIL)")
    print(f" TAG: BAT_TAG:{control_data[0]:.2f}V | MODO:{control_data[1]:.0f}")
    
    # Mostrar joystick: valor único 0-7 y, opcionalmente, bits
    if 0 <= joy_valor <= 7:
        print(f" JOYSTICK: VALOR={joy_valor} (0-7) | BITS={buttons_data.astype(int) if buttons_data is not None and len(buttons_data)==4 else 'N/A'}")
    else:
        print(" JOYSTICK: SIN DATOS")

 
# Loop principal inicial
def main():
    # Configurar logger para errores
    data_receiver = UARTDataReceiver(port=DATA_PORT, baudrate=BAUDRATE) 
    motor_controller = UARTMotorController(port=DATA_PORT, baudrate=BAUDRATE)
    pwm_manager = PWMManager(motor_controller)
    control_modos = ControlModos(velocidad_manual=VELOCIDAD_MANUAL)
    
    # Variables para protección contra datos corruptos
    ultimo_modo_valido = 0  # Modo por defecto
    contador_errores_consecutivos = 0
    max_errores_consecutivos = 5  # Máximo de errores antes de usar modo seguro
    
    if not data_receiver.connect():
        error_msg = f"Error: No se pudo conectar al receptor de datos en {DATA_PORT}"
        print(error_msg)
        return
        
    if not motor_controller.connect():
        error_msg = f"Error: No se pudo conectar al controlador de motores en {DATA_PORT}"
        print(error_msg)
        data_receiver.disconnect()
        return
    
    print(" Sistema iniciado. Esperando datos para determinar modo...")
    
    # Loop principal de modos
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            # Obtener modo actual con breve confirmación para robustez
            control_data = data_receiver.get_control_data_array()  # [BAT_TAG, MODO]
            modo_recibido = int(control_data[1])
            t0 = time.time()
            estable = True
            while time.time() - t0 < 0.03:  # ~30 ms
                d2 = data_receiver.read_data()
                if d2 is None:
                    continue
                if int(data_receiver.get_control_data_array()[1]) != modo_recibido:
                    estable = False
                    break
            
            # VALIDAR EL MODO ANTES DE USARLO
            if not es_modo_valido(modo_recibido) or not estable:
                contador_errores_consecutivos += 1
                
                if contador_errores_consecutivos >= max_errores_consecutivos:
                    # Demasiados errores: usar modo seguro (APAGADO)
                    modo_actual = 0
                    
                    print(f" MODO SEGURO: Activando APAGADO por errores de comunicación")
                else:
                    # Usar el último modo válido
                    modo_actual = ultimo_modo_valido
                    
                    print(f" Datos corruptos - Manteniendo MODO {ultimo_modo_valido} ({contador_errores_consecutivos}/{max_errores_consecutivos} errores)")
            else:
                # Datos válidos: usar el modo recibido y resetear contador
                modo_actual = modo_recibido
                ultimo_modo_valido = modo_actual
                contador_errores_consecutivos = 0
            
            modo_descripcion = control_modos.procesar_modo(modo_actual)
            
            # Solo mostrar cambio de modo si es diferente al anterior
            print(f" MODO {modo_actual}: {modo_descripcion}")
            
            if modo_actual == 0:
                # MODO 0: APAGADO
                pwm_manager.detener()
                print(" Sistema APAGADO - Motores detenidos")
                
            elif modo_actual == 1:
                # MODO 1: SEGUIMIENTO CONTINUO
                print(" Iniciando MODO SEGUIMIENTO...")
                ejecutar_modo_seguimiento(data_receiver, motor_controller, pwm_manager, control_modos)

            elif modo_actual == 2:
                # MODO 2: PAUSA  
                print(" Iniciando MODO PAUSA...")
                ejecutar_modo_pausa(data_receiver, motor_controller, pwm_manager, control_modos)
                
            elif modo_actual == 3:
                # MODO 3: MANUAL
                print(" Iniciando MODO MANUAL...")
                ejecutar_modo_manual(data_receiver, motor_controller, pwm_manager, control_modos)
            elif modo_actual == 4:
                # MODO 4: TILT (inclinación con BNO080)
                print(" Iniciando MODO TILT (inclinación)...")
                # Loop de TILT simplificado: usa quaternion del TAG para controlar motores
                last_print = 0
                q_ref = None  # referencia para remover yaw/offset
                while True:
                    d = data_receiver.read_data()
                    if d is None:
                        if TIEMPO_ESPERA:
                            sleep(TIEMPO_ESPERA)
                        continue
                    # Verificar cambio de modo
                    control_data = data_receiver.get_control_data_array()
                    modo_recibido = int(control_data[1])
                    if modo_recibido != 4:
                        print(f" Cambiando de MODO 4 a MODO {modo_recibido}")
                        pwm_manager.detener()
                        break
                    # Obtener quaternion (i,j,k,real,acc)
                    q_i, q_j, q_k, q_r, q_acc = data_receiver.get_quaternion()
                    # Normalizar
                    q_i, q_j, q_k, q_r = quat_normalize(q_i, q_j, q_k, q_r)
                    # Calibrar referencia la primera vez
                    if q_ref is None:
                        q_ref = (q_i, q_j, q_k, q_r)
                        print(" TILT calibrado: referencia establecida")
                    # Q relativa para eliminar yaw/orientación inicial: q_rel = conj(q_ref) * q_cur
                    q_rel = quat_multiply(quat_conjugate(q_ref), (q_i, q_j, q_k, q_r))
                    # Derivar pitch/roll desde q_rel
                    pitch, roll = quat_to_pitch_roll(*q_rel)
                    # Ajuste de orientación según flags (si se usan)
                    pitch, roll = ajustar_orientacion_tilt(pitch, roll)
                    # Aplicar mapeo de ejes configurable (yaw-invariant)
                    if TILT_FORWARD_AXIS == 'pitch':
                        f = TILT_FORWARD_SIGN * pitch
                    else:
                        f = TILT_FORWARD_SIGN * roll
                    if TILT_TURN_AXIS == 'roll':
                        t = TILT_TURN_SIGN * roll
                    else:
                        t = TILT_TURN_SIGN * pitch
                    # Mapear inclinación directamente a PWM -255..255 con f,t
                    vL, vR = tilt_to_pwm(f, t, dead=10, max_tilt=30.0, max_pwm=VELOCIDAD_MANUAL)
                    pwm_manager.enviar_pwm(vL, vR)
                    # Debug ocasional
                    now = time.time()
                    if now - last_print > 1.0:
                        print(
                            f" TILT: pitch={pitch:.1f} roll={roll:.1f} | f={f:.1f} t={t:.1f} -> L={vL} R={vR} qacc={int(q_acc)} "
                            f"[Z180={ORIENT_Z_180} X180={ORIENT_X_180} Y180={ORIENT_Y_180} | FA={TILT_FORWARD_AXIS} FS={TILT_FORWARD_SIGN} TA={TILT_TURN_AXIS} TS={TILT_TURN_SIGN}]"
                        )
                        last_print = now
                
        else:
            print(" Esperando datos válidos...")
            
        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

if __name__ == "__main__":
    main()




