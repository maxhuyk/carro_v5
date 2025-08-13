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
VELOCIDAD_MANUAL
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


def es_modo_valido(modo):
    """
    Verificar si el modo recibido es válido
    """
    return modo in [0, 1, 2, 3]



def ejecutar_modo_seguimiento(data_receiver, motor_controller, pwm_manager, control_modos, error_logger=None):
    """
    MODO 1: Seguimiento continuo con PID
    """
    media_movil = MediaMovil(n_sensores=3, ventana=MEDIA_MOVIL_VENTANA)
    kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)
    
    # PID para control de ángulo
    pid = PIDController(kp=1.2, ki=0.01, kd=0.3, setpoint=0.0, alpha=PID_ALPHA, 
                       salida_maxima=PID_SALIDA_MAX, integral_maxima=INTEGRAL_MAX)
    
    # PID para control de distancia
    pid_distancia = PIDController(kp=PID_DISTANCIA_KP, ki=PID_DISTANCIA_KI, kd=PID_DISTANCIA_KD, 
                                  setpoint=DISTANCIA_OBJETIVO, alpha=PID_DISTANCIA_ALPHA, 
                                  salida_maxima=VELOCIDAD_MAXIMA, integral_maxima=PID_DISTANCIA_INTEGRAL_MAX)
    
    distancias_previas = None
    angulo_anterior = 0
    velocidad_actual = 0
    
    
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            # Cambio de modo seguro
            control_data = data_receiver.get_control_data_array()
            modo_recibido = int(control_data[1])
            
            # Cambio de modo inmediato si es válido y distinto
            if es_modo_valido(modo_recibido) and modo_recibido != 1:
                
                print(f" Cambiando de MODO 1 a MODO {modo_recibido}")
                return  # Salir del modo seguimiento
            
            # Procesar sensores y control PID
            distancias = data_receiver.get_distances_array()
            
            if distancias_previas is not None:
                distancias = limitar_variacion(distancias, distancias_previas, max_delta=MAX_DELTA)
            distancias_previas = distancias.copy()
            
            distancias_media = media_movil.actualizar(distancias)
            distancias_kalman = kalman.filtrar(distancias_media)
            
            # Trilateración y control PID
            pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)
            angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
            angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual)
            angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, max_delta=MAX_DELTA)
            angulo_anterior = angulo_relativo
            
            # Control PID de ángulo
            if debe_corregir(angulo_relativo, umbral=UMBRAL):
                correccion = pid.update(angulo_relativo)
            else:
                correccion = 0
            
            # Control PID de distancia
            distancia_al_tag = (distancias_kalman[0] + distancias_kalman[1]) / 2.0
            velocidad_pid = pid_distancia.update(distancia_al_tag)
            error_distancia = distancia_al_tag - DISTANCIA_OBJETIVO
            
            if abs(error_distancia) > 200:
                if error_distancia > 0:
                    velocidad_objetivo = int(np.clip(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA))
                else:
                    velocidad_objetivo = 0
            else:
                velocidad_objetivo = 0
            
            velocidad_actual = suavizar_velocidad(velocidad_actual, velocidad_objetivo, aceleracion_maxima=ACELERACION_MAXIMA)
            velocidad_avance = int(velocidad_actual)
            
            # Enviar comandos a motores
            if velocidad_avance > 0.0:
                vel_izq, vel_der, giro_normalizado = calcular_velocidades_diferenciales(
                    v_lineal=velocidad_avance, angulo_relativo=correccion, max_v=VELOCIDAD_MAXIMA)
                pwm_manager.enviar_pwm(vel_izq, vel_der)
            else:
                pwm_manager.detener()
        else:
            pass
            
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
            
            # Cambio inmediato si corresponde
            if  es_modo_valido(modo_recibido) and modo_recibido != 2:
                
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
            
            
            # Obtener modo actual
            control_data = data_receiver.get_control_data_array()  # [BAT_TAG, MODO]
            modo_recibido = int(control_data[1])
            
            # VALIDAR EL MODO ANTES DE USARLO
            if not es_modo_valido(modo_recibido):
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
                
        else:
            print(" Esperando datos válidos...")
            
        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

if __name__ == "__main__":
    main()

