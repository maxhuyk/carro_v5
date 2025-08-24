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
from src.utils.controladores.pwm_manager import velocidad_a_pwm   
from src.utils.controladores.control import ControlModos
from src.utils.graficadores.distancias import graficar_distancias, graficar_distancia_sensor1
from src.utils.graficadores.angulo import graficar_angulos
from src.utils.controladores.uart_motor_controller import UARTMotorController

from time import sleep
import numpy as np
import time
import logging
from datetime import datetime
import os

class UARTController:
    def __init__(self, port):
        self.port = port  # Aquí podrías inicializar tu objeto real de comunicación

    def send(self, msg):
        print(f" UART enviado: {msg}")
        # Lógica real de envío (serial.write, etc.)

def setup_error_logger():
    """
    Configurar el logger para errores de distancias y ESP-NOW
    """
    # Crear directorio de logs si no existe
    logs_dir = "logs"
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)
    
    # Crear nombre de archivo con timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(logs_dir, f"errores_sistema_{timestamp}.log")
    
    # Configurar el logger
    logger = logging.getLogger('ErrorLogger')
    logger.setLevel(logging.INFO)
    
    # Remover handlers existentes para evitar duplicados
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # Crear handler para archivo
    file_handler = logging.FileHandler(log_filename, encoding='utf-8')
    file_handler.setLevel(logging.INFO)
    
    # Crear formato
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                 datefmt='%Y-%m-%d %H:%M:%S')
    file_handler.setFormatter(formatter)
    
    # Agregar handler al logger
    logger.addHandler(file_handler)
    
    # Log inicial
    logger.info("="*60)
    logger.info("INICIO DEL SISTEMA DE MONITOREO DE ERRORES")
    logger.info("="*60)
    logger.info(f"Archivo de log: {log_filename}")
    logger.info("Monitoreando:")
    logger.info("  - Distancias con valor -10 (error de sensor)")
    logger.info("  - Valores 99 (errores ESP-NOW)")
    logger.info("="*60)
    
    print(f" Logger configurado: {log_filename}")
    return logger

def es_modo_valido(modo):
    """
    Verificar si el modo recibido es válido
    """
    return modo in [0, 1, 2, 3]

def verificar_datos_corruptos(data_receiver):
    """
    Verificar si los datos están corruptos por errores ESP-NOW
    """
    try:
        # Obtener todos los arrays (NUEVO FORMATO SIN IMU)
        distancias = data_receiver.get_distances_array()
        power_data = data_receiver.get_power_data_array()
        anchor_status = data_receiver.get_anchor_status_array()  # Cambio de get_tag_sensors_array()
        control_data = data_receiver.get_control_data_array()
        # Para el joystick, usar el valor entero original 0-7 (o 99 si error)
        joy_value = data_receiver.get_joystick_as_int()

        # Contar valores corruptos (-10 y 99)
        valores_corruptos = 0

        # Verificar distancias -10
        valores_corruptos += sum(1 for d in distancias if d == -10)

        # Verificar valores 99 en todos los arrays
        valores_corruptos += sum(1 for val in power_data if val == 99)
        valores_corruptos += sum(1 for val in anchor_status if val == 99)
        valores_corruptos += sum(1 for val in control_data if val == 99)
        # Verificar error 99 en el valor crudo del joystick
        valores_corruptos += 1 if joy_value == 99 else 0
        valores_corruptos += sum(1 for val in distancias if val == 99)

        # Si hay más de 3 valores corruptos, considerar datos corruptos
        return valores_corruptos >= 3

    except Exception as e:
        print(f" Error verificando datos: {e}")
        return True  # En caso de error, asumir datos corruptos

def verificar_errores_datos(data_receiver, logger):
    """
    Verificar y registrar errores en los datos recibidos (FORMATO NUEVO SIN IMU)
    """
    try:
        # Verificar distancias con valor -10
        distancias = data_receiver.get_distances_array()
        for i, dist in enumerate(distancias):
            if dist == -10:
                logger.error(f"DISTANCIA_ERROR: Sensor S{i+1} reporta -10mm (error de lectura)")
                print(f" ERROR: Sensor S{i+1} con distancia -10 detectado")

        # Verificar valores 99 (ESP-NOW error) en los diferentes arrays (SIN IMU)
        power_data = data_receiver.get_power_data_array()
        anchor_status = data_receiver.get_anchor_status_array()  # Cambio de get_tag_sensors_array()
        control_data = data_receiver.get_control_data_array()
        # Usar el valor entero del joystick para detectar 99
        joy_value = data_receiver.get_joystick_as_int()

        # Verificar si hay valores 99 en arrays específicos
        if any(val == 99 for val in power_data):
            logger.error(f"ESPNOW_ERROR: Valor 99 en datos de potencia: {power_data}")
            print(" ERROR ESP-NOW: Valor 99 en datos de potencia")

        if any(val == 99 for val in control_data):
            logger.error(f"ESPNOW_ERROR: Valor 99 en datos de control: {control_data}")
            print(" ERROR ESP-NOW: Valor 99 en datos de control")

        if joy_value == 99:
            logger.error(f"ESPNOW_ERROR: Valor 99 en datos de joystick (JOY_TAG)")
            print(" ERROR ESP-NOW: Valor 99 en datos de joystick (JOY_TAG)")

        if any(val == 99 for val in anchor_status):
            logger.error(f"ESPNOW_ERROR: Valor 99 en status anchors: {anchor_status}")
            print(" ERROR ESP-NOW: Valor 99 en status anchors")

        # Verificar distancias UWB también por valores 99
        if any(val == 99 for val in distancias):
            logger.error(f"ESPNOW_ERROR: Valor 99 en distancias UWB: {distancias}")
            print(" ERROR ESP-NOW: Valor 99 en distancias UWB")

    except Exception as e:
        logger.error(f"SISTEMA_ERROR: Error verificando datos: {str(e)}")
        print(f" ERROR verificando datos: {e}")

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
    
    if error_logger:
        error_logger.info("MODO_SEGUIMIENTO: Iniciando modo seguimiento")
    
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            # VERIFICAR ERRORES EN LOS DATOS
            if error_logger:
                verificar_errores_datos(data_receiver, error_logger)
            
            # Verificar si el modo cambió - PERO SOLO SI LOS DATOS SON VÁLIDOS
            control_data = data_receiver.get_control_data_array()
            modo_recibido = int(control_data[1])
            
            # Solo cambiar de modo si los datos no están corruptos Y el modo es válido
            datos_corruptos = verificar_datos_corruptos(data_receiver)
            
            if not datos_corruptos and es_modo_valido(modo_recibido) and modo_recibido != 1:
                if error_logger:
                    error_logger.info(f"CAMBIO_MODO: Saliendo de MODO 1 hacia MODO {modo_recibido}")
                print(f" Cambiando de MODO 1 a MODO {modo_recibido}")
                return  # Salir del modo seguimiento
            elif datos_corruptos and error_logger:
                error_logger.warning("MODO_SEGUIMIENTO: Ignorando cambio de modo por datos corruptos")
                print(" [SEGUIMIENTO] Ignorando datos corruptos - continuando en modo actual")
            
            # Procesar sensores y control PID (tu código existente)
            distancias = data_receiver.get_distances_array()
            
            if distancias_previas is not None:
                distancias = limitar_variacion(distancias, distancias_previas, max_delta=MAX_DELTA)
            distancias_previas = distancias.copy()
            
            distancias_media = media_movil.actualizar(distancias)
            distancias_kalman = kalman.filtrar(distancias_media)
            
            print(f"Las distancias POR KALMAN en mm son: {distancias_kalman}")
            
            # Mostrar datos adicionales
            mostrar_datos_esp32(data_receiver)
            
            # Trilateración y control PID
            pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)
            angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
            angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual)
            angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, max_delta=MAX_DELTA)
            angulo_anterior = angulo_relativo
            
            print("ÁNGULO : ", angulo_relativo)
            
            # Control PID de ángulo
            if debe_corregir(angulo_relativo, umbral=UMBRAL):
                correccion = pid.update(angulo_relativo)
            else:
                correccion = 0
            print("PID ", correccion)
            
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
            
            print(f"Distancia al tag: {distancia_al_tag:.1f}mm, Objetivo: {DISTANCIA_OBJETIVO:.1f}mm")
            print(f"PID Distancia: {velocidad_pid:.2f} -> Vel Objetivo: {velocidad_objetivo} -> Vel Suavizada: {velocidad_avance}")
            
            # Enviar comandos a motores
            if velocidad_avance > 0.0:
                vel_izq, vel_der, giro_normalizado = calcular_velocidades_diferenciales(
                    v_lineal=velocidad_avance, angulo_relativo=correccion, max_v=VELOCIDAD_MAXIMA)
                pwm_manager.enviar_pwm(vel_izq, vel_der)
            else:
                pwm_manager.detener()
        else:
            print("Aún no hay datos válidos")
            
        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

def ejecutar_modo_pausa(data_receiver, motor_controller, pwm_manager, control_modos, error_logger=None):
    """
    MODO 2: Pausa - procesa sensores pero no mueve motores
    """
    media_movil = MediaMovil(n_sensores=3, ventana=MEDIA_MOVIL_VENTANA)
    kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)
    distancias_previas = None
    
    if error_logger:
        error_logger.info("MODO_PAUSA: Iniciando modo pausa")
    
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            # VERIFICAR ERRORES EN LOS DATOS
            if error_logger:
                verificar_errores_datos(data_receiver, error_logger)
            
            # Verificar si el modo cambió - PERO SOLO SI LOS DATOS SON VÁLIDOS
            control_data = data_receiver.get_control_data_array()
            modo_recibido = int(control_data[1])
            
            # Solo cambiar de modo si los datos no están corruptos Y el modo es válido
            datos_corruptos = verificar_datos_corruptos(data_receiver)
            
            if not datos_corruptos and es_modo_valido(modo_recibido) and modo_recibido != 2:
                if error_logger:
                    error_logger.info(f"CAMBIO_MODO: Saliendo de MODO 2 hacia MODO {modo_recibido}")
                print(f" Cambiando de MODO 2 a MODO {modo_recibido}")
                return
            elif datos_corruptos and error_logger:
                error_logger.warning("MODO_PAUSA: Ignorando cambio de modo por datos corruptos")
                print(" [PAUSA] Ignorando datos corruptos - continuando en modo actual")
            
            # Procesar sensores (mismo que seguimiento) pero SIN mover motores
            distancias = data_receiver.get_distances_array()
            
            if distancias_previas is not None:
                distancias = limitar_variacion(distancias, distancias_previas, max_delta=MAX_DELTA)
            distancias_previas = distancias.copy()
            
            distancias_media = media_movil.actualizar(distancias)
            distancias_kalman = kalman.filtrar(distancias_media)
            
            print(f"[PAUSA] Las distancias POR KALMAN en mm son: {distancias_kalman}")
            mostrar_datos_esp32(data_receiver)
            
            # Calcular pero NO enviar a motores
            pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)
            angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
            print(f"[PAUSA] ÁNGULO calculado: {angulo_actual} (motores detenidos)")
            
            # Mantener motores detenidos
            pwm_manager.detener()
            
        else:
            print("Aún no hay datos válidos")
            
        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

def ejecutar_modo_manual(data_receiver, motor_controller, pwm_manager, control_modos, error_logger=None):
    """
    MODO 3: Control manual por joystick
    """
    print(" === ENTRANDO EN MODO MANUAL ===")
    if error_logger:
        error_logger.info("MODO_MANUAL: Iniciando modo manual")
        
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            print(" Datos recibidos en modo manual")
            # VERIFICAR ERRORES EN LOS DATOS
            if error_logger:
                verificar_errores_datos(data_receiver, error_logger)
            
            # Verificar si el modo cambió - PERO SOLO SI LOS DATOS SON VÁLIDOS
            control_data = data_receiver.get_control_data_array()
            modo_recibido = int(control_data[1])
            
            print(f" DEBUG: modo_recibido = {modo_recibido}")
            
            # Solo cambiar de modo si los datos no están corruptos Y el modo es válido
            datos_corruptos = verificar_datos_corruptos(data_receiver)
            
            print(f" DEBUG: datos_corruptos = {datos_corruptos}, es_modo_valido = {es_modo_valido(modo_recibido)}")
            
            if not datos_corruptos and es_modo_valido(modo_recibido) and modo_recibido != 3:
                if error_logger:
                    error_logger.info(f"CAMBIO_MODO: Saliendo de MODO 3 hacia MODO {modo_recibido}")
                print(f" Cambiando de MODO 3 a MODO {modo_recibido}")
                return
            elif datos_corruptos and error_logger:
                error_logger.warning("MODO_MANUAL: Ignorando cambio de modo por datos corruptos")
                print(" [MANUAL] Ignorando datos corruptos - continuando en modo actual")
            
            # Obtener datos del joystick: usar SIEMPRE el valor entero crudo 0-7 del TAG
            joy_valor = data_receiver.get_joystick_as_int()
            buttons_data = data_receiver.get_buttons_data_array()  # Solo para debug visual

            print(f" DEBUG: joy_valor = {joy_valor} | buttons_data = {buttons_data}, len = {len(buttons_data) if buttons_data is not None else 'None'}")

            # Validar rango y aplicar control
            if 0 <= joy_valor <= 7:
                print(f"   JOYSTICK VALOR: {joy_valor}")
                print(f"   Mapeando valor {joy_valor} a botones...")

                # Mostrar mapeo para debug
                joy_d, joy_a, joy_i, joy_t = control_modos.mapear_joystick_valor(joy_valor)
                print(f"   Mapeo: {joy_valor} -> D:{joy_d} A:{joy_a} I:{joy_i} T:{joy_t}")

                # Calcular velocidades usando el nuevo mapeo
                vel_izq, vel_der = control_modos.control_manual_joystick_valor(joy_valor)
                print(f"   Velocidades calculadas: Izq={vel_izq} | Der={vel_der}")
            else:
                print("   Valor de joystick fuera de rango o inválido -> Detener")
                vel_izq, vel_der = 0, 0
            
            print(f"  Control Manual FINAL: Izq={vel_izq} | Der={vel_der}")
            
            # Enviar comandos a motores
            if vel_izq != 0 or vel_der != 0:
                pwm_manager.enviar_pwm(vel_izq, vel_der)
            else:
                pwm_manager.detener()
                
        else:
            print("Aún no hay datos válidos")
            
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
    error_logger = setup_error_logger()

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
        error_logger.critical(f"CONEXION_ERROR: {error_msg}")
        return
        
    if not motor_controller.connect():
        error_msg = f"Error: No se pudo conectar al controlador de motores en {DATA_PORT}"
        print(error_msg)
        error_logger.critical(f"CONEXION_ERROR: {error_msg}")
        data_receiver.disconnect()
        return
    
    print(" Sistema iniciado. Esperando datos para determinar modo...")
    error_logger.info("Sistema iniciado correctamente. Monitoreando datos...")
    
    # Loop principal de modos
    while True:
        data = data_receiver.read_data()
        
        if data is not None:
            # VERIFICAR ERRORES EN LOS DATOS RECIBIDOS
            verificar_errores_datos(data_receiver, error_logger)
            
            # VERIFICAR SI LOS DATOS ESTÁN CORRUPTOS
            datos_corruptos = verificar_datos_corruptos(data_receiver)
            
            # Obtener modo actual
            control_data = data_receiver.get_control_data_array()  # [BAT_TAG, MODO]
            modo_recibido = int(control_data[1])
            
            # VALIDAR EL MODO ANTES DE USARLO
            if datos_corruptos or not es_modo_valido(modo_recibido):
                contador_errores_consecutivos += 1
                
                if contador_errores_consecutivos >= max_errores_consecutivos:
                    # Demasiados errores: usar modo seguro (APAGADO)
                    modo_actual = 0
                    error_logger.warning(f"MODO_SEGURO: Activando modo APAGADO por {contador_errores_consecutivos} errores consecutivos")
                    print(f" MODO SEGURO: Activando APAGADO por errores de comunicación")
                else:
                    # Usar el último modo válido
                    modo_actual = ultimo_modo_valido
                    error_logger.warning(f"MODO_MANTENIDO: Usando último modo válido ({ultimo_modo_valido}) por datos corruptos. Error {contador_errores_consecutivos}/{max_errores_consecutivos}")
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
                print("ù Sistema APAGADO - Motores detenidos")
                
            elif modo_actual == 1:
                # MODO 1: SEGUIMIENTO CONTINUO
                print(" Iniciando MODO SEGUIMIENTO...")
                error_logger.info("MODO_CAMBIO: Iniciando MODO 1 - SEGUIMIENTO")
                ejecutar_modo_seguimiento(data_receiver, motor_controller, pwm_manager, control_modos, error_logger)
                
            elif modo_actual == 2:
                # MODO 2: PAUSA  
                print("ø Iniciando MODO PAUSA...")
                error_logger.info("MODO_CAMBIO: Iniciando MODO 2 - PAUSA")
                ejecutar_modo_pausa(data_receiver, motor_controller, pwm_manager, control_modos, error_logger)
                
            elif modo_actual == 3:
                # MODO 3: MANUAL
                print(" Iniciando MODO MANUAL...")
                error_logger.info("MODO_CAMBIO: Iniciando MODO 3 - MANUAL")
                ejecutar_modo_manual(data_receiver, motor_controller, pwm_manager, control_modos, error_logger)
                
        else:
            print(" Esperando datos válidos...")
            
        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

if __name__ == "__main__":
    main()
