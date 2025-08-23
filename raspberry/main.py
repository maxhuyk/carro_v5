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
ACELERACION_MAXIMA
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
from src.utils.graficadores.distancias import graficar_distancias, graficar_distancia_sensor1
from src.utils.graficadores.angulo import graficar_angulos
from src.utils.controladores.uart_motor_controller import UARTMotorController

from time import sleep
import numpy as np
import time

class UARTController:
    def __init__(self, port):
        self.port = port  # Aquí podrías inicializar tu objeto real de comunicación

    def send(self, msg):
        print(f" UART enviado: {msg}")
        # Lógica real de envío (serial.write, etc.)

 
# Loop principal inicial
def main():


    data_receiver = UARTDataReceiver(port=DATA_PORT, baudrate=BAUDRATE) 
    motor_controller = UARTMotorController(port=DATA_PORT, baudrate=BAUDRATE)
    pwm_manager = PWMManager(motor_controller)
    media_movil = MediaMovil(n_sensores=3, ventana=MEDIA_MOVIL_VENTANA)
    #kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)
    kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)
    # ##### datos grafico distancias ########################
    tiempos = []
    distancias_hist = []
    distancias_media_hist = []
    distancias_kalman_hist = []
    postag_hist = []    
    start_time = time.time()
    angulos_hist = []
    tag2d = []
    ciclo = 0
    distancia_s1_hist = []

    N_CICLOS = 100  # Número de ciclos a graficar
    # ######################################################
    
    if not data_receiver.connect():
        print(f"Error: No se pudo conectar al receptor de datos en {DATA_PORT}")
        return
        
    
    if not motor_controller.connect():
        print(f" Error: No se pudo conectar al controlador de motores en {DATA_PORT}")
        data_receiver.disconnect()
        return
        
    #pid = PIDController(kp=PID_KP, ki=PID_KI, kd=PID_KD, setpoint=PID_SETPOINT)
    
    # PID para control de ángulo
    pid = PIDController(kp=1.2, ki=0.01, kd=0.3, setpoint=0.0, alpha=PID_ALPHA, salida_maxima=PID_SALIDA_MAX, integral_maxima=INTEGRAL_MAX)
    
    # PID para control de distancia (seguimiento a distancia fija)
    pid_distancia = PIDController(kp=PID_DISTANCIA_KP, ki=PID_DISTANCIA_KI, kd=PID_DISTANCIA_KD, 
                                  setpoint=DISTANCIA_OBJETIVO, alpha=PID_DISTANCIA_ALPHA, 
                                  salida_maxima=VELOCIDAD_MAXIMA, integral_maxima=PID_DISTANCIA_INTEGRAL_MAX)
    
    ciclo = 0
    distancias_previas = None
    angulo_anterior = 0  # Inicializamos la variable
    y_anterior = 0.0  # Inicialización del valor Y anterior
    velocidad_actual = 0  # Para suavizado de velocidad

    while True:
        data = data_receiver.read_data()
        # ###########################################################
        # PASO 1: Obtengo datos
        # ###########################################################
        if data is not None:
            #print("Datos completos en cm:", data, type(data))
            distancias = data_receiver.get_distances_array()
            # Paso 1.5: Limito variación entre ciclos (anti-salto)
            if distancias_previas is not None:
                distancias = limitar_variacion(distancias, distancias_previas, max_delta=MAX_DELTA)

            # Actualizo distancias_previas para el próximo ciclo
            distancias_previas = distancias.copy()
            #print(f"Las distancias ORIGINALES en mm son: {distancias}")
            
            # ###########################################################
            # PASO 2: FILTRO DE MEDIA MOVIL
            # ###########################################################
            #distancias_media = media_movil.actualizar(distancias_previas)
            
            distancias_media = media_movil.actualizar(distancias)
            #print(f"Las distancias POR MEDIA en mm son: {distancias_media}")
            
            # ###########################################################
            # PASO 3: FILTRO KALMAN
            # ###########################################################
            distancias_kalman = kalman.filtrar(distancias_media)
            print(f"Las distancias POR KALMAN en mm son: {distancias_kalman}")
            
            # ###########################################################
            # MOSTRAR DATOS ADICIONALES DEL ESP32 (FORMATO CORRECTO)
            # ###########################################################
            # Obtener datos con el mapeo correcto del formato real
            power_data = data_receiver.get_power_data_array()      # [BAT_C, ML_C, MR_C] pos 3-5
            imu_data = data_receiver.get_imu_data_array()          # [PITCH, ROLL, YAW, MOV, VEL, ACCEL_Z] pos 6-11
            tag_sensors = data_receiver.get_tag_sensors_array()    # [S1, S2, S3] pos 12-14
            control_data = data_receiver.get_control_data_array()  # [BAT_TAG, MODO] pos 15-16
            buttons_data = data_receiver.get_buttons_data_array()  # [JOY_D, JOY_A, JOY_I, JOY_T] pos 17-20
            
            print(f" CARRO: BAT_C:{power_data[0]:.2f}V | ML_C:{power_data[1]:.2f}A | MR_C:{power_data[2]:.2f}A")
            print(f" IMU: Pitch:{imu_data[0]:.1f}° | Roll:{imu_data[1]:.1f}° | Yaw:{imu_data[2]:.1f}° | MOV:{imu_data[3]:.0f} | VEL:{imu_data[4]:.2f} | ACCEL_Z:{imu_data[5]:.2f}")
            print(f" TAG: S1:{tag_sensors[0]:.1f} | S2:{tag_sensors[1]:.1f} | S3:{tag_sensors[2]:.1f} | BAT_TAG:{control_data[0]:.2f}V | MODO:{control_data[1]:.0f}")
            print(f" JOYSTICK: D:{buttons_data[0]:.0f} A:{buttons_data[1]:.0f} I:{buttons_data[2]:.0f} T:{buttons_data[3]:.0f}")
            
            # ###########################################################
            # PASO 4: Mediante trilateración obtengo la ubicación del tag
            # ###########################################################
            pos_tag3d = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias)
            pos_tag3d_media = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_media)
            pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)

            # ###########################################################
            # GUARDAR PARA GRAFICAR #####################################
            # ###########################################################
            tiempos.append(time.time() - start_time)
            distancias_hist.append(distancias_previas.copy())
            distancias_media_hist.append(distancias_media.copy())
            distancias_kalman_hist.append(distancias_kalman.copy())
            postag_hist.append(pos_tag3d_kalman[1].copy())
            posiciones_tag = np.array([[0.0, y, 0.0] for y in postag_hist])

            distancia_s1_hist.append(distancias_kalman[0])
            # ###########################################################
            # PASO 5: Calculo del ángulo a partir de trilateración
            # -180° hacia la izquierda y 180° hacia la derecha
            # ###########################################################
            angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
    
            angulo_desenrollado = desenrollar_angulo(angulo_anterior, angulo_actual) #<---- evita el salto entre -179 y 179
            angulo_relativo = limitar_cambio(angulo_anterior, angulo_desenrollado, max_delta=MAX_DELTA)  #<---- Limita los cambios bruscos
            angulo_anterior = angulo_relativo  # Actualizamos para la próxima iteración
            angulos_hist.append(angulo_actual) # Para graficar
            print("ÁNGULO : ", angulo_relativo)

            # ###########################################################
            # PASO 6: UMBRAL 
            # ###########################################################
            if debe_corregir(angulo_relativo, umbral=UMBRAL):
                # ###########################################################
                # PASO 7: CCNTROL PID 
                # ###########################################################
                correccion = pid.update(angulo_relativo)
                print("PID ", correccion)
            else:
                #print(f"Corrección ignorada: ángulo de {angulo_relativo:.2f}° está dentro del umbral")
                correccion = 0  # No se aplica corrección
                print("PID ", correccion)
            # ###########################################################
            # PASO 8  Control PID de velocidad por distancia
            # ###########################################################
            # Control progresivo de velocidad para mantener distancia objetivo
            # Usar promedio de sensores frontales (1 y 2) para mejor precisión
            distancia_al_tag = (distancias_kalman[0] + distancias_kalman[1]) / 2.0
            
            # El PID calcula cuánto debe acelerar/desacelerar para alcanzar la distancia objetivo
            velocidad_pid = pid_distancia.update(distancia_al_tag)
            
            # Lógica correcta del PID:
            # Si error > 0 (lejos): PID da salida NEGATIVA para acercarse
            # Si error < 0 (cerca): PID da salida POSITIVA para alejarse
            # Como queremos seguir al tag, invertimos la lógica:
            error_distancia = distancia_al_tag - DISTANCIA_OBJETIVO
            
            # Calcular velocidad objetivo basada en la distancia
            if abs(error_distancia) > 200:  # Fuera del rango objetivo: mover
                if error_distancia > 0:  # Lejos: acelerar
                    velocidad_objetivo = int(np.clip(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA))
                else:  # Muy cerca: retroceder (o parar si no quieres reversa)
                    velocidad_objetivo = 0  # Cambia esto si quieres que retroceda
            else:  # En rango aceptable (±200mm): PARAR
                velocidad_objetivo = 0
            
            # Aplicar suavizado de velocidad SOLO para arranque y frenado
            velocidad_actual = suavizar_velocidad(velocidad_actual, velocidad_objetivo, aceleracion_maxima=ACELERACION_MAXIMA)
            velocidad_avance = int(velocidad_actual)
            
            print(f"Distancia al tag: {distancia_al_tag:.1f}mm, Objetivo: {DISTANCIA_OBJETIVO:.1f}mm")
            print(f"PID Distancia: {velocidad_pid:.2f} -> Vel Objetivo: {velocidad_objetivo} -> Vel Suavizada: {velocidad_avance}")
            # ###########################################################
            # PASO 9  Control diferencial y pwm 
            # Un valor como: PWM:7,91 indica que la instrucción para 
            # el motor izquierdo es 7 y para el derecho es 91 con lo
            # que giraría hacia la izquierda para que el TAG regrese 
            # a ángulo cero
            # ###########################################################
            if velocidad_avance > 0.0:
                vel_izq, vel_der, giro_normalizado = calcular_velocidades_diferenciales(
                    v_lineal=velocidad_avance,
                    angulo_relativo=correccion,
                    max_v=VELOCIDAD_MAXIMA
                )

                # Las velocidades ya vienen como PWM de calcular_velocidades_diferenciales
                pwm_manager.enviar_pwm(vel_izq, vel_der)
            else:
                pwm_manager.detener()
            
        else:
            print("Aún no hay datos válidos")

        if TIEMPO_ESPERA:
            sleep(TIEMPO_ESPERA)

        ciclo += 1
        if NUM_CICLOS is not None and ciclo >= NUM_CICLOS:
            print(f"\nFinalizando ejecución tras {NUM_CICLOS} ciclos.")
            break

    # ###########################################################
    # Graficar al final
    # ###########################################################
    #graficar_distancias(tiempos, distancias_hist, distancias_kalman_hist) 
    #graficar_angulos(tiempos, angulos_hist)
    #graficar_distancia_sensor1(tiempos, distancia_s1_hist)
    
if __name__ == "__main__":
    main()
