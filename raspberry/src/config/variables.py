import numpy as np


# ## #################################################################
# CONTROL DE MOVIENTO SIN SENAL
# #################################################################
# DISTANCIA DE SEGURIDAD
DISTANCIA_FALLBACK = 2500  # Distancia de seguridad para moverse sin señal
VELOCIDAD_FALLBACK = 15  # Velocidad de seguridad para moverse sin señal
TIMEOUT_FALLBACK = 1.0  # Tiempo máximo en segundos para usar fallback

# #################################################################
# NÚMERO DE CICLOS
# ## #################################################################
# CONTROL DE ACELERACIÓN SUAVE
# #################################################################
# Límite de cambio de velocidad para arranque y frenado suaves
ACELERACION_MAXIMA = 5  # Máximo cambio de velocidad por ciclo (PWM/ciclo)

# #################################################################
# CONTROL MANUAL (MODO 3)
# #################################################################
# Velocidad fija para control manual por joystick
VELOCIDAD_MANUAL = 30  # Velocidad PWM para modo manual
NUM_CICLOS = 100

# #################################################################
# TIEMPO DE ESPERA
# #################################################################
TIEMPO_ESPERA = 0.01

# #################################################################
# COMUNICACIÓN UART
# #################################################################
DATA_PORT = '/dev/ttyAMA0'
BAUDRATE = 500000


# #################################################################
# CONFIGURACIÓN DE SENSORES
# #################################################################
N_SENSORES = 3

SENSOR_POSICIONES = [(-280, 0, 0), (280, 0, 0), (165, -230, 0)]


# #################################################################
# PASO 2:  FILTRO DE MEDIA ACUMULADA
# Para reducir el ruido de alta frecuencia y valores atípicos.
# #################################################################
# Número máximo de muestras consideradas en la media móvil (reducido para menos latencia)
MEDIA_MOVIL_VENTANA = 2

# #################################################################
# PASO 3: Filtro kalman a la salida de la media móvil
# Aplica el filtro Kalman sobre los datos suavizados para 
# permite obtener una estimación óptima y más robusta.
# Mejora la estabilidad del filtro Kalman y evita que 
# el ruido excesivo afecte su estimación.
# #################################################################
# Parámetros del filtro Kalman

KALMAN_Q = 1e-2
KALMAN_R = 0.1   #1.0
 

# #################################################################
# Determinar ángulo
# #################################################################
MAX_DELTA_POS = 100   # Delta para la posiciones iniciales antes de media

MAX_DELTA = 10       # Delta del ángulo

# #################################################################
# UMBRAL A PARTIR DEL CUAL SE CORRIGE
# #################################################################
# Este valos solo activa el PID si el cambio es significativo
# en este caso según el umbral
UMBRAL = 10

# #################################################################
# PARÁMETROS DEL CONTROLADOR PID
# #################################################################
# Este es el valor que debo enviar a los motores para corregir la velocidad. 

# Basicamente el pid permite corregir el ángulo, es decir que si 
# el ángulo es positivo, el PID envia una señal de forma de corregir
# la dirección y que el tag vuelva a estar en ángulo cero, por lo que 
# la señal en este caso tiene un valor de ángulo de corrección negativo
# en caso de que el ángulo sea negativo la idea es la misma por lo que
# el valor del PID ahora será positivo. La idea es que el TAG siempre
# se ubique a cero grado, de esta forma se alinea el carrito con el TAG.
PID_KP = 2
PID_KI = 0.05
PID_KD = 1
PID_SETPOINT = 0.0  # Generalmente 0 si se desea mantener alineación
# Límites para la corrección de giro
PID_ALPHA=0.3
PID_SALIDA_MAX = 30.0
INTEGRAL_MAX=50.0

 

# #################################################################
# CONTROL DE VELOCIDAD LINEAL
# #################################################################
# #################################################################
# ESCALONAMIENTO DE VELOCIDAD LINEAL
# #################################################################
VELOCIDADES_ESCALONADAS = {
                             #0: 10.0, <---------------QUITAR EN PRODUCCIÓN
    1: 20.0,   # Si la distancia está entre 0.5 m y 1 m
    2: 40.0,   # Entre 1 m y 2 m
    3: 60.0,   # Entre 2 m y 3 m
    4: 80.0,   # Entre 3 m y 4 m
    5: 100.0   # Entre 4 m y 5 m
}
DISTANCIA_MINIMA_PARADA = 0.9         #<------------PONER EN 0.9
# ############################################################# ############# 
##################################
# CONTROL DE VELOCIDAD LINEAL POR DISTANCIA
# #################################################################
# Control PID de seguimiento de distancia
DISTANCIA_OBJETIVO = 1500.0  # Distancia objetivo en mm (1.5 metros)

# Parámetros del PID para control de distancia (configuración rápida)
PID_DISTANCIA_KP = 0.2        # Más agresivo
PID_DISTANCIA_KI = 0.03       # Más corrección integral
PID_DISTANCIA_KD = 0.08       # Más suavizado
PID_DISTANCIA_ALPHA = 0.7     # Menos filtrado = más responsivo
PID_DISTANCIA_INTEGRAL_MAX = 200.0

# Velocidad máxima del sistema (usada también como límite del PID)
VELOCIDAD_MAXIMA = 60

# #################################################################
# CONTROL DE ACELERACIÓN SUAVE
# #################################################################
# Límite de cambio de velocidad para arranque y frenado suaves
ACELERACION_MAXIMA = 3  # Máximo cambio de velocidad por ciclo (PWM/ciclo)