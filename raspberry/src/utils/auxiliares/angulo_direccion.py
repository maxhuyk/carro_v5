import numpy as np

import math

def angulo_direccion_xy(pos_tag):
    """
    Calcula el ángulo entre el vector desde el origen al tag y el eje Y positivo.
    Ángulo positivo en sentido antihorario, negativo en sentido horario.

    Parameters:
        pos_tag (tuple): Coordenadas (x, y) del tag en el plano XY.

    Returns:
        float: Ángulo en grados entre -180 y +180.
    """
    x, y = pos_tag[0], pos_tag[1]

    # Vector desde el origen hacia el tag
    # Ángulo respecto al eje Y: usamos atan2 para obtener signo
    angulo_rad = math.atan2(x, y)  # Invierte los parámetros para que 0° sea en eje Y
    angulo_grados = math.degrees(angulo_rad)

    return angulo_grados

# Esto evita saltos bruscos entre ±180° y mantiene la continuidad angular
def desenrollar_angulo(previo, actual):
    delta = actual - previo
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360
    return previo + delta

#Si el tag se mueve rápidamente, incluso con desenrollado, el ángulo puede cambiar abruptamente. 
#Para evitar valores "no reales", limitamos cuán rápido puede variar el ángulo entre frames.
#Esto evita que el sistema salte de, digamos, 30° a 150°, cuando quizás solo debería haber cambiado a 50°.
def limitar_cambio(previo, actual, max_delta=20):
    delta = actual - previo
    if abs(delta) > max_delta:
        delta = max_delta if delta > 0 else -max_delta
    return previo + delta
