import numpy as np

def calcular_velocidades_diferenciales(v_lineal, angulo_relativo, max_v=255):
    """
    Calcula las velocidades para las ruedas izquierda y derecha en base a
    la velocidad lineal y el ángulo de giro relativo.

    Parámetros:
    - v_lineal (int): velocidad deseada en línea recta (PWM)
    - angulo_relativo (float): ángulo entre el frente del robot y el tag (grados)
    - max_v (int): velocidad máxima (PWM)

    Retorna:
    - (int, int): velocidades para rueda izquierda y derecha
    """

    # Escalado del ángulo a [-1.0, 1.0] (control de curvatura)
    giro_normalizado = np.clip(angulo_relativo / 30.0, -1.0, 1.0)

    # Velocidades diferenciales
    vel_der = np.clip(v_lineal * (1.0 - giro_normalizado), 0, max_v)
    vel_izq = np.clip(v_lineal * (1.0 + giro_normalizado), 0, max_v)

    return int(vel_izq), int(vel_der), giro_normalizado