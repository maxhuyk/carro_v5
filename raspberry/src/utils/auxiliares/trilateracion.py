import numpy as np


def trilateracion_3d(sensor_posiciones, distancias):
    P1 = np.array(sensor_posiciones[0])
    P2 = np.array(sensor_posiciones[1])
    P3 = np.array(sensor_posiciones[2])
    r1, r2, r3 = distancias

    # Vector X: de P1 a P2
    ex = (P2 - P1) / np.linalg.norm(P2 - P1)

    # Vector Y: lo definimos manualmente como hacia adelante
    ey_dir = np.array([0, 1, 0])  # Y hacia adelante en sistema global
    ey = ey_dir / np.linalg.norm(ey_dir)

    # Vector Z: lo obtenemos como ex × ey → apuntará hacia arriba
    ez = np.cross(ex, ey)

    # Ajustamos el plano de trabajo
    d = np.linalg.norm(P2 - P1)
    i = np.dot(ex, P3 - P1)
    j = np.dot(ey, P3 - P1)

    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = (r1**2 - r3**2 + i**2 + j**2) / (2 * j) - (i / j) * x
    z2 = r1**2 - x**2 - y**2

    if z2 < 0:
        z = 0.0
    else:
        z = np.sqrt(z2)

    resultado = P1 + x * ex + y * ey + z * ez
    return resultado
"""
def trilateracion_3d(sensor_posiciones, distancias):
    P1 = np.array(sensor_posiciones[0])
    P2 = np.array(sensor_posiciones[1])
    P3 = np.array(sensor_posiciones[2])
    r1, r2, r3 = distancias

    # Definición de ejes
    ex = (P2 - P1) / np.linalg.norm(P2 - P1)
    ey_dir = np.array([0, 1, 0])  # Y hacia adelante
    ey = ey_dir / np.linalg.norm(ey_dir)
    ez = np.cross(ex, ey)         # Z hacia arriba
    print("ex:", ex)
    print("ey:", ey)
    print("ez:", ez)

    # Proyecciones
    d = np.linalg.norm(P2 - P1)
    i = np.dot(ex, P3 - P1)
    j = np.dot(ey, P3 - P1)

    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = (r1**2 - r3**2 + i**2 + j**2) / (2 * j) - (i / j) * x
    z2 = r1**2 - x**2 - y**2

    if z2 < 0:
        return None  # No solución real
    else:
        z = np.sqrt(z2)

        # Punto sobre el plano (Z positivo en dirección ez)
        punto_arriba = P1 + x * ex + y * ey + z * ez

        # Punto debajo del plano (Z negativo en dirección ez)
        punto_abajo = P1 + x * ex + y * ey - z * ez

        return np.array([punto_arriba, punto_abajo])

"""

"""
import numpy as np

def trilateracion_3d(sensor_posiciones, distancias):
 
    print("000>>>>>>>>>>:", distancias)
    P1 = np.array(sensor_posiciones[0])
    P2 = np.array(sensor_posiciones[1])
    P3 = np.array(sensor_posiciones[2])
    #print("111>>>>>>>>>>:", P1, P2, P3)
    r1, r2, r3 = distancias
    #print("222>>>>>>>>>>:", r1, r2, r3, distancias)
    ex = (P2 - P1) / np.linalg.norm(P2 - P1)
    i = np.dot(ex, P3 - P1)
    ey = (P3 - P1 - i * ex) / np.linalg.norm(P3 - P1 - i * ex)
    ez = np.cross(ex, ey)
    print("ex:", ex)
    print("ey:", ey)
    print("ez:", ez)
    d = np.linalg.norm(P2 - P1)
    j = np.dot(ey, P3 - P1)

    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = (r1**2 - r3**2 + i**2 + j**2) / (2 * j) - (i / j) * x

    z2 = r1**2 - x**2 - y**2
    if z2 < 0:
        z = 0.0  # No solución real, se fuerza a cero
    else:
        z = np.sqrt(z2)

    resultado = P1 + x * ex + y * ey + z * ez
    return resultado
"""