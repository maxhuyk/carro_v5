import numpy as np

def trilateracion_2d(sensores, distancias):
    """
    Estima la posición en 2D (x, y) usando sensores 3D y distancias, ignorando z.

    sensores: array de forma (3, 3) con coordenadas (x, y, z)
    distancias: array de forma (3,) con distancias medidas

    Retorna: array (x, y) o None si no hay solución válida
    """
    # Solo usamos x e y
    P1, P2, P3 = sensores[:, :2]
    r1, r2, r3 = distancias

    ex = P2 - P1
    ex = ex / np.linalg.norm(ex)

    i = np.dot(ex, P3 - P1)
    ey = P3 - P1 - i * ex
    ey_norm = np.linalg.norm(ey)
    if ey_norm == 0:
        return None  # No se puede resolver: sensores colineales

    ey = ey / ey_norm
    d = np.linalg.norm(P2 - P1)
    j = np.dot(ey, P3 - P1)

    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = ((r1**2 - r3**2 + i**2 + j**2) / (2 * j)) - (i / j) * x
    z_sq = r1**2 - x**2 - y**2

    if z_sq < 0:
        return None  # No hay solución válida

    # Ignoramos z: solo usamos el plano
    punto = P1 + x * ex + y * ey
    return punto
