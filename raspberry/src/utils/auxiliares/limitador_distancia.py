import numpy as np

def limitar_variacion(dist_actual, dist_anterior, max_delta=10.0):
    """
    Limita la variación entre dos medidas de distancia.
    Retorna una nueva medida ajustada si la diferencia supera el umbral.
    """
    dist_filtrada = []
    for actual, anterior in zip(dist_actual, dist_anterior):
        delta = actual - anterior
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        dist_filtrada.append(anterior + delta)
    return np.array(dist_filtrada)
