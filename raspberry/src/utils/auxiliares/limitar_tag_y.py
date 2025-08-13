def limitar_variacion_y(valor_anterior, valor_actual, max_delta):
    """
    Limita la variación entre dos valores en Y para evitar saltos no reales.

    Parámetros:
    - valor_anterior: valor anterior en eje Y
    - valor_actual: valor actual en eje Y
    - max_delta: variación máxima permitida (LIMITAR_Y_TAG)

    Retorna:
    - valor filtrado en Y
    """
    delta = valor_actual - valor_anterior
    if abs(delta) > max_delta:
        delta = max_delta if delta > 0 else -max_delta
    return valor_anterior + delta