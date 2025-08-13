# El umbral esta por defecto en grados no radianes    
def debe_corregir(angulo, umbral=1.0):
    """
    Evalúa si el ángulo es suficientemente grande para justificar una corrección.
    
    Parámetros:
    - angulo: float, en grados
    - umbral: float, umbral mínimo (ej. 1.0°)
    
    Retorna:
    - bool: True si se debe corregir, False si se ignora
    """
    return abs(angulo) > umbral