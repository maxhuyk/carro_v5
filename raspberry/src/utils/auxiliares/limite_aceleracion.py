

def suavizar_velocidad(velocidad_actual, velocidad_objetivo, aceleracion_maxima=15):
   
    
    # CASO 1: Arranque desde parado (0 -> velocidad)
    if velocidad_actual == 0 and velocidad_objetivo > 0:
        return min(velocidad_objetivo, aceleracion_maxima)
    
    # CASO 2: Frenado hacia parado (velocidad -> 0)
    elif velocidad_actual > 0 and velocidad_objetivo == 0:
        return max(0, velocidad_actual - aceleracion_maxima)
    
    # CASO 3: Cambios normales (velocidad -> velocidad diferente)
    # No aplicar filtro para no afectar las curvas
    else:
        return velocidad_objetivo
