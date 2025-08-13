def calcular_velocidad_escalonada(distancia_mm, distancia_minima_m, velocidades_por_metro, velocidad_maxima):
    distancia = distancia_mm / 1000.0  # convertir a metros

    if distancia <= distancia_minima_m:
        return 0.0

    nivel = int(distancia)
    if nivel in velocidades_por_metro:
        return velocidades_por_metro[nivel]
    else:
        return velocidad_maxima