import numpy as np
from collections import deque

class MediaMovil:
    def __init__(self, n_sensores: int, ventana: int):
        """
        Inicializa la clase de media móvil para n sensores.
        n_sensores: cantidad de sensores
        ventana: tamaño de la ventana de la media móvil
        """
        self.ventana = ventana
        self.buffers = [deque(maxlen=ventana) for _ in range(n_sensores)]

    def actualizar(self, nuevos_valores: np.ndarray) -> np.ndarray:
        """
        Actualiza los buffers con nuevos valores y retorna la media móvil.
        nuevos_valores: array numpy con los valores actuales de los sensores
        Retorna: array numpy con la media móvil de cada sensor
        """
        for i in range(len(self.buffers)):
            self.buffers[i].append(nuevos_valores[i])
        return np.array([np.mean(self.buffers[i]) for i in range(len(self.buffers))])