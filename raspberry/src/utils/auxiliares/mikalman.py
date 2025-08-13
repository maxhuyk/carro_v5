import numpy as np

class SimpleKalman:
    def __init__(self, n_sensores, Q=1e-2, R=1.0):
        """
        Inicializa el filtro Kalman para n sensores.
        Q: Varianza del proceso (ruido interno)
        R: Varianza de la medición (ruido externo)
        """
        self.n = n_sensores
        self.x = np.zeros(n_sensores)  # Estado estimado
        self.P = np.ones(n_sensores)   # Covarianza estimada
        self.Q = Q
        self.R = R

    def filtrar(self, z: np.ndarray) -> np.ndarray:
        """
        Aplica el filtro Kalman a las mediciones z (array numpy).
        Retorna: array numpy filtrado
        """
        for i in range(self.n):
            # Predicción
            self.P[i] += self.Q
            # Ganancia de Kalman
            K = self.P[i] / (self.P[i] + self.R)
            # Actualización
            self.x[i] = self.x[i] + K * (z[i] - self.x[i])
            self.P[i] = (1 - K) * self.P[i]
        return self.x.copy()