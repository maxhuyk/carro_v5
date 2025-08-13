class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0, alpha=0.2, salida_maxima=None, integral_maxima=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.alpha = alpha
        self.error_anterior = 0.0
        self.integral = 0.0
        self.salida_maxima = salida_maxima
        self.integral_maxima = integral_maxima  # nuevo límite para la integral

    def update(self, medida_actual):
        # Error actual
        error_actual = self.setpoint - medida_actual

        # Filtro exponencial del error
        error_filtrado = self.alpha * error_actual + (1 - self.alpha) * self.error_anterior

        # Derivada
        derivada = error_filtrado - self.error_anterior

        # Acumulación de la integral
        self.integral += error_filtrado

        # Clamping de la integral
        if self.integral_maxima is not None:
            self.integral = max(min(self.integral, self.integral_maxima), -self.integral_maxima)

        # PID completo
        salida = (
            self.kp * error_filtrado +
            self.ki * self.integral +
            self.kd * derivada
        )

        # Saturación de la salida
        if self.salida_maxima is not None:
            salida = max(min(salida, self.salida_maxima), -self.salida_maxima)

        # Actualizar error anterior
        self.error_anterior = error_filtrado

        return salida









"""
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error_prev = 0.0
        self.integral = 0.0

    def update(self, valor_actual):
        error = self.setpoint - valor_actual
        self.integral += error
        derivada = error - self.error_prev
        salida = self.kp * error + self.ki * self.integral + self.kd * derivada
        self.error_prev = error
        return salida
"""
