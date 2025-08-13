# src/utils/controladores/pwm_manager.py
import numpy as np

class PWMManager:
    def __init__(self, uart_controller):
        self.uart = uart_controller

    def enviar_pwm(self, canal_izq, canal_der):
        self.uart.send_motor_command(canal_izq, canal_der)

    def detener(self):
        self.enviar_pwm(0, 0)


def velocidad_a_pwm(v):
    return int(np.clip((v + 1) / 2 * 255, 0, 255))
