#!/usr/bin/env python3
"""
UART Motor Controller Module
Modulo para enviar comandos de control de motores al ESP32 via UART

Author: Sistema UWB Carrito de Golf  
Date: July 2025
"""

import serial
import time
import json
import numpy as np
from typing import Dict, Tuple, Any
from dataclasses import dataclass
from threading import Lock
from enum import Enum

class MotorDirection(Enum):
    """Direcciones de movimiento del motor"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"

@dataclass
class MotorCommand:
    """Comando de motor"""
    left_speed: int      # Velocidad motor izquierdo (-255 a 255)
    right_speed: int     # Velocidad motor derecho (-255 a 255)
    timestamp: float = 0.0
    command_id: int = 0

@dataclass
class MotorStatus:
    """Estado de los motores"""
    left_speed: int = 0      # Velocidad actual motor izquierdo
    right_speed: int = 0     # Velocidad actual motor derecho
    left_current: float = 0.0    # Corriente motor izquierdo (A)
    right_current: float = 0.0   # Corriente motor derecho (A)
    temperature: float = 0.0     # Temperatura (°C)
    last_command_time: float = 0.0

class UARTMotorController:
    """
    Controlador de motores via UART
    Envía comandos de control de motores al ESP32
    """
    
    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 2000000, timeout: float = 0.5):
        """
        Inicializar el controlador de motores
        
        Args:
            port: Puerto serie (ej: '/dev/ttyAMA0', 'COM3')
            baudrate: Velocidad de comunicación  
            timeout: Timeout para escritura de datos
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False
        self.lock = Lock()
        
        # Estado actual
        self.current_command = MotorCommand(0, 0)
        self.motor_status = MotorStatus()
        self.command_counter = 0
        
        # Límites de seguridad
        self.max_speed = 255
        self.min_speed = -255
        
        # Configuración de suavizado
        self.enable_smoothing = True
        self.max_acceleration = 50  # Cambio máximo por comando
        
    def connect(self) -> bool:
        """
        Conectar al puerto serie
        
        Returns:
            bool: True si la conexión fue exitosa
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.serial_conn.is_open:
                self.is_connected = True
                print(f"Motor Controller conectado a {self.port} @ {self.baudrate} bps")
                
                # Enviar comando inicial (parar motores)
                self.stop_motors()
                return True
            else:
                print(f"Error: No se pudo abrir el puerto {self.port}")
                return False
                
        except Exception as e:
            print(f"Error conectando Motor Controller: {e}")
            self.serial_conn = None
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Desconectar del puerto serie"""
        with self.lock:
            # Parar motores antes de desconectar
            if self.is_connected:
                self.stop_motors()
                time.sleep(0.1)  # Dar tiempo para que se envíe el comando
            
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                print("Motor Controller desconectado")
            
            self.serial_conn = None
            self.is_connected = False
    
    def _clamp_speed(self, speed: int) -> int:
        """
        Limitar velocidad a rangos seguros
        
        Args:
            speed: Velocidad a limitar
            
        Returns:
            Velocidad limitada
        """
        return max(self.min_speed, min(self.max_speed, int(speed)))
    
    def _apply_smoothing(self, target_left: int, target_right: int) -> Tuple[int, int]:
        """
        Aplicar suavizado a los comandos de motor
        
        Args:
            target_left: Velocidad objetivo motor izquierdo
            target_right: Velocidad objetivo motor derecho
            
        Returns:
            Tuple con velocidades suavizadas
        """
        if not self.enable_smoothing:
            return target_left, target_right
        
        current_left = self.current_command.left_speed
        current_right = self.current_command.right_speed
        
        # Calcular cambios
        left_change = target_left - current_left
        right_change = target_right - current_right
        
        # Limitar cambios por aceleración máxima
        if abs(left_change) > self.max_acceleration:
            left_change = self.max_acceleration if left_change > 0 else -self.max_acceleration
        
        if abs(right_change) > self.max_acceleration:
            right_change = self.max_acceleration if right_change > 0 else -self.max_acceleration
        
        # Aplicar cambios suavizados
        smooth_left = current_left + left_change
        smooth_right = current_right + right_change
        
        return int(smooth_left), int(smooth_right)
    
    def send_motor_command(self, left_speed: int, right_speed: int, smooth: bool = True) -> bool:
        """
        Enviar comando de motor
        
        Args:
            left_speed: Velocidad motor izquierdo (-255 a 255)
            right_speed: Velocidad motor derecho (-255 a 255)  
            smooth: Aplicar suavizado
            
        Returns:
            bool: True si el comando se envió exitosamente
        """
        if not self.is_connected or not self.serial_conn:
            print("Error: Motor Controller no conectado")
            return False
        
        try:
            with self.lock:
                # Limitar velocidades
                left_speed = self._clamp_speed(left_speed)
                right_speed = self._clamp_speed(right_speed)
                
                # Aplicar suavizado si está habilitado
                if smooth and self.enable_smoothing:
                    left_speed, right_speed = self._apply_smoothing(left_speed, right_speed)
                
                # Crear comando
                self.command_counter += 1
                command = MotorCommand(
                    left_speed=left_speed,
                    right_speed=right_speed,
                    timestamp=time.time(),
                    command_id=self.command_counter
                )
                
                # Crear mensaje JSON
                message = {
                    "type": "motor_command",
                    "left_speed": command.left_speed,
                    "right_speed": command.right_speed,
                    "timestamp": command.timestamp,
                    "command_id": command.command_id
                }
                
                # Enviar comando
                json_data = json.dumps(message) + '\n'
                bytes_sent = self.serial_conn.write(json_data.encode('utf-8'))
                self.serial_conn.flush()
                
                if bytes_sent and bytes_sent > 0:
                    self.current_command = command
                    self.motor_status.left_speed = left_speed
                    self.motor_status.right_speed = right_speed
                    self.motor_status.last_command_time = command.timestamp
                    
                    print(f"Comando enviado: L={left_speed}, R={right_speed}")
                    return True
                else:
                    print("Error: No se enviaron datos")
                    return False
                
        except Exception as e:
            print(f"Error enviando comando de motor: {e}")
            return False
    
    def send_motor_command_numpy(self, motor_array: np.ndarray, smooth: bool = True) -> bool:
        """
        Enviar comando de motor usando array numpy
        
        Args:
            motor_array: Array numpy con [left_speed, right_speed]
            smooth: Aplicar suavizado
            
        Returns:
            bool: True si el comando se envió exitosamente
        """
        if motor_array.size < 2:
            print("Error: Array debe tener al menos 2 elementos [left_speed, right_speed]")
            return False
        
        left_speed = int(motor_array[0])
        right_speed = int(motor_array[1])
        
        return self.send_motor_command(left_speed, right_speed, smooth)
    
    def stop_motors(self) -> bool:
        """
        Parar todos los motores inmediatamente
        
        Returns:
            bool: True si el comando se envió exitosamente
        """
        return self.send_motor_command(0, 0, smooth=False)
    
    def emergency_stop(self) -> bool:
        """
        Parada de emergencia - detiene motores inmediatamente
        
        Returns:  
            bool: True si el comando se envió exitosamente
        """
        if not self.is_connected or not self.serial_conn:
            return False
        
        try:
            with self.lock:
                # Crear mensaje de emergencia
                message = {
                    "type": "emergency_stop",
                    "timestamp": time.time(),
                    "command_id": self.command_counter + 1
                }
                
                json_data = json.dumps(message) + '\n'
                bytes_sent = self.serial_conn.write(json_data.encode('utf-8'))
                self.serial_conn.flush()
                
                if bytes_sent and bytes_sent > 0:
                    # Actualizar estado local
                    self.current_command = MotorCommand(0, 0)
                    self.motor_status.left_speed = 0
                    self.motor_status.right_speed = 0
                    self.motor_status.last_command_time = time.time()
                    
                    print("PARADA DE EMERGENCIA enviada")
                    return True
                else:
                    print("Error: No se pudo enviar parada de emergencia")
                    return False
                    
        except Exception as e:
            print(f"Error en parada de emergencia: {e}")
            return False
    
    def move_direction(self, direction: MotorDirection, speed: int = 128) -> bool:
        """
        Mover en una dirección específica
        
        Args:
            direction: Dirección de movimiento
            speed: Velocidad (0-255)
            
        Returns:
            bool: True si el comando se envió exitosamente
        """
        speed = abs(self._clamp_speed(speed))
        
        if direction == MotorDirection.FORWARD:
            return self.send_motor_command(speed, speed)
        elif direction == MotorDirection.BACKWARD:
            return self.send_motor_command(-speed, -speed)
        elif direction == MotorDirection.LEFT:
            return self.send_motor_command(-speed//2, speed//2)
        elif direction == MotorDirection.RIGHT:
            return self.send_motor_command(speed//2, -speed//2)
        elif direction == MotorDirection.STOP:
            return self.stop_motors()
        else:
            print(f"Dirección desconocida: {direction}")
            return False
    
    def move_with_steering(self, forward_speed: int, turn_rate: float) -> bool:
        """
        Mover con dirección diferencial
        
        Args:
            forward_speed: Velocidad hacia adelante (-255 a 255)
            turn_rate: Tasa de giro (-1.0 a 1.0), -1=giro izquierda completo, 1=giro derecha completo
            
        Returns:
            bool: True si el comando se envió exitosamente
        """
        forward_speed = self._clamp_speed(forward_speed)
        turn_rate = max(-1.0, min(1.0, float(turn_rate)))
        
        # Calcular velocidades diferencias
        if turn_rate >= 0:  # Giro derecha
            left_speed = forward_speed
            right_speed = int(forward_speed * (1.0 - turn_rate))
        else:  # Giro izquierda
            left_speed = int(forward_speed * (1.0 + turn_rate))
            right_speed = forward_speed
        
        return self.send_motor_command(left_speed, right_speed)
    
    def get_current_command(self) -> MotorCommand:
        """
        Obtener el comando actual de motor
        
        Returns:
            MotorCommand con el último comando enviado
        """
        return self.current_command
    
    def get_motor_status(self) -> MotorStatus:
        """
        Obtener estado actual de los motores
        
        Returns:
            MotorStatus con información de estado
        """
        return self.motor_status
    
    def get_current_speeds_array(self) -> np.ndarray:
        """
        Obtener velocidades actuales como array numpy
        
        Returns:
            numpy array con [left_speed, right_speed]
        """
        return np.array([
            self.current_command.left_speed,
            self.current_command.right_speed
        ], dtype=np.int32)
    
    def set_smoothing_parameters(self, enable: bool = True, max_acceleration: int = 50):
        """
        Configurar parámetros de suavizado
        
        Args:
            enable: Habilitar suavizado
            max_acceleration: Cambio máximo por comando
        """
        self.enable_smoothing = enable
        self.max_acceleration = max(1, min(255, int(max_acceleration)))
        print(f"Suavizado: {'Habilitado' if enable else 'Deshabilitado'}, Aceleración máxima: {self.max_acceleration}")
    
    def set_speed_limits(self, max_speed: int = 255, min_speed: int = -255):
        """
        Configurar límites de velocidad
        
        Args:
            max_speed: Velocidad máxima
            min_speed: Velocidad mínima (negativa)
        """
        self.max_speed = max(0, min(255, int(max_speed)))
        self.min_speed = max(-255, min(0, int(min_speed)))
        print(f"Límites de velocidad: {self.min_speed} a {self.max_speed}")
    
    def is_connected_and_ready(self) -> bool:
        """
        Verificar si el controlador está conectado y listo
        
        Returns:
            bool: True si está listo para enviar comandos
        """
        return self.is_connected and self.serial_conn is not None
    
    def get_connection_info(self) -> Dict[str, Any]:
        """
        Obtener información de conexión
        
        Returns:
            Dict con información de estado
        """
        return {
            'connected': self.is_connected,
            'port': self.port,
            'baudrate': self.baudrate,
            'current_left_speed': self.current_command.left_speed,
            'current_right_speed': self.current_command.right_speed,
            'last_command_time': self.motor_status.last_command_time,
            'smoothing_enabled': self.enable_smoothing,
            'max_acceleration': self.max_acceleration,
            'speed_limits': [self.min_speed, self.max_speed]
        }

# Ejemplo de uso
if __name__ == "__main__":
    # Crear controlador
    controller = UARTMotorController(port='COM3', baudrate=115200)  # Ajustar según tu configuración
    
    # Conectar
    if controller.connect():
        print("Conexión exitosa!")
        
        try:
            # Configurar suavizado
            controller.set_smoothing_parameters(enable=True, max_acceleration=30)
            
            # Pruebas de movimiento
            print("Moviendo hacia adelante...")
            controller.move_direction(MotorDirection.FORWARD, 100)
            time.sleep(2)
            
            print("Girando derecha...")
            controller.move_direction(MotorDirection.RIGHT, 80)
            time.sleep(1)
            
            print("Movimiento con dirección diferencial...")
            controller.move_with_steering(120, 0.5)  # Adelante con giro derecha
            time.sleep(2)
            
            print("Usando array numpy...")
            motor_command = np.array([80, -80])  # Giro en su lugar
            controller.send_motor_command_numpy(motor_command)
            time.sleep(1)
            
            print("Deteniendo...")
            controller.stop_motors()
            
            # Mostrar estado
            status = controller.get_connection_info()
            print(f"Estado final: {status}")
            
        except KeyboardInterrupt:
            print("Interrumpido por usuario")
            controller.emergency_stop()
        
        finally:
            controller.disconnect()
    else:
        print("No se pudo conectar al puerto serie")

