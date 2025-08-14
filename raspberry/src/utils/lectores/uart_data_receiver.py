#!/usr/bin/env python3
"""
UART Data Receiver Module
Modulo para recibir datos del ESP32 via UART y retornarlos como arrays numpy
FORMATO EXTENDIDO (14 valores):
    [A1,A2,A3,BAT_C,ML_C,MR_C,BAT_TAG,MODO_TAG,JOY_TAG,q_i,q_j,q_k,q_r,q_acc]

Mapeo de índices:
0-2:   Anchors UWB (A1, A2, A3) - distancias en cm, -1 si inválido
3:     Batería carro (BAT_C) - voltaje en V, -1 si inválido  
4-5:   Corrientes motores (ML_C, MR_C) - corriente en A, -1 si inválido
6:     Batería del TAG (BAT_TAG) - voltaje en V, -1 si sin datos
7:     Modo TAG (MODO_TAG) - 0-7, -1 si sin datos
8:     Joystick TAG (JOY_TAG) - 0-7, -1 si sin datos
9-12:  Quaternion del TAG (q_i, q_j, q_k, q_r) - unitario
13:    Precisión del quaternion (q_acc) - 0..3

NOTA: Los 9 primeros índices se mantienen igual para compatibilidad.

Author: Sistema UWB Carrito de Golf
Date: August 2025
"""

import serial
import time
import numpy as np
from typing import Optional
from threading import Lock

class UARTDataReceiver:
    """
    Receptor de datos UART que convierte los datos recibidos en arrays numpy
    FORMATO EXTENDIDO (14 valores): [A1,A2,A3,BAT_C,ML_C,MR_C,BAT_TAG,MODO_TAG,JOY_TAG,q_i,q_j,q_k,q_r,q_acc]
    """
    
    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 500000, timeout: float = 1.0):
        """
        Inicializar el receptor UART
        
        Args:
            port: Puerto serie (ej: '/dev/ttyAMA0', 'COM3')
            baudrate: Velocidad de comunicación
            timeout: Timeout para lectura de datos
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False
        self.lock = Lock()
        
        # Buffer para datos recibidos
        self.buffer = b''
        
    
        
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
                print(f"UART conectado a {self.port} @ {self.baudrate} bps")
                return True
            else:
                print(f"Error: No se pudo abrir el puerto {self.port}")
                return False
                
        except Exception as e:
            print(f"Error conectando UART: {e}")
            self.serial_conn = None
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Desconectar del puerto serie"""
        with self.lock:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                print("UART desconectado")
            
            self.serial_conn = None
            self.is_connected = False
    
    def read_data(self) -> Optional[np.ndarray]:
        """
        Leer datos desde UART y retornar como array numpy
        
        Returns:
            numpy array con [A1,A2,A3,BAT_C,ML_C,MR_C,BAT_TAG,MODO_TAG,JOY_TAG,q_i,q_j,q_k,q_r,q_acc] o None si no hay datos válidos
        """
        if not self.is_connected or not self.serial_conn:
            return None
            
        try:
            with self.lock:
                # Leer datos disponibles
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.buffer += data
                    
                    # Procesar buffer para encontrar mensajes completos
                    data_array = self._parse_buffer()
                    if data_array is not None:
                        self.last_data_array = data_array
                        self.data_valid = True
                        return data_array
                        
                # Retornar último dato válido si no hay nuevos datos
                if self.data_valid:
                    return self.last_data_array.copy()
                else:
                    return None
                
        except Exception as e:
            print(f"Error leyendo datos UART: {e}")
            return None
    
    def _parse_buffer(self) -> Optional[np.ndarray]:
        """
        Parsear el buffer para extraer array NUEVO CON DATOS TAG (9 valores)
        [A1,A2,A3,BAT_C,ML_C,MR_C,BAT_TAG,MODO_TAG,JOY_TAG]
        
        Returns:
            numpy array con los 9 valores o None si no hay mensaje completo válido
        """
        try:
            # Buscar inicio y fin de array
            start_marker = b'['
            end_marker = b']'
            
            start_idx = self.buffer.find(start_marker)
            if start_idx == -1:
                return None
                
            end_idx = self.buffer.find(end_marker, start_idx)
            if end_idx == -1:
                return None  # Mensaje incompleto
                
            # Extraer mensaje
            array_data = self.buffer[start_idx:end_idx+1]
            self.buffer = self.buffer[end_idx+1:]  # Remover mensaje procesado
            
            # Parsear array
            array_str = array_data.decode('utf-8')
            # Remover corchetes y dividir por comas
            values_str = array_str.strip('[]').split(',')
            
            if len(values_str) not in (9, 14):
                print(f"Error: Se esperaban 9 o 14 valores, se recibieron {len(values_str)}")
                return None
            
            # Convertir a float y crear array numpy
            values = [float(v.strip()) for v in values_str]
            arr = np.array(values, dtype=np.float64)
            if arr.size == 9:
                # Compat: expandir a 14 con quaternion por defecto
                expanded = np.zeros(14, dtype=np.float64)
                expanded[:9] = arr
                expanded[9] = 0.0  # q_i
                expanded[10] = 0.0 # q_j
                expanded[11] = 0.0 # q_k
                expanded[12] = 1.0 # q_r
                expanded[13] = 0.0 # q_acc
                return expanded
            return arr
            
        except Exception as e:
            print(f"Error parseando buffer: {e}")
            # Limpiar buffer en caso de error
            self.buffer = b''
            return None
    
    def get_distances_array(self) -> np.ndarray:
        """
        Obtener distancias UWB de ANCHORS como array numpy
        Returns: numpy array con [A1, A2, A3] en milímetros (posiciones 0-2)
        """
        if self.data_valid:
            distances_cm = self.last_data_array[:3]  # A1, A2, A3
            distances_mm = distances_cm * 10.0
            return distances_mm.copy()
        else:
            return np.zeros(3, dtype=np.float64)
    
    def get_power_data_array(self) -> np.ndarray:
        """
        Obtener datos de alimentación del carro como array numpy
        Returns: numpy array con [BAT_C, ML_C, MR_C] (posiciones 3-5)
        """
        if self.data_valid:
            return self.last_data_array[3:6].copy()
        else:
            return np.zeros(3, dtype=np.float64)
    
    def get_anchor_status_array(self) -> np.ndarray:
        """
        Obtener status de anchors UWB como array numpy (SIMULADO para compatibilidad)
        Returns: numpy array con [S1, S2, S3] - 1=OK si distancia > 0, 0=FAIL si -1
        """
        if self.data_valid:
            distances = self.last_data_array[:3]  # A1, A2, A3
            # Simular status: 1 si distancia > 0, 0 si es -1 (inválido)
            status = np.where(distances > 0, 1.0, 0.0)
            return status.copy()
        else:
            return np.zeros(3, dtype=np.float64)
    
    def get_imu_data_array(self) -> np.ndarray:
        """
        FUNCIÓN DEPRECADA: Los datos IMU fueron eliminados del firmware
        Returns: numpy array con ceros (no hay datos IMU disponibles)
        """
        print("ADVERTENCIA: get_imu_data_array() deprecada - datos IMU eliminados del firmware")
        return np.zeros(6, dtype=np.float64)  # Retorna ceros por compatibilidad
    
    def get_tag_sensors_array(self) -> np.ndarray:
        """
        FUNCIÓN DEPRECADA: Los sensores del TAG ahora están en get_anchor_status_array()
        Returns: numpy array con [S1, S2, S3] (posiciones 6-8) - redirige a anchor status
        """
        print("ADVERTENCIA: get_tag_sensors_array() deprecada - usar get_anchor_status_array()")
        return self.get_anchor_status_array()
    
    def get_control_data_array(self) -> np.ndarray:
        """
        Obtener datos de control como array numpy
        Returns: numpy array con [BAT_TAG, MODO_TAG] (posiciones 6-7)
        """
        if self.data_valid:
            return self.last_data_array[6:8].copy()
        else:
            return np.zeros(2, dtype=np.float64)
    
    def get_buttons_data_array(self) -> np.ndarray:
        """
        Obtener datos de joystick/botones como array numpy
        Returns: numpy array con [JOY_TAG] expandido a formato compatible (posición 8)
        Convierte el valor 0-7 del joystick a formato binario [D,A,I,T]
        """
        if self.data_valid:
            joy_value = int(self.last_data_array[8])
            if joy_value == -1:  # Sin datos válidos
                return np.zeros(4, dtype=np.float64)
            
            # Convertir valor 0-7 a formato binario [D,A,I,T]
            # 0=0000, 1=0001(T), 2=0010(I), 3=0011(I+T), 4=0100(A), etc.
            buttons = np.zeros(4, dtype=np.float64)
            buttons[3] = (joy_value & 1)      # Bit 0: Atrás
            buttons[2] = (joy_value >> 1) & 1 # Bit 1: Izquierda
            buttons[1] = (joy_value >> 2) & 1 # Bit 2: Adelante
            buttons[0] = (joy_value >> 3) & 1 # Bit 3: Derecha
            return buttons
        else:
            return np.zeros(4, dtype=np.float64)
    
    def get_joystick_as_int(self) -> int:
        """
        Obtener estado del joystick como int (formato original del TAG)
        Returns: int correspondiente al valor recibido del TAG (0-7)
        """
        if self.data_valid:
            joy_value = int(self.last_data_array[8])
            return joy_value if joy_value != -1 else 0
        else:
            return 0
    
    def get_joystick_as_string(self) -> str:
        """
        Obtener estado del joystick como string en formato binario
        Returns: string en formato "DAIT" (Derecha,Adelante,Izquierda,Atrás)
        """
        if self.data_valid:
            buttons = self.get_buttons_data_array().astype(int)
            return f"{buttons[0]}{buttons[1]}{buttons[2]}{buttons[3]}"
        else:
            return "0000"
    
    def get_orientation_data(self) -> np.ndarray:
        """
        FUNCIÓN DEPRECADA: Los datos de orientación IMU fueron eliminados del firmware
        Returns: numpy array con ceros (no hay datos de orientación disponibles)
        """
        print("ADVERTENCIA: get_orientation_data() deprecada - datos IMU eliminados del firmware")
        return np.zeros(3, dtype=np.float64)  # Retorna ceros por compatibilidad
    
    def get_movement_data(self) -> np.ndarray:
        """
        FUNCIÓN DEPRECADA: Los datos de movimiento IMU fueron eliminados del firmware  
        Returns: numpy array con ceros (no hay datos de movimiento disponibles)
        """
        print("ADVERTENCIA: get_movement_data() deprecada - datos IMU eliminados del firmware")
        return np.zeros(3, dtype=np.float64)  # Retorna ceros por compatibilidad
    
    def get_full_data_array(self) -> np.ndarray:
        """
        Obtener todos los datos como array numpy
        Returns: numpy array con [A1,A2,A3,BAT_C,ML_C,MR_C,BAT_TAG,MODO_TAG,JOY_TAG,q_i,q_j,q_k,q_r,q_acc]
        """
        if self.data_valid:
            return self.last_data_array.copy()
        else:
            return np.zeros(14, dtype=np.float64)
    
    def get_battery_voltage(self) -> np.ndarray:
        """
        Obtener voltaje de batería del TAG como array numpy
        Returns: numpy array con [BAT_TAG] (posición 6)
        """
        if self.data_valid:
            return np.array([self.last_data_array[6]], dtype=np.float64)  # Índice 6: BAT_TAG
        else:
            return np.zeros(1, dtype=np.float64)
    
    def get_carro_battery_voltage(self) -> np.ndarray:
        """
        Obtener voltaje de batería del CARRO como array numpy
        Returns: numpy array con [BAT_C] (posición 3)
        """
        if self.data_valid:
            return np.array([self.last_data_array[3]], dtype=np.float64)  # Índice 3: BAT_C
        else:
            return np.zeros(1, dtype=np.float64)
    
    def get_motor_currents(self) -> np.ndarray:
        """
        Obtener corrientes de motores como array numpy
        Returns: numpy array con [ML_C, MR_C] (posiciones 4-5)
        """
        if self.data_valid:
            return self.last_data_array[4:6].copy()  # Índices 4-5: ML_C, MR_C
        else:
            return np.zeros(2, dtype=np.float64)
    
    def get_modo(self) -> np.ndarray:
        """
        Obtener modo del TAG como array numpy
        Returns: numpy array con [MODO_TAG] (posición 7)
        """
        if self.data_valid:
            return np.array([self.last_data_array[7]], dtype=np.float64)  # Índice 7: MODO_TAG
        else:
            return np.zeros(1, dtype=np.float64)
    
    def is_data_valid(self) -> bool:
        """
        Verificar si los últimos datos recibidos son válidos
        
        Returns:
            bool: True si los datos son válidos
        """
        return self.data_valid and self.is_connected
    
    def get_connection_status(self) -> dict:
        """
        Obtener estado de la conexión
        Returns: Dict con información de estado y array numpy en 'last_data'
        """
        return {
            'connected': self.is_connected,
            'port': self.port,
            'baudrate': self.baudrate,
            'data_valid': self.data_valid,
            'last_data': self.last_data_array.copy() if self.data_valid else np.zeros(14, dtype=np.float64)
        }

    def get_quaternion(self) -> np.ndarray:
        """
        Obtener quaternion del TAG como array numpy
        Returns: numpy array con [q_i, q_j, q_k, q_r, q_acc]
        """
        if self.data_valid:
            return self.last_data_array[9:14].copy()
        else:
            return np.array([0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float64)

