#!/usr/bin/env python3
"""
UART Data Receiver Module
Recibe y procesa datos del ESP32 via UART como arrays numpy.
Formato de datos: [D1,D2,D3,Vbat,Currmot1,Currmot2]

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import serial
import numpy as np
from threading import Lock
from typing import Optional, Tuple


class SensorReader:
    def __init__(self, port='/dev/ttyAMA0', baudrate=2000000, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.serial_conn: Optional[serial.Serial] = None
        self.lock = Lock()
        self.buffer = bytearray()
        self.last_data = np.zeros(6, dtype=np.float64)
        self.data_valid = False

    # #####################################################
    # CONEXIÓN Y DESCONEXIÓN UART
    # #####################################################
    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            print(f" UART conectado a {self.port} @ {self.baudrate} bps")
            return self.serial_conn.is_open   #<--- RETORNA True
        except Exception as e:
            print(f" Error al conectar UART: {e}")
            return False

    def disconnect(self):
        with self.lock:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                print(" UART desconectado")
            self.serial_conn = None

    # #####################################################
    # LECTURA DE DATOS DATOS
    # #####################################################
    def read_data(self) -> Optional[np.ndarray]:
        if not (self.serial_conn and self.serial_conn.is_open):
            return None

        try:
            with self.lock:
                bytes_available = self.serial_conn.in_waiting
                if bytes_available > 0:
                    self.buffer.extend(self.serial_conn.read(bytes_available))
                    parsed = self._parse_buffer()
                    if parsed is not None:
                        self.last_data = parsed
                        self.data_valid = True

            # Siempre retornar el último dato válido si existe
            return self.last_data.copy() if self.data_valid else None

        except Exception as e:
            print(f"Error al leer datos UART: {e}")
            return None
        
    # #####################################################
    # INTERPRETA LOS DATOS - PARSEO DE BUFFER
    # #####################################################
    def _parse_buffer(self) -> Optional[np.ndarray]:
        start, end = self.buffer.find(b'['), self.buffer.find(b']')
        if start == -1 or end == -1 or end <= start:
            return None

        try:
            raw = self.buffer[start + 1:end].decode('utf-8')
            self.buffer = self.buffer[end + 1:]  # Limpiar buffer

            values = [float(v.strip()) for v in raw.split(',')]
            if len(values) != 6:
                print(f" Cantidad incorrecta de valores: {len(values)}")
                return None
            
            if any(v == -1.0 for v in values):
                print("Valor inválido (-1.0) detectado en los datos del sensor")
                return None

            return np.array(values, dtype=np.float64)

        except Exception as e:
            print(f" Error parseando datos: {e}")
            self.buffer.clear()
            return None

    # #####################################################
    # FUNCIONES AUXILIARES - NO SE USAN EN EL CÓDIGO
    # #####################################################
    def get_distances(self) -> np.ndarray:
        if self.data_valid:
            # Asegura tipo numpy y evita errores de comparación
            return np.array(self.last_data[:3] * 10.0, dtype=np.float64)
        else:
            return np.zeros(3, dtype=np.float64)

    def get_power_info(self) -> np.ndarray:
        if self.data_valid:
            return np.array(self.last_data[3:], dtype=np.float64)
        else:
            return np.zeros(3, dtype=np.float64)

    def get_full_data(self) -> np.ndarray:
        if self.data_valid:
            return np.array(self.last_data.copy(), dtype=np.float64)
        else:
            return np.zeros(6, dtype=np.float64)

    def get_connection_info(self) -> dict:
        return {
            'port': self.port,
            'baudrate': self.baudrate,
            'connected': bool(self.serial_conn and self.serial_conn.is_open),
            'data_valid': self.data_valid,
            'last_data': self.last_data.copy().tolist() if self.data_valid else None
        }