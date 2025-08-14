#!/usr/bin/env python3
"""
Módulo de Control de Modos
Maneja los diferentes modos de operación del carrito de golf
- Modo 0: APAGADO
- Modo 1: SEGUIMIENTO (PID continuo)
- Modo 2: PAUSA (sensores activos, motores parados)
- Modo 3: MANUAL (control por joystick)

NUEVO FORMATO JOYSTICK:
El joystick ahora envía un valor único (0-7) en lugar de 4 botones separados.
Mapeo de valores a combinaciones de botones (DAIT):
- 0 → (0000) - Sin movimiento
- 1 → (0100) - Solo adelante
- 2 → (0110) - Adelante + Izquierda  
- 3 → (0010) - Solo izquierda
- 4 → (0011) - Izquierda + Atrás
- 5 → (0001) - Solo atrás
- 6 → (1001) - Derecha + Atrás
- 7 → (1000) - Solo derecha

Author: Sistema UWB Carrito de Golf  
Date: August 2025
"""

import numpy as np
from typing import Tuple

class ControlModos:
    """
    Controlador de modos de operación del carrito
    """
    
    def __init__(self, velocidad_manual=50):
        """
        Inicializar el controlador de modos
        
        Args:
            velocidad_manual (int): Velocidad fija para modo manual (PWM)
        """
        self.velocidad_manual = velocidad_manual
        self.modo_actual = 0
        
    def procesar_modo(self, modo: int) -> str:
        """
        Determinar el comportamiento según el modo
        
        Args:
            modo: Modo actual (0-3)
            
        Returns:
            str: Descripción del modo activo
        """
        self.modo_actual = modo
        
        if modo == 0:
            return "APAGADO"
        elif modo == 1:
            return "SEGUIMIENTO"
        elif modo == 2:
            return "PAUSA"
        elif modo == 3:
            return "MANUAL"
        elif modo == 4:
            return "TILT"
        else:
            return "DESCONOCIDO"
    
    def mapear_joystick_valor(self, joy_valor: int) -> Tuple[int, int, int, int]:
        """
        Mapear valor de joystick (0-7) a combinaciones de botones
        
        Args:
            joy_valor: Valor del joystick (0-7)
            
        Returns:
            Tuple[int, int, int, int]: (joy_d, joy_a, joy_i, joy_t)
            
        Mapeo:
            0 → (0000) - Sin movimiento
            1 → (0100) - Solo adelante
            2 → (0110) - Adelante + Izquierda
            3 → (0010) - Solo izquierda
            4 → (0011) - Izquierda + Atrás
            5 → (0001) - Solo atrás
            6 → (1001) - Derecha + Atrás
            7 → (1000) - Solo derecha
        """
        mapeo_joystick = {
            0: (0, 0, 0, 0),  # (0000) - Sin movimiento
            1: (0, 1, 0, 0),  # (0100) - Solo adelante
            2: (0, 1, 1, 0),  # (0110) - Adelante + Izquierda  
            3: (0, 0, 1, 0),  # (0010) - Solo izquierda
            4: (0, 0, 1, 1),  # (0011) - Izquierda + Atrás
            5: (0, 0, 0, 1),  # (0001) - Solo atrás
            6: (1, 0, 0, 1),  # (1001) - Derecha + Atrás
            7: (1, 0, 0, 0),  # (1000) - Solo derecha
        }
        
        # Validar rango y proporcionar valor por defecto
        if joy_valor < 0 or joy_valor > 7:
            print(f"⚠️ Valor de joystick fuera de rango: {joy_valor}. Usando 0 (sin movimiento)")
            joy_valor = 0
            
        return mapeo_joystick.get(joy_valor, (0, 0, 0, 0))  # Por defecto: sin movimiento

    def control_manual_joystick_valor(self, joy_valor: int) -> Tuple[int, int]:
        """
        Control manual usando valor único de joystick (0-7)
        
        Args:
            joy_valor: Valor del joystick (0-7)
            
        Returns:
            Tuple[int, int]: (vel_izq, vel_der) en PWM
        """
        # Convertir valor a combinación de botones
        joy_d, joy_a, joy_i, joy_t = self.mapear_joystick_valor(joy_valor)
        
        # Usar la lógica existente de control
        return self.control_manual_joystick(joy_d, joy_a, joy_i, joy_t)

    def control_manual_joystick(self, joy_d: int, joy_a: int, joy_i: int, joy_t: int) -> Tuple[int, int]:
        """
        Control manual usando joystick con combinaciones
        
        Args:
            joy_d: Derecha (1=activo, 0=inactivo)
            joy_a: Adelante (1=activo, 0=inactivo) 
            joy_i: Izquierda (1=activo, 0=inactivo)
            joy_t: Atrás (1=activo, 0=inactivo)
            
        Returns:
            Tuple[int, int]: (vel_izq, vel_der) en PWM
        """
        vel_base = self.velocidad_manual
        vel_izq = 0
        vel_der = 0
        
        # Movimiento hacia ADELANTE
        if joy_a == 1:
            if joy_d == 1:  # Adelante + Derecha
                vel_izq = vel_base      # Rueda izquierda más rápida
                vel_der = vel_base // 2  # Rueda derecha más lenta
            elif joy_i == 1:  # Adelante + Izquierda  
                vel_izq = vel_base // 2  # Rueda izquierda más lenta
                vel_der = vel_base      # Rueda derecha más rápida
            else:  # Solo adelante
                vel_izq = vel_base
                vel_der = vel_base
                
        # Movimiento hacia ATRÁS
        elif joy_t == 1:
            if joy_d == 1:  # Atrás + Derecha
                vel_izq = -vel_base      # Rueda izquierda reversa rápida
                vel_der = -vel_base // 2  # Rueda derecha reversa lenta
            elif joy_i == 1:  # Atrás + Izquierda
                vel_izq = -vel_base // 2  # Rueda izquierda reversa lenta  
                vel_der = -vel_base      # Rueda derecha reversa rápida
            else:  # Solo atrás
                vel_izq = -vel_base
                vel_der = -vel_base
                
        # Giro en el lugar (sin avance/retroceso)
        elif joy_d == 1:  # Solo derecha
            vel_izq = vel_base // 2   # Rueda izquierda adelante
            vel_der = -vel_base // 2  # Rueda derecha atrás
        elif joy_i == 1:  # Solo izquierda
            vel_izq = -vel_base // 2  # Rueda izquierda atrás
            vel_der = vel_base // 2   # Rueda derecha adelante
            
        return (vel_izq, vel_der)
    
    def debe_mover_motores(self) -> bool:
        """
        Determinar si los motores deben moverse según el modo
        
        Returns:
            bool: True si los motores deben funcionar
        """
        return self.modo_actual in [1, 3]  # Seguimiento o Manual
    
    def debe_procesar_sensores(self) -> bool:
        """
        Determinar si se deben procesar los sensores según el modo
        
        Returns:
            bool: True si se deben procesar sensores
        """
        return self.modo_actual in [1, 2]  # Seguimiento o Pausa
    
    def es_modo_continuo(self) -> bool:
        """
        Determinar si el modo actual requiere ejecución continua
        
        Returns:
            bool: True si debe ejecutarse continuamente
        """
        return self.modo_actual != 0  # Todos excepto APAGADO
