import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import numpy as np
from datetime import datetime
import queue

# Importar módulos del sistema
from src.config.variables import *
from src.utils.lectores.uart_data_receiver import UARTDataReceiver
from src.utils.controladores.control import ControlModos
from src.utils.controladores.pwm_manager import PWMManager
from src.utils.controladores.uart_motor_controller import UARTMotorController

class CarritoGolfGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🚗 Carrito de Golf - Panel de Control v2.0")
        self.root.geometry("1400x900")
        self.root.configure(bg='#1e1e1e')
        
        # Variables de control
        self.sistema_activo = False
        self.thread_sistema = None
        self.data_queue = queue.Queue(maxsize=10)  # Limitar tamaño de cola
        
        # Datos para gráficos - listas más pequeñas
        self.tiempo_datos = []
        self.distancias_hist = {'S1': [], 'S2': [], 'S3': []}
        self.angulo_hist = []
        self.velocidad_hist = []
        
        # Componentes del sistema
        self.data_receiver = None
        self.motor_controller = None
        self.pwm_manager = None
        self.control_modos = None
        
        # Variables para evitar spam de logs
        self.last_error_time = 0
        self.update_counter = 0
        
        # Control simple de gráficos
        self.last_graphics_update = 0
        self.graphics_data_buffer = []
        self.graphics_enabled = True  # Para deshabilitar gráficos si hay problemas
        
        # Configurar estilo
        self.setup_styles()
        
        # Crear interfaz
        self.create_main_interface()
        
        # Iniciar actualización de datos con delay inicial
        self.root.after(1000, self.update_gui)  # Esperar 1s antes del primer update
        
    def setup_styles(self):
        """Configurar estilos modernos para la GUI"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Colores modernos
        style.configure('Title.TLabel', font=('Segoe UI', 16, 'bold'), 
                       background='#1e1e1e', foreground='#ffffff')
        style.configure('Header.TLabel', font=('Segoe UI', 12, 'bold'), 
                       background='#2d2d2d', foreground='#00ff88')
        style.configure('Data.TLabel', font=('Consolas', 10), 
                       background='#2d2d2d', foreground='#ffffff')
        style.configure('Status.TLabel', font=('Segoe UI', 11, 'bold'), 
                       background='#1e1e1e')
        
    def create_main_interface(self):
        """Crear la interfaz principal"""
        # Frame principal con scroll
        main_canvas = tk.Canvas(self.root, bg='#1e1e1e', highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=main_canvas.yview)
        scrollable_frame = ttk.Frame(main_canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: main_canvas.configure(scrollregion=main_canvas.bbox("all"))
        )
        
        main_canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        main_canvas.configure(yscrollcommand=scrollbar.set)
        
        # Header con título y controles
        self.create_header(scrollable_frame)
        
        # Panel principal con 3 columnas
        content_frame = ttk.Frame(scrollable_frame)
        content_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Columna 1: Estado y Control
        col1 = self.create_control_panel(content_frame)
        col1.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Columna 2: Sensores y Datos
        col2 = self.create_sensor_panel(content_frame)
        col2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Columna 3: Gráficos
        col3 = self.create_graphics_panel(content_frame)
        col3.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Pack canvas y scrollbar
        main_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
    def create_header(self, parent):
        """Crear header con título y controles principales"""
        header_frame = tk.Frame(parent, bg='#0d7377', height=80)
        header_frame.pack(fill=tk.X, padx=10, pady=10)
        header_frame.pack_propagate(False)
        
        # Título
        title_label = tk.Label(header_frame, text="🚗 CARRITO DE GOLF - SISTEMA DE CONTROL AUTÓNOMO", 
                              font=('Segoe UI', 18, 'bold'), bg='#0d7377', fg='white')
        title_label.pack(side=tk.LEFT, padx=20, pady=20)
        
        # Controles principales
        controls_frame = tk.Frame(header_frame, bg='#0d7377')
        controls_frame.pack(side=tk.RIGHT, padx=20, pady=15)
        
        self.btn_start = tk.Button(controls_frame, text="🚀 INICIAR SISTEMA", 
                                  font=('Segoe UI', 11, 'bold'), bg='#28a745', fg='white',
                                  command=self.toggle_sistema, width=15, height=2)
        self.btn_start.pack(side=tk.LEFT, padx=5)
        
        self.btn_emergency = tk.Button(controls_frame, text="🛑 EMERGENCIA", 
                                      font=('Segoe UI', 11, 'bold'), bg='#dc3545', fg='white',
                                      command=self.emergency_stop, width=15, height=2)
        self.btn_emergency.pack(side=tk.LEFT, padx=5)
        
        # Status indicator
        self.status_frame = tk.Frame(header_frame, bg='#0d7377')
        self.status_frame.pack(side=tk.RIGHT, padx=(0, 20), pady=15)
        
        self.status_indicator = tk.Label(self.status_frame, text="●", font=('Segoe UI', 20), 
                                        bg='#0d7377', fg='#dc3545')
        self.status_indicator.pack()
        
        self.status_text = tk.Label(self.status_frame, text="DESCONECTADO", 
                                   font=('Segoe UI', 9, 'bold'), bg='#0d7377', fg='white')
        self.status_text.pack()
        
    def create_control_panel(self, parent):
        """Panel de control y estado del sistema"""
        frame = tk.Frame(parent, bg='#2d2d2d', relief=tk.RAISED, bd=2)
        
        # Título del panel
        title = tk.Label(frame, text="🎮 CONTROL Y ESTADO", font=('Segoe UI', 14, 'bold'), 
                        bg='#2d2d2d', fg='#00ff88')
        title.pack(pady=10)
        
        # Estado del modo actual
        modo_frame = tk.LabelFrame(frame, text="MODO OPERACIÓN", font=('Segoe UI', 10, 'bold'),
                                  bg='#2d2d2d', fg='white', labelanchor='n')
        modo_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.modo_actual_label = tk.Label(modo_frame, text="DESCONOCIDO", 
                                         font=('Segoe UI', 16, 'bold'), bg='#2d2d2d', fg='#ffaa00')
        self.modo_actual_label.pack(pady=10)
        
        self.modo_desc_label = tk.Label(modo_frame, text="Sistema desconectado", 
                                       font=('Segoe UI', 10), bg='#2d2d2d', fg='#cccccc')
        self.modo_desc_label.pack(pady=(0, 10))
        
        # Estado de motores
        motor_frame = tk.LabelFrame(frame, text="ESTADO MOTORES", font=('Segoe UI', 10, 'bold'),
                                   bg='#2d2d2d', fg='white', labelanchor='n')
        motor_frame.pack(fill=tk.X, padx=10, pady=5)
        
        motor_info_frame = tk.Frame(motor_frame, bg='#2d2d2d')
        motor_info_frame.pack(fill=tk.X, pady=5)
        
        self.motor_izq_label = tk.Label(motor_info_frame, text="Izq: 0 PWM", 
                                       font=('Consolas', 11), bg='#2d2d2d', fg='#00aaff')
        self.motor_izq_label.pack(side=tk.LEFT, padx=20)
        
        self.motor_der_label = tk.Label(motor_info_frame, text="Der: 0 PWM", 
                                       font=('Consolas', 11), bg='#2d2d2d', fg='#00aaff')
        self.motor_der_label.pack(side=tk.RIGHT, padx=20)
        
        # Control PID
        pid_frame = tk.LabelFrame(frame, text="CONTROL PID", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='white', labelanchor='n')
        pid_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.angulo_label = tk.Label(pid_frame, text="Ángulo: 0.0°", 
                                    font=('Consolas', 11), bg='#2d2d2d', fg='#ffff00')
        self.angulo_label.pack(pady=2)
        
        self.distancia_label = tk.Label(pid_frame, text="Distancia: 0.0mm", 
                                       font=('Consolas', 11), bg='#2d2d2d', fg='#ff6600')
        self.distancia_label.pack(pady=2)
        
        self.velocidad_label = tk.Label(pid_frame, text="Velocidad: 0 PWM", 
                                       font=('Consolas', 11), bg='#2d2d2d', fg='#00ff00')
        self.velocidad_label.pack(pady=2)
        
        # Joystick (solo modo manual)
        joy_frame = tk.LabelFrame(frame, text="JOYSTICK", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='white', labelanchor='n')
        joy_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Crear pad direccional visual
        pad_frame = tk.Frame(joy_frame, bg='#2d2d2d')
        pad_frame.pack(pady=10)
        
        # Crear botones del pad direccional
        self.joy_buttons = {}
        
        # Fila superior (vacío, arriba, vacío)
        tk.Frame(pad_frame, bg='#2d2d2d', width=40, height=40).grid(row=0, column=0)
        self.joy_buttons['A'] = tk.Label(pad_frame, text="A", width=3, height=2, 
                                        font=('Segoe UI', 10, 'bold'), bg='#404040', fg='white')
        self.joy_buttons['A'].grid(row=0, column=1, padx=2, pady=2)
        tk.Frame(pad_frame, bg='#2d2d2d', width=40, height=40).grid(row=0, column=2)
        
        # Fila media (izq, centro, der)  
        self.joy_buttons['I'] = tk.Label(pad_frame, text="I", width=3, height=2,
                                        font=('Segoe UI', 10, 'bold'), bg='#404040', fg='white')
        self.joy_buttons['I'].grid(row=1, column=0, padx=2, pady=2)
        
        tk.Label(pad_frame, text="🕹️", font=('Segoe UI', 16), bg='#2d2d2d', fg='white').grid(row=1, column=1)
        
        self.joy_buttons['D'] = tk.Label(pad_frame, text="D", width=3, height=2,
                                        font=('Segoe UI', 10, 'bold'), bg='#404040', fg='white')
        self.joy_buttons['D'].grid(row=1, column=2, padx=2, pady=2)
        
        # Fila inferior (vacío, abajo, vacío)
        tk.Frame(pad_frame, bg='#2d2d2d', width=40, height=40).grid(row=2, column=0)
        self.joy_buttons['T'] = tk.Label(pad_frame, text="T", width=3, height=2,
                                        font=('Segoe UI', 10, 'bold'), bg='#404040', fg='white')
        self.joy_buttons['T'].grid(row=2, column=1, padx=2, pady=2)
        tk.Frame(pad_frame, bg='#2d2d2d', width=40, height=40).grid(row=2, column=2)
        
        # Log de eventos
        log_frame = tk.LabelFrame(frame, text="LOG DE EVENTOS", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='white', labelanchor='n')
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = tk.Text(log_frame, height=8, font=('Consolas', 9), 
                               bg='#1a1a1a', fg='#00ff00', insertbackground='white')
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.add_log("🔧 Sistema iniciado")
        self.add_log("📡 Esperando conexión...")
        
        return frame
        
    def create_sensor_panel(self, parent):
        """Panel de sensores y datos del ESP32"""
        frame = tk.Frame(parent, bg='#2d2d2d', relief=tk.RAISED, bd=2)
        
        # Título del panel
        title = tk.Label(frame, text="📊 SENSORES Y DATOS", font=('Segoe UI', 14, 'bold'), 
                        bg='#2d2d2d', fg='#00ff88')
        title.pack(pady=10)
        
        # Sensores UWB
        uwb_frame = tk.LabelFrame(frame, text="SENSORES UWB", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='white', labelanchor='n')
        uwb_frame.pack(fill=tk.X, padx=10, pady=5)
        
        sensors_frame = tk.Frame(uwb_frame, bg='#2d2d2d')
        sensors_frame.pack(fill=tk.X, pady=5)
        
        # Crear indicadores de sensores en formato circular
        for i, sensor in enumerate(['S1', 'S2', 'S3']):
            sensor_frame = tk.Frame(sensors_frame, bg='#2d2d2d')
            sensor_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
            
            # Círculo del sensor
            canvas = tk.Canvas(sensor_frame, width=60, height=60, bg='#2d2d2d', highlightthickness=0)
            canvas.pack()
            
            circle = canvas.create_oval(10, 10, 50, 50, fill='#404040', outline='#666666')
            text = canvas.create_text(30, 30, text=sensor, font=('Segoe UI', 10, 'bold'), fill='white')
            
            setattr(self, f'sensor_{sensor.lower()}_circle', circle)
            setattr(self, f'sensor_{sensor.lower()}_canvas', canvas)
            
            # Label con distancia
            label = tk.Label(sensor_frame, text="0.0 mm", font=('Consolas', 10), 
                           bg='#2d2d2d', fg='#00aaff')
            label.pack()
            setattr(self, f'sensor_{sensor.lower()}_label', label)
        
        # Datos del carrito
        carro_frame = tk.LabelFrame(frame, text="DATOS DEL CARRITO", font=('Segoe UI', 10, 'bold'),
                                   bg='#2d2d2d', fg='white', labelanchor='n')
        carro_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.bateria_carro_label = tk.Label(carro_frame, text="🔋 Batería: 0.0V", 
                                           font=('Consolas', 10), bg='#2d2d2d', fg='#00ff00')
        self.bateria_carro_label.pack(anchor='w', padx=10, pady=2)
        
        self.corriente_ml_label = tk.Label(carro_frame, text="⚡ Motor Izq: 0.0A", 
                                          font=('Consolas', 10), bg='#2d2d2d', fg='#ffaa00')
        self.corriente_ml_label.pack(anchor='w', padx=10, pady=2)
        
        self.corriente_mr_label = tk.Label(carro_frame, text="⚡ Motor Der: 0.0A", 
                                          font=('Consolas', 10), bg='#2d2d2d', fg='#ffaa00')
        self.corriente_mr_label.pack(anchor='w', padx=10, pady=2)
        
        # Datos IMU (YA NO DISPONIBLES EN FORMATO NUEVO)
        imu_frame = tk.LabelFrame(frame, text="IMU - NO DISPONIBLE", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='#666666', labelanchor='n')
        imu_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Mensaje explicativo
        no_imu_label = tk.Label(imu_frame, text="⚠️ Datos IMU eliminados del firmware para optimización UWB", 
                               font=('Segoe UI', 9, 'italic'), bg='#2d2d2d', fg='#ffaa00')
        no_imu_label.pack(pady=10)
        
        # Mantener labels para compatibilidad (pero deshabilitados)
        imu_grid = tk.Frame(imu_frame, bg='#2d2d2d')
        imu_grid.pack(fill=tk.X, pady=5)
        
        # Primera fila IMU (deshabilitada)
        imu_row1 = tk.Frame(imu_grid, bg='#2d2d2d')
        imu_row1.pack(fill=tk.X, pady=2)
        
        self.pitch_label = tk.Label(imu_row1, text="Pitch: --°", font=('Consolas', 10), 
                                   bg='#2d2d2d', fg='#666666')
        self.pitch_label.pack(side=tk.LEFT, padx=10)
        
        self.roll_label = tk.Label(imu_row1, text="Roll: --°", font=('Consolas', 10), 
                                  bg='#2d2d2d', fg='#666666')
        self.roll_label.pack(side=tk.RIGHT, padx=10)
        
        # Segunda fila IMU (deshabilitada)
        imu_row2 = tk.Frame(imu_grid, bg='#2d2d2d')
        imu_row2.pack(fill=tk.X, pady=2)
        
        self.yaw_label = tk.Label(imu_row2, text="Yaw: --°", font=('Consolas', 10), 
                                 bg='#2d2d2d', fg='#666666')
        self.yaw_label.pack(side=tk.LEFT, padx=10)
        
        self.mov_label = tk.Label(imu_row2, text="Mov: --", font=('Consolas', 10), 
                                 bg='#2d2d2d', fg='#666666')
        self.mov_label.pack(side=tk.RIGHT, padx=10)
        
        # Tercera fila IMU (deshabilitada)
        imu_row3 = tk.Frame(imu_grid, bg='#2d2d2d')
        imu_row3.pack(fill=tk.X, pady=2)
        
        self.vel_imu_label = tk.Label(imu_row3, text="Vel: --", font=('Consolas', 10), 
                                     bg='#2d2d2d', fg='#666666')
        self.vel_imu_label.pack(side=tk.LEFT, padx=10)
        
        self.accel_z_label = tk.Label(imu_row3, text="Accel Z: --", font=('Consolas', 10), 
                                     bg='#2d2d2d', fg='#666666')
        self.accel_z_label.pack(side=tk.RIGHT, padx=10)
        
        # Datos del TAG
        tag_frame = tk.LabelFrame(frame, text="DATOS DEL TAG", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='white', labelanchor='n')
        tag_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.bateria_tag_label = tk.Label(tag_frame, text="🔋 Batería TAG: 0.0V", 
                                         font=('Consolas', 10), bg='#2d2d2d', fg='#00ff00')
        self.bateria_tag_label.pack(anchor='w', padx=10, pady=5)
        
        # Estadísticas
        stats_frame = tk.LabelFrame(frame, text="ESTADÍSTICAS", font=('Segoe UI', 10, 'bold'),
                                   bg='#2d2d2d', fg='white', labelanchor='n')
        stats_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.tiempo_activo_label = tk.Label(stats_frame, text="⏱️ Tiempo activo: 00:00:00", 
                                           font=('Consolas', 10), bg='#2d2d2d', fg='#cccccc')
        self.tiempo_activo_label.pack(anchor='w', padx=10, pady=2)
        
        self.datos_recibidos_label = tk.Label(stats_frame, text="📦 Datos recibidos: 0", 
                                             font=('Consolas', 10), bg='#2d2d2d', fg='#cccccc')
        self.datos_recibidos_label.pack(anchor='w', padx=10, pady=2)
        
        self.fps_label = tk.Label(stats_frame, text="🔄 FPS: 0.0", font=('Consolas', 10), 
                                 bg='#2d2d2d', fg='#cccccc')
        self.fps_label.pack(anchor='w', padx=10, pady=2)
        
        return frame
        
    def create_graphics_panel(self, parent):
        """Panel de gráficos simple con tkinter (SIN matplotlib)"""
        frame = tk.Frame(parent, bg='#2d2d2d', relief=tk.RAISED, bd=2)
        
        # Título del panel
        title = tk.Label(frame, text="📈 DATOS TIEMPO REAL", font=('Segoe UI', 14, 'bold'), 
                        bg='#2d2d2d', fg='#00ff88')
        title.pack(pady=10)
        
        # Frame para controles de gráficos
        controls_frame = tk.Frame(frame, bg='#2d2d2d')
        controls_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Indicador de estado de gráficos
        self.graphics_status_label = tk.Label(controls_frame, text="🟢 Datos activos", 
                                            font=('Segoe UI', 9), bg='#2d2d2d', fg='#00ff88')
        self.graphics_status_label.pack(side=tk.LEFT)
        
        # Botón para reiniciar datos
        self.btn_restart_graphics = tk.Button(controls_frame, text="🔄 Limpiar", 
                                            font=('Segoe UI', 8), bg='#007acc', fg='white',
                                            command=self.restart_graphics, width=10)
        self.btn_restart_graphics.pack(side=tk.RIGHT, padx=5)
        
        # GRÁFICO 1: Distancias (Barras simples)
        dist_frame = tk.LabelFrame(frame, text="DISTANCIAS UWB", font=('Segoe UI', 10, 'bold'),
                                  bg='#2d2d2d', fg='white', labelanchor='n')
        dist_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Canvas para barras de distancias
        self.dist_canvas = tk.Canvas(dist_frame, width=300, height=120, bg='#1a1a1a', highlightthickness=1)
        self.dist_canvas.pack(pady=10)
        
        # GRÁFICO 2: Ángulo (Indicador circular)
        angle_frame = tk.LabelFrame(frame, text="ÁNGULO DIRECCIÓN", font=('Segoe UI', 10, 'bold'),
                                   bg='#2d2d2d', fg='white', labelanchor='n')
        angle_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Canvas para ángulo
        self.angle_canvas = tk.Canvas(angle_frame, width=150, height=150, bg='#1a1a1a', highlightthickness=1)
        self.angle_canvas.pack(pady=10)
        
        # Label para valor del ángulo
        self.angle_value_label = tk.Label(angle_frame, text="0.0°", font=('Consolas', 14, 'bold'),
                                         bg='#2d2d2d', fg='#ffaa00')
        self.angle_value_label.pack()
        
        # GRÁFICO 3: Velocidades (Barras horizontales)
        vel_frame = tk.LabelFrame(frame, text="VELOCIDADES MOTORES", font=('Segoe UI', 10, 'bold'),
                                 bg='#2d2d2d', fg='white', labelanchor='n')
        vel_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Canvas para velocidades
        self.vel_canvas = tk.Canvas(vel_frame, width=250, height=100, bg='#1a1a1a', highlightthickness=1)
        self.vel_canvas.pack(pady=10)
        
        # Labels para valores de velocidad
        vel_labels_frame = tk.Frame(vel_frame, bg='#2d2d2d')
        vel_labels_frame.pack(fill=tk.X, pady=5)
        
        self.vel_izq_value = tk.Label(vel_labels_frame, text="Izq: 0", font=('Consolas', 10),
                                     bg='#2d2d2d', fg='#00aaff')
        self.vel_izq_value.pack(side=tk.LEFT, padx=20)
        
        self.vel_der_value = tk.Label(vel_labels_frame, text="Der: 0", font=('Consolas', 10),
                                     bg='#2d2d2d', fg='#00aaff')
        self.vel_der_value.pack(side=tk.RIGHT, padx=20)
        
        # Datos históricos simples (solo últimos valores)
        data_frame = tk.LabelFrame(frame, text="DATOS ACTUALES", font=('Segoe UI', 10, 'bold'),
                                  bg='#2d2d2d', fg='white', labelanchor='n')
        data_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.current_data_text = tk.Text(data_frame, height=6, font=('Consolas', 9), 
                                        bg='#1a1a1a', fg='#00ff88', insertbackground='white')
        data_scroll = ttk.Scrollbar(data_frame, orient="vertical", command=self.current_data_text.yview)
        self.current_data_text.configure(yscrollcommand=data_scroll.set)
        
        self.current_data_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        data_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Inicializar gráficos
        self.init_simple_graphics()
        
        return frame
        
    def toggle_sistema(self):
        """Iniciar o detener el sistema"""
        if not self.sistema_activo:
            if self.iniciar_sistema():
                self.sistema_activo = True
                self.btn_start.config(text="⏹️ DETENER SISTEMA", bg='#dc3545')
                self.status_indicator.config(fg='#28a745')
                self.status_text.config(text="CONECTADO")
                self.add_log("🚀 Sistema iniciado exitosamente")
        else:
            self.detener_sistema()
            self.sistema_activo = False
            self.btn_start.config(text="🚀 INICIAR SISTEMA", bg='#28a745')
            self.status_indicator.config(fg='#dc3545')
            self.status_text.config(text="DESCONECTADO")
            self.add_log("⏹️ Sistema detenido")
            
    def iniciar_sistema(self):
        """Inicializar componentes del sistema"""
        try:
            # Inicializar componentes exactamente como main.py
            self.data_receiver = UARTDataReceiver(port=DATA_PORT, baudrate=BAUDRATE)
            self.motor_controller = UARTMotorController(port=DATA_PORT, baudrate=BAUDRATE)
            self.pwm_manager = PWMManager(self.motor_controller)
            self.control_modos = ControlModos(velocidad_manual=VELOCIDAD_MANUAL)
            
            # Conectar ambos por separado (como en main.py)
            if not self.data_receiver.connect():
                raise Exception(f"No se pudo conectar al puerto {DATA_PORT}")
            
            if not self.motor_controller.connect():
                raise Exception("No se pudo conectar al controlador de motores")
            
            self.add_log(f"📡 Data Receiver conectado a {DATA_PORT} @ {BAUDRATE} bps")
            self.add_log(f"� Motor Controller conectado a {DATA_PORT} @ {BAUDRATE} bps")
            
            # Inicializar filtros y controladores PID
            from src.utils.auxiliares.mediam import MediaMovil
            from src.utils.auxiliares.mikalman import SimpleKalman
            from src.utils.controladores.pid_controller import PIDController
            
            self.media_movil = MediaMovil(n_sensores=3, ventana=MEDIA_MOVIL_VENTANA)
            self.kalman = SimpleKalman(n_sensores=3, Q=KALMAN_Q, R=KALMAN_R)
            
            # PID para control de ángulo (alineado con main.py)
            self.pid_angulo = PIDController(kp=1.2, ki=0.01, kd=0.3, setpoint=PID_SETPOINT, 
                                          alpha=PID_ALPHA, salida_maxima=PID_SALIDA_MAX, integral_maxima=INTEGRAL_MAX)
            
            # PID para control de distancia  
            self.pid_distancia = PIDController(kp=PID_DISTANCIA_KP, ki=PID_DISTANCIA_KI, kd=PID_DISTANCIA_KD, 
                                             setpoint=DISTANCIA_OBJETIVO, alpha=PID_DISTANCIA_ALPHA, 
                                             salida_maxima=VELOCIDAD_MAXIMA, integral_maxima=PID_DISTANCIA_INTEGRAL_MAX)
            
            # Variables de estado
            self.distancias_previas = None
            self.angulo_anterior = 0
            self.velocidad_actual = 0
            # Variables persistentes para seguimiento
            self.angulo_calculado = 0
            self.distancia_promedio = 0
            
            # Iniciar thread del sistema
            self.thread_sistema = threading.Thread(target=self.sistema_worker, daemon=True)
            self.thread_sistema.start()
            
            return True
            
        except Exception as e:
            messagebox.showerror("Error de Conexión", f"No se pudo iniciar el sistema:\n{str(e)}")
            self.add_log(f"❌ Error: {str(e)}")
            return False
            
    def detener_sistema(self):
        """Detener el sistema de forma segura"""
        self.sistema_activo = False
        
        if self.pwm_manager:
            try:
                self.pwm_manager.detener()
            except:
                pass
            
        if self.data_receiver:
            try:
                self.data_receiver.disconnect()
            except:
                pass
            
        if self.motor_controller:
            try:
                self.motor_controller.disconnect()
            except:
                pass
            
    def emergency_stop(self):
        """Parada de emergencia"""
        self.detener_sistema()
        self.sistema_activo = False
        self.btn_start.config(text="🚀 INICIAR SISTEMA", bg='#28a745')
        self.status_indicator.config(fg='#dc3545')
        self.status_text.config(text="EMERGENCIA")
        self.add_log("🚨 PARADA DE EMERGENCIA ACTIVADA")
        messagebox.showwarning("Parada de Emergencia", "Sistema detenido por emergencia")
        
    def init_simple_graphics(self):
        """Inicializar los gráficos simples de tkinter"""
        try:
            # Dibujar gráfico inicial de distancias
            self.draw_distance_bars([0, 0, 0])
            
            # Dibujar indicador de ángulo inicial
            self.draw_angle_indicator(0)
            
            # Dibujar barras de velocidad inicial
            self.draw_velocity_bars(0, 0)
            
            print("✅ Gráficos simples inicializados")
            
        except Exception as e:
            print(f"⚠️ Error inicializando gráficos simples: {e}")
            
    def draw_distance_bars(self, distances):
        """Dibujar barras de distancia"""
        try:
            self.dist_canvas.delete("all")
            
            # Configuración
            width = 280
            height = 100
            bar_width = 60
            max_dist = 5000  # Máximo 5m
            
            colors = ['#ff6b6b', '#4ecdc4', '#45b7d1']
            labels = ['S1', 'S2', 'S3']
            
            for i, (dist, color, label) in enumerate(zip(distances, colors, labels)):
                x = 20 + i * 90
                
                # Limitar distancia para la barra
                normalized_dist = min(dist, max_dist) / max_dist
                bar_height = int(normalized_dist * 80)
                
                # Dibujar barra
                self.dist_canvas.create_rectangle(x, height - bar_height, x + bar_width, height,
                                                 fill=color, outline='white', width=1)
                
                # Etiqueta del sensor
                self.dist_canvas.create_text(x + bar_width//2, height + 15, text=label,
                                           fill='white', font=('Arial', 10, 'bold'))
                
                # Valor de distancia
                self.dist_canvas.create_text(x + bar_width//2, height - bar_height - 10,
                                           text=f"{dist:.0f}", fill=color, font=('Arial', 8))
                
        except Exception as e:
            print(f"Error dibujando barras: {e}")
            
    def draw_angle_indicator(self, angle):
        """Dibujar indicador circular de ángulo"""
        try:
            self.angle_canvas.delete("all")
            
            # Centro y radio
            cx, cy = 75, 75
            radius = 60
            
            # Círculo base
            self.angle_canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius,
                                         outline='white', width=2, fill='#1a1a1a')
            
            # Línea central (0°)
            self.angle_canvas.create_line(cx, cy-radius, cx, cy-radius+10,
                                         fill='#666666', width=2)
            
            # Calcular posición de la aguja
            angle_rad = np.radians(-angle)  # Negativo para sentido horario
            end_x = cx + radius * 0.8 * np.sin(angle_rad)
            end_y = cy - radius * 0.8 * np.cos(angle_rad)
            
            # Dibujar aguja
            self.angle_canvas.create_line(cx, cy, end_x, end_y,
                                         fill='#ffaa00', width=4, capstyle=tk.ROUND)
            
            # Punto central
            self.angle_canvas.create_oval(cx-5, cy-5, cx+5, cy+5,
                                         fill='#ffaa00', outline='white')
            
            # Actualizar label
            self.angle_value_label.config(text=f"{angle:.1f}°")
            
        except Exception as e:
            print(f"Error dibujando ángulo: {e}")
            
    def draw_velocity_bars(self, vel_left, vel_right):
        """Dibujar barras horizontales de velocidad"""
        try:
            self.vel_canvas.delete("all")
            
            # Configuración
            width = 230
            height = 80
            max_vel = 255
            
            # Barra izquierda
            left_width = int(abs(vel_left) / max_vel * width * 0.4)
            color_left = '#00aaff' if vel_left >= 0 else '#ff4444'
            
            self.vel_canvas.create_rectangle(width//2 - left_width, 20, width//2, 40,
                                           fill=color_left, outline='white')
            
            # Barra derecha
            right_width = int(abs(vel_right) / max_vel * width * 0.4)
            color_right = '#00aaff' if vel_right >= 0 else '#ff4444'
            
            self.vel_canvas.create_rectangle(width//2, 50, width//2 + right_width, 70,
                                           fill=color_right, outline='white')
            
            # Labels
            self.vel_canvas.create_text(width//2 - 10, 10, text="IZQ", fill='white', font=('Arial', 9))
            self.vel_canvas.create_text(width//2 + 10, 80, text="DER", fill='white', font=('Arial', 9))
            
            # Línea central
            self.vel_canvas.create_line(width//2, 0, width//2, height, fill='#666666', width=1)
            
            # Actualizar labels
            self.vel_izq_value.config(text=f"Izq: {vel_left}")
            self.vel_der_value.config(text=f"Der: {vel_right}")
            
        except Exception as e:
            print(f"Error dibujando velocidades: {e}")
        
    def sistema_worker(self):
        """Thread worker con lógica de control RESTAURADA pero OPTIMIZADA"""
        from src.utils.auxiliares.trilateracion import trilateracion_3d
        from src.utils.auxiliares.angulo_direccion import angulo_direccion_xy, desenrollar_angulo, limitar_cambio
        from src.utils.auxiliares.limitador_distancia import limitar_variacion
        from src.utils.auxiliares.control_diferencial import calcular_velocidades_diferenciales
        from src.utils.auxiliares.validador import debe_corregir
        from src.utils.auxiliares.limite_aceleracion import suavizar_velocidad
        import numpy as np
        
        start_time = time.time()
        datos_count = 0
        last_log_time = 0
        last_control_time = 0
        
        while self.sistema_activo:
            try:
                current_time = time.time()  # Definir current_time al inicio del bucle
                
                if self.data_receiver:
                    data = self.data_receiver.read_data()
                    
                    if data is not None:
                        datos_count += 1
                        
                        # Obtener datos básicos (FORMATO NUEVO SIN IMU)
                        distancias = self.data_receiver.get_distances_array()
                        power_data = self.data_receiver.get_power_data_array()
                        anchor_status = self.data_receiver.get_anchor_status_array()  # Cambio de get_tag_sensors_array()
                        control_data = self.data_receiver.get_control_data_array()
                        # Joystick: usar bits para debug y entero 0-7 para control
                        buttons_data = self.data_receiver.get_buttons_data_array()  # [D,A,I,T]
                        joy_valor = self.data_receiver.get_joystick_as_int()       # 0-7
                        
                        # Obtener modo actual
                        modo_actual = int(control_data[1]) if len(control_data) > 1 else 0
                        
                        # Variables para los cálculos PID (usar últimos valores persistentes)
                        angulo_calculado = self.angulo_calculado
                        distancia_promedio = self.distancia_promedio
                        velocidad_pid = 0
                        correccion_angulo = 0
                        vel_izq = 0
                        vel_der = 0
                        
                        # PROCESAMIENTO DE SENSORES (solo cada 3 ciclos para reducir carga)
                        if datos_count % 3 == 0:
                            # Validar y filtrar distancias
                            if self.distancias_previas is not None:
                                distancias = limitar_variacion(distancias, self.distancias_previas, max_delta=MAX_DELTA)
                            self.distancias_previas = distancias.copy()
                            
                            distancias_media = self.media_movil.actualizar(distancias)
                            distancias_kalman = self.kalman.filtrar(distancias_media)
                            
                            # Trilateración y cálculo de ángulo (optimizado)
                            try:
                                pos_tag3d_kalman = trilateracion_3d(np.array(SENSOR_POSICIONES), distancias_kalman)
                                angulo_actual = angulo_direccion_xy(pos_tag3d_kalman)
                                angulo_desenrollado = desenrollar_angulo(self.angulo_anterior, angulo_actual)
                                angulo_calculado = limitar_cambio(self.angulo_anterior, angulo_desenrollado, max_delta=MAX_DELTA)
                                self.angulo_anterior = angulo_calculado

                                # Distancia promedio para PID
                                distancia_promedio = (distancias_kalman[0] + distancias_kalman[1]) / 2.0
                            except Exception as e:
                                # Si falla el cálculo, usar valores anteriores
                                angulo_calculado = self.angulo_anterior
                                distancia_promedio = distancia_promedio if distancia_promedio else (np.mean(distancias) if len(distancias) > 0 else 0)

                            # Guardar valores calculados para próximos ciclos
                            self.angulo_calculado = angulo_calculado
                            self.distancia_promedio = distancia_promedio
                        
                        # CONTROL DE MOTORES (solo cada 2 ciclos y según el modo)
                        if current_time - last_control_time > 0.05:  # Control a 20Hz máximo
                            last_control_time = current_time
                            
                            if modo_actual == 0:
                                # MODO 0: APAGADO
                                if self.pwm_manager:
                                    self.pwm_manager.detener()
                                if current_time - last_log_time > 10:  # Log cada 10s
                                    self.add_log("⏹️ Modo APAGADO")
                                    last_log_time = current_time
                                    
                            elif modo_actual == 1:
                                # MODO 1: SEGUIMIENTO AUTOMÁTICO
                                # Usar últimos cálculos persistentes aunque el ángulo sea 0 (avance recto)
                                if self.distancias_previas is not None:
                                    # Control PID de ángulo
                                    if debe_corregir(angulo_calculado, umbral=UMBRAL):
                                        correccion_angulo = self.pid_angulo.update(angulo_calculado)
                                    else:
                                        correccion_angulo = 0
                                        
                                    # Control PID de distancia
                                    velocidad_pid = self.pid_distancia.update(distancia_promedio)
                                    error_distancia = distancia_promedio - DISTANCIA_OBJETIVO
                                    
                                    if abs(error_distancia) > 200:
                                        if error_distancia > 0:
                                            velocidad_objetivo = int(np.clip(abs(velocidad_pid), 10, VELOCIDAD_MAXIMA))
                                        else:
                                            velocidad_objetivo = 0
                                    else:
                                        velocidad_objetivo = 0
                                        
                                    # Suavizar velocidad
                                    self.velocidad_actual = suavizar_velocidad(self.velocidad_actual, velocidad_objetivo, 
                                                                             aceleracion_maxima=ACELERACION_MAXIMA)
                                    velocidad_avance = int(self.velocidad_actual)
                                    
                                    # Calcular velocidades diferenciales y enviar
                                    if velocidad_avance > 0:
                                        vel_izq, vel_der, _ = calcular_velocidades_diferenciales(
                                            v_lineal=velocidad_avance, angulo_relativo=correccion_angulo, max_v=VELOCIDAD_MAXIMA)
                                        if self.pwm_manager:
                                            self.pwm_manager.enviar_pwm(vel_izq, vel_der)
                                        if current_time - last_log_time > 3:  # Log cada 3s
                                            self.add_log(f"🎯 Seguimiento: L={vel_izq} R={vel_der} A={angulo_calculado:.1f}° D={distancia_promedio:.0f}mm VPID={velocidad_pid:.1f} Vobj={velocidad_objetivo}")
                                            last_log_time = current_time
                                    else:
                                        if self.pwm_manager:
                                            self.pwm_manager.detener()
                                            
                            elif modo_actual == 2:
                                # MODO 2: PAUSA
                                if self.pwm_manager:
                                    self.pwm_manager.detener()
                                if current_time - last_log_time > 5:  # Log cada 5s
                                    self.add_log(f"⏸️ PAUSA - A:{angulo_calculado:.1f}° D:{distancia_promedio:.0f}mm")
                                    last_log_time = current_time
                                    
                            elif modo_actual == 3:
                                # MODO 3: CONTROL MANUAL - usar valor entero 0-7 del TAG
                                if 0 <= joy_valor <= 7 and self.control_modos:
                                    vel_izq, vel_der = self.control_modos.control_manual_joystick_valor(joy_valor)

                                    if vel_izq != 0 or vel_der != 0:
                                        if self.pwm_manager:
                                            self.pwm_manager.enviar_pwm(vel_izq, vel_der)
                                        if current_time - last_log_time > 2:  # Log cada 2s
                                            self.add_log(f"🕹️ Manual: L={vel_izq} R={vel_der} JOY={joy_valor}")
                                            last_log_time = current_time
                                    else:
                                        if self.pwm_manager:
                                            self.pwm_manager.detener()
                                else:
                                    # Sin datos de joystick válidos
                                    if self.pwm_manager:
                                        self.pwm_manager.detener()
                        
                        # Crear paquete de datos para la GUI (FORMATO NUEVO SIN IMU)
                        data_packet = {
                            'timestamp': current_time - start_time,
                            'distancias': distancias,
                            'power_data': power_data,
                            'anchor_status': anchor_status,  # Cambio de imu_data
                            'control_data': control_data,
                            'buttons_data': buttons_data,
                            'joy_valor': joy_valor,
                            'datos_count': datos_count,
                            'angulo_calculado': angulo_calculado,
                            'distancia_promedio': distancia_promedio,
                            'velocidad_pid': velocidad_pid,
                            'correccion_angulo': correccion_angulo,
                            'vel_izq': vel_izq,
                            'vel_der': vel_der
                        }
                        
                        # Enviar a la GUI de forma no bloqueante
                        try:
                            self.data_queue.put_nowait(data_packet)
                        except queue.Full:
                            # Limpiar cola y poner dato más reciente
                            while not self.data_queue.empty():
                                try:
                                    self.data_queue.get_nowait()
                                except queue.Empty:
                                    break
                            self.data_queue.put_nowait(data_packet)
                    
                    # Log de espera solo esporádico
                    elif current_time - last_log_time > 8:
                        self.add_log("⚠️ Esperando datos ESP32...")
                        last_log_time = current_time
                        
                    time.sleep(0.02)  # 50Hz para lectura de datos
                    
            except Exception as e:
                current_time = time.time()  # Definir current_time en caso de error
                if current_time - last_log_time > 3:  # Error log cada 3s máximo
                    self.add_log(f"❌ Error worker: {str(e)}")
                    last_log_time = current_time
                time.sleep(0.1)
                
    def update_gui(self):
        """Actualizar la GUI con los datos recibidos - OPTIMIZADO"""
        updates_processed = 0
        max_updates_per_cycle = 3  # Limitar actualizaciones por ciclo
        
        try:
            # Procesar solo algunos datos por ciclo para no sobrecargar
            while not self.data_queue.empty() and updates_processed < max_updates_per_cycle:
                data_packet = self.data_queue.get_nowait()
                
                if 'error' in data_packet:
                    self.add_log(f"❌ Error: {data_packet['error']}")
                    updates_processed += 1
                    continue
                    
                # Actualizar displays de sensores (más liviano)
                self.update_sensor_displays(data_packet)
                
                # Actualizar gráficos simples cada 10 actualizaciones
                if updates_processed % 10 == 0:
                    current_time = time.time()
                    if current_time - self.last_graphics_update > 1.0:  # Solo cada segundo
                        self.update_simple_graphics(data_packet)
                
                # Actualizar estadísticas
                self.update_statistics(data_packet)
                
                updates_processed += 1
                
        except queue.Empty:
            pass
        except Exception as e:
            print(f"Error actualizando GUI: {e}")
            
        # Programar próxima actualización - más lento para estabilidad
        self.root.after(100, self.update_gui)  # 100ms = 10 FPS para estabilidad

    def update_simple_graphics(self, data_packet):
        """Actualizar gráficos simples de tkinter"""
        try:
            if not self.graphics_enabled:
                return
                
            # Actualizar distancias
            if 'distancias' in data_packet and len(data_packet['distancias']) >= 3:
                self.draw_distance_bars(data_packet['distancias'][:3])
            
            # Actualizar ángulo
            if 'angulo_calculado' in data_packet:
                self.draw_angle_indicator(data_packet['angulo_calculado'])
            
            # Actualizar velocidades
            if 'vel_izq' in data_packet and 'vel_der' in data_packet:
                self.draw_velocity_bars(data_packet['vel_izq'], data_packet['vel_der'])
            
            # Actualizar datos actuales en texto
            self.update_current_data_display(data_packet)
            self.last_graphics_update = time.time()
            
        except Exception as e:
            print(f"Error actualizando gráficos simples: {e}")
            
    def update_current_data_display(self, data):
        """Actualizar display de datos actuales"""
        try:
            self.current_data_text.delete('1.0', tk.END)
            
            # Formatear datos actuales
            lines = []
            if 'timestamp' in data:
                lines.append(f"Tiempo: {data['timestamp']:.1f}s")
            
            if 'distancias' in data and len(data['distancias']) >= 3:
                lines.append(f"Distancias:")
                for i, dist in enumerate(data['distancias'][:3]):
                    lines.append(f"  S{i+1}: {dist:.1f}mm")
            
            if 'angulo_calculado' in data:
                lines.append(f"Ángulo: {data['angulo_calculado']:.1f}°")
            
            if 'distancia_promedio' in data:
                lines.append(f"Dist. Promedio: {data['distancia_promedio']:.0f}mm")
            
            if 'vel_izq' in data and 'vel_der' in data:
                lines.append(f"Velocidades:")
                lines.append(f"  Izq: {data['vel_izq']} PWM")
                lines.append(f"  Der: {data['vel_der']} PWM")
            
            if 'control_data' in data and len(data['control_data']) >= 2:
                modo = int(data['control_data'][1])
                modos = {0: "APAGADO", 1: "SEGUIMIENTO", 2: "PAUSA", 3: "MANUAL"}
                lines.append(f"Modo: {modos.get(modo, 'DESCONOCIDO')}")
                
            # Mostrar joystick en nuevo formato (usar joy_valor si está)
            if 'joy_valor' in data:
                lines.append(f"Joystick: {int(data['joy_valor'])} (0-7)")
            
            self.current_data_text.insert('1.0', '\n'.join(lines))
            
        except Exception as e:
            print(f"Error actualizando datos actuales: {e}")

    def restart_graphics(self):
        """Reiniciar el sistema de gráficos simples"""
        try:
            print("🔄 Reiniciando gráficos simples...")
            self.graphics_enabled = True
            
            # Limpiar y reinicializar gráficos
            self.init_simple_graphics()
            
            self.add_log("🔄 Gráficos reiniciados exitosamente")
            print("✅ Gráficos simples reiniciados")
                
            self.update_graphics_status()
            
        except Exception as e:
            print(f"❌ Error en restart_graphics: {e}")
            self.graphics_enabled = False
            self.update_graphics_status()
            
    def update_graphics_status(self):
        """Actualizar indicador de estado de gráficos"""
        try:
            if self.graphics_enabled:
                self.graphics_status_label.config(text="🟢 Datos activos", fg='#00ff88')
            else:
                self.graphics_status_label.config(text="🔴 Datos deshabilitados", fg='#ff4444')
        except Exception:
            pass

    def update_sensor_displays(self, data):
        """Actualizar displays de sensores y datos - OPTIMIZADO"""
        try:
            # Sensores UWB - con validación
            if 'distancias' in data and len(data['distancias']) >= 3:
                distancias = data['distancias']
                for i, dist in enumerate(distancias[:3]):  # Solo primeros 3
                    sensor_name = f"s{i+1}"
                    try:
                        label = getattr(self, f'sensor_{sensor_name}_label')
                        canvas = getattr(self, f'sensor_{sensor_name}_canvas')
                        circle = getattr(self, f'sensor_{sensor_name}_circle')
                        
                        label.config(text=f"{dist:.1f} mm")
                        
                        # Cambiar color según distancia
                        if dist < 1000:
                            color = '#ff4444'
                        elif dist < 2000:
                            color = '#ffaa00'
                        elif dist < 3000:
                            color = '#00ff00'
                        else:
                            color = '#0088ff'
                            
                        canvas.itemconfig(circle, fill=color)
                    except (AttributeError, IndexError):
                        continue
            
            # Datos del carrito - con validación
            if 'power_data' in data and len(data['power_data']) >= 3:
                power_data = data['power_data']
                self.bateria_carro_label.config(text=f"🔋 Batería: {power_data[0]:.1f}V")
                self.corriente_ml_label.config(text=f"⚡ Motor Izq: {power_data[1]:.2f}A")
                self.corriente_mr_label.config(text=f"⚡ Motor Der: {power_data[2]:.2f}A")
            
            # Datos IMU - YA NO DISPONIBLES (solo mostrar mensaje fijo)
            # Los labels se mantienen deshabilitados mostrando "--"
            self.pitch_label.config(text="Pitch: --°")
            self.roll_label.config(text="Roll: --°")
            self.yaw_label.config(text="Yaw: --°")
            self.mov_label.config(text="Mov: --")
            self.vel_imu_label.config(text="Vel: --")
            self.accel_z_label.config(text="Accel Z: --")
            
            # Control y modo - con validación
            if 'control_data' in data and len(data['control_data']) >= 2:
                control_data = data['control_data']
                modo_actual = int(control_data[1])
                modos = {0: "APAGADO", 1: "SEGUIMIENTO", 2: "PAUSA", 3: "MANUAL"}
                colores_modo = {0: "#666666", 1: "#00ff88", 2: "#ffaa00", 3: "#00aaff"}
                
                self.modo_actual_label.config(text=f"MODO {modo_actual}", fg=colores_modo.get(modo_actual, "#ffffff"))
                self.modo_desc_label.config(text=modos.get(modo_actual, "DESCONOCIDO"))
                self.bateria_tag_label.config(text=f"🔋 Batería TAG: {control_data[0]:.1f}V")
            
            # Mostrar datos calculados del PID (si existen)
            if 'angulo_calculado' in data:
                self.angulo_label.config(text=f"Ángulo: {data['angulo_calculado']:.1f}°")
            else:
                self.angulo_label.config(text="Ángulo: -- °")
                
            if 'distancia_promedio' in data:
                self.distancia_label.config(text=f"Distancia: {data['distancia_promedio']:.0f}mm")
            else:
                self.distancia_label.config(text="Distancia: -- mm")
                
            if 'vel_izq' in data and 'vel_der' in data:
                self.motor_izq_label.config(text=f"Izq: {data['vel_izq']} PWM")
                self.motor_der_label.config(text=f"Der: {data['vel_der']} PWM")
                vel_max = max(abs(data['vel_izq']), abs(data['vel_der']))
                self.velocidad_label.config(text=f"Velocidad: {vel_max} PWM")
            else:
                self.motor_izq_label.config(text="Izq: -- PWM")
                self.motor_der_label.config(text="Der: -- PWM")
                self.velocidad_label.config(text="Velocidad: -- PWM")
            
            # Joystick - NUEVO FORMATO (valor único 0-7)
            if 'joy_valor' in data:
                joy_valor = int(data['joy_valor'])

                # Mapear valor a combinación de botones para mostrar en la GUI
                from src.utils.controladores.control import ControlModos
                control_temp = ControlModos()
                joy_d, joy_a, joy_i, joy_t = control_temp.mapear_joystick_valor(joy_valor)

                # Actualizar botones visuales según el mapeo
                botones = {'D': joy_d, 'A': joy_a, 'I': joy_i, 'T': joy_t}
                for button, is_pressed in botones.items():
                    try:
                        color = '#00ff00' if is_pressed == 1 else '#404040'
                        self.joy_buttons[button].config(bg=color, fg='black' if is_pressed else 'white')
                    except KeyError:
                        continue
            else:
                # Sin datos válidos de joystick, mostrar todo inactivo
                for button in ['D', 'A', 'I', 'T']:
                    try:
                        self.joy_buttons[button].config(bg='#404040', fg='white')
                    except KeyError:
                        continue
                
        except Exception as e:
            print(f"Error actualizando sensores: {e}")
            # No hacer add_log aquí para evitar spam
            
    def update_statistics(self, data):
        """Actualizar estadísticas del sistema"""
        try:
            timestamp = data['timestamp']
            datos_count = data['datos_count']
            
            # Tiempo activo
            horas = int(timestamp // 3600)
            minutos = int((timestamp % 3600) // 60)
            segundos = int(timestamp % 60)
            self.tiempo_activo_label.config(text=f"⏱️ Tiempo activo: {horas:02d}:{minutos:02d}:{segundos:02d}")
            
            # Datos recibidos
            self.datos_recibidos_label.config(text=f"📦 Datos recibidos: {datos_count}")
            
            # FPS aproximado
            if timestamp > 0:
                fps = datos_count / timestamp
                self.fps_label.config(text=f"🔄 FPS: {fps:.1f}")
                
        except Exception as e:
            print(f"Error actualizando estadísticas: {e}")
            
    def add_log(self, message):
        """Agregar mensaje al log - OPTIMIZADO"""
        try:
            # Evitar spam de logs
            current_time = time.time()
            if current_time - self.last_error_time < 1 and ("Error" in message or "❌" in message):
                return  # Ignorar mensajes de error muy frecuentes
            
            if "Error" in message or "❌" in message:
                self.last_error_time = current_time
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_entry = f"[{timestamp}] {message}\n"
            
            # Usar after_idle para no bloquear
            self.root.after_idle(lambda: self._insert_log(log_entry))
                
        except Exception:
            pass  # Silenciar errores de logging
            
    def _insert_log(self, log_entry):
        """Insertar entry en el log de forma segura"""
        try:
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
            
            # Limitar líneas del log más agresivamente
            lines = self.log_text.get("1.0", tk.END).split('\n')
            if len(lines) > 50:  # Menos líneas
                self.log_text.delete("1.0", f"{len(lines)-50}.0")
        except Exception:
            pass
            
    def on_closing(self):
        """Manejar cierre de la aplicación"""
        if self.sistema_activo:
            self.detener_sistema()
        self.root.destroy()

def main():
    """Función principal de la GUI"""
    root = tk.Tk()
    app = CarritoGolfGUI(root)
    
    # Manejar cierre de ventana
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Centrar ventana
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
    y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
    root.geometry(f"+{x}+{y}")
    
    print("🚀 Iniciando GUI del Carrito de Golf...")
    print("📱 Panel de control disponible en la interfaz gráfica")
    
    root.mainloop()

if __name__ == "__main__":
    main()
