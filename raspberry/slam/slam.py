import time
import depthai as dai
import heapq  # Para implementar el algoritmo A* de manera eficiente
import os
import sys
import math

# Asegurar import de módulos locales (../src)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Control de motores por UART (sin Rerun)
from src.utils.controladores.uart_motor_controller import UARTMotorController

# La función A* para encontrar el camino más corto en la cuadrícula de ocupación
def find_path_astar(grid, start, end):
    # Asegúrate de que el inicio y el final no estén en un obstáculo
    if not (0 <= start[0] < len(grid) and 0 <= start[1] < len(grid[0])) or grid[start[0]][start[1]] == 1:
        print("Error: El punto de inicio está fuera de la cuadrícula o en un obstáculo.")
        return None
    if not (0 <= end[0] < len(grid) and 0 <= end[1] < len(grid[0])) or grid[end[0]][end[1]] == 1:
        print("Error: El punto de destino está fuera de la cuadrícula o en un obstáculo.")
        return None
    
    # Heurística: Distancia de Manhattan entre el punto actual y el destino
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Colas de prioridad para los nodos a explorar
    open_list = []
    heapq.heappush(open_list, (0, start))
    
    # Diccionarios para almacenar el camino y el costo
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_list:
        current_f, current = heapq.heappop(open_list)
        
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1] # Devuelve el camino invertido

        # Explora los vecinos
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Revisa si el vecino está dentro de la cuadrícula y no es un obstáculo
            if not (0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0])) or grid[neighbor[0]][neighbor[1]] == 1:
                continue

            tentative_g_score = g_score[current] + 1
            
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
                
    return None # No se encontró un camino


# Función para convertir los datos de RTABMap a una cuadrícula 2D simple
def convert_to_grid(rtab_grid_data, grid_size=100, resolution=0.1):
    # La cuadrícula de ocupación de RTABMap puede ser 3D, pero para path planning 2D
    # necesitamos un plano (ej. el plano XZ del suelo).
    # grid_size es el número de celdas por lado, resolution es metros por celda.
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
    origin_x = grid_size // 2
    origin_y = grid_size // 2
    
    if rtab_grid_data is not None and rtab_grid_data.points is not None:
        for point in rtab_grid_data.points:
            # Convertir la coordenada 3D a la celda 2D
            cell_x = int(origin_x + point.x / resolution)
            cell_y = int(origin_y - point.z / resolution)
            
            if 0 <= cell_x < grid_size and 0 <= cell_y < grid_size:
                # Marcamos la celda como obstáculo (1)
                grid[cell_x][cell_y] = 1
    
    return grid

def get_translation_xz(tf_msg):
    """Compatibilidad para obtener (x,z) de TransformData en distintas APIs."""
    if hasattr(tf_msg, 'getTranslation'):
        t = tf_msg.getTranslation()
        if hasattr(t, 'x') and hasattr(t, 'z'):
            return float(t.x), float(t.z)
        try:
            return float(t[0]), float(t[2])
        except Exception:
            pass
    if hasattr(tf_msg, 'translation'):
        tr = tf_msg.translation
        if hasattr(tr, 'x') and hasattr(tr, 'z'):
            return float(tr.x), float(tr.z)
        try:
            return float(tr[0]), float(tr[2])
        except Exception:
            pass
    return 0.0, 0.0

# Helper de compatibilidad para crear XLinkOut en distintas versiones de DepthAI (v2/v3)
def create_xlink_out(pipeline: 'dai.Pipeline', stream_name: str):
    NodeType = None
    # DepthAI v2
    if hasattr(dai, 'node') and hasattr(dai.node, 'XLinkOut'):
        NodeType = dai.node.XLinkOut
    # DepthAI v3 (posible reubicación en submódulo IO)
    elif hasattr(dai, 'node') and hasattr(dai.node, 'IO') and hasattr(dai.node.IO, 'XLinkOut'):
        NodeType = dai.node.IO.XLinkOut
    # Fallback (algunos paquetes exponen clase en el módulo raíz)
    elif hasattr(dai, 'XLinkOut'):
        NodeType = dai.XLinkOut
    else:
        raise AttributeError("No se encontró XLinkOut en depthai; verifique la versión/API de DepthAI v3.")
    node = pipeline.create(NodeType)
    if hasattr(node, 'setStreamName'):
        node.setStreamName(stream_name)
    else:
        # API alternativa: propiedad o método distinto
        try:
            node.streamName = stream_name
        except Exception:
            pass
    return node

# --- Inicio del pipeline de DepthAI ---
with dai.Pipeline() as p:
    fps = 30
    width = 640
    height = 400

    # Define sources and outputs
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    featureTracker = p.create(dai.node.FeatureTracker)
    odom = p.create(dai.node.RTABMapVIO)
    slam = p.create(dai.node.RTABMapSLAM)
    
    # Reducir tamaño del mensaje del grid para evitar límites de XLink
    params = {
        "RGBD/CreateOccupancyGrid": "true",
        "Grid/3D": "false",          # 2D grid para menos datos
        "Grid/DepthDecimation": "3", # decimación de profundidad
        "Grid/CellSize": "0.20",     # celdas más grandes
        "Grid/RangeMax": "6.0",      # limitar alcance
        "Rtabmap/SaveWMState": "true"
    }
    slam.setParams(params)
    
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    featureTracker.setHardwareResources(1, 2)
    featureTracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTracker.initialConfig.setNumTargetFeatures(1000)
    featureTracker.initialConfig.setMotionEstimator(False)
    featureTracker.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

    # Salida para acceder a la cuadrícula de ocupación y a la transformación de odometría
    xout_occupancy_grid = create_xlink_out(p, "occupancy_grid")
    xout_transform = create_xlink_out(p, "odom_transform")

    # Linking
    left.requestOutput((width, height)).link(stereo.left)
    right.requestOutput((width, height)).link(stereo.right)
    
    featureTracker.passthroughInputImage.link(odom.rect)
    stereo.rectifiedLeft.link(featureTracker.inputImage)
    stereo.depth.link(odom.depth)
    imu.out.link(odom.imu)
    featureTracker.outputFeatures.link(odom.features)

    odom.transform.link(slam.odom)
    odom.passthroughRect.link(slam.rect)
    odom.passthroughDepth.link(slam.depth)

    # Conectamos las salidas de interés a nuestros nodos XLinkOut
    slam.occupancyGridMap.link(xout_occupancy_grid.input)
    # Ahora también conectamos la salida de la transformación de odometría
    odom.transform.link(xout_transform.input)

    # Conectar el passthrough de la imagen rectificada para que no se detenga el flujo
    slam.passthroughRect.link(slam.rect)
    
    # Arrancamos la tubería y la navegación (sin Rerun)
    with dai.Device(p) as device:
        print("Pipeline iniciado. Presiona Ctrl+C para detener.")

        # Queues de salida
        q_occupancy_grid = device.getOutputQueue(name="occupancy_grid")
        q_odom_transform = device.getOutputQueue(name="odom_transform")

        # Controlador de motores
        motor = UARTMotorController(port='/dev/ttyAMA0', baudrate=500000)
        connected = motor.connect()
        if connected:
            # Suavizado moderado para evitar tirones
            motor.set_smoothing_parameters(enable=True, max_acceleration=30)
            # Limitar velocidad por seguridad
            motor.set_speed_limits(max_speed=180, min_speed=-180)
        else:
            print("Advertencia: No se pudo conectar al controlador de motores. Solo navegación simulada.")

        # Parámetros de navegación
        grid_size = 100
        resolution = 0.1  # metros por celda (debe coincidir con convert_to_grid)
        origin_x = grid_size // 2
        origin_y = grid_size // 2
        base_speed = 120  # PWM base avance
        turn_gain = 0.8   # Ganancia de giro (0-1)

        try:
            while True:
                # Odometría
                transform_data = q_odom_transform.tryGet()
                if transform_data is None:
                    time.sleep(0.01)
                    continue

                current_x, current_z = get_translation_xz(transform_data)

                # Mapa de ocupación
                occupancy_grid_data = q_occupancy_grid.tryGet()
                grid = convert_to_grid(occupancy_grid_data, grid_size=grid_size, resolution=resolution)

                # Posición actual en celdas (fila, col) == (y, x)
                current_position = (
                    int(origin_x + current_x / resolution),
                    int(origin_y - current_z / resolution)
                )

                # Destino relativo al coche (ejemplo: 1m a la derecha, 2m adelante)
                tag_x_relative = 1.0
                tag_z_relative = 2.0
                tag_x_global = current_x + tag_x_relative
                tag_z_global = current_z + tag_z_relative
                destination = (
                    int(origin_x + tag_x_global / resolution),
                    int(origin_y - tag_z_global / resolution)
                )

                # Planificar ruta
                path = find_path_astar(grid, current_position, destination)

                if path and len(path) > 0:
                    next_step = path[1] if len(path) > 1 else destination
                    print(f"pos={current_position} dest={destination} next={next_step}")

                    # Control: calcular ángulo hacia la próxima celda.
                    # Vector objetivo en coords del mapa: (dx, dy). Adelante ≈ dy<0
                    dx = next_step[0] - current_position[0]
                    dy = next_step[1] - current_position[1]
                    # Error de orientación: +giro derecha si dx>0 y objetivo adelante
                    # Aproximación: ángulo hacia el punto relativo al eje -Y (adelante)
                    # angle = atan2(dx, -dy)  -> 0=recto adelante, +derecha, -izquierda
                    angle = math.atan2(dx, -dy) if (dx != 0 or dy != 0) else 0.0
                    # Mapear a [-1,1]
                    turn = max(-1.0, min(1.0, (angle / (math.pi/2)) * turn_gain))
                    # Reducir avance si el giro es grande
                    forward = int(base_speed * max(0.2, 1.0 - abs(turn)))

                    if connected:
                        # Aplicar comando a motores (move_with_steering usa turn_rate en [-1,1])
                        motor.move_with_steering(forward_speed=forward, turn_rate=turn)
                else:
                    print("Sin ruta: detener por seguridad")
                    if connected:
                        motor.stop_motors()

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("Interrumpido. Deteniendo motores...")
        finally:
            if connected:
                motor.stop_motors()
                motor.disconnect()
