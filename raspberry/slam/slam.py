import time
import depthai as dai
import heapq # Para implementar el algoritmo A* de manera eficiente

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
    
    params = {"RGBD/CreateOccupancyGrid": "true",
              "Grid/3D": "true",
              "Rtabmap/SaveWMState": "true"}
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
    xout_occupancy_grid = p.create(dai.node.XLinkOut)
    xout_occupancy_grid.setStreamName("occupancy_grid")

    xout_transform = p.create(dai.node.XLinkOut)
    xout_transform.setStreamName("odom_transform")

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
    
    # Arrancamos la tubería
    with dai.Device(p) as device:
        print("Pipeline iniciado. Presiona Ctrl+C para detener.")
        
        q_occupancy_grid = device.getOutputQueue(name="occupancy_grid", maxSize=8, blocking=False)
        q_odom_transform = device.getOutputQueue(name="odom_transform", maxSize=8, blocking=False)
        
        while True:
            try:
                # Obtenemos la posición actual del coche de la odometría
                transform_data = q_odom_transform.tryGet()
                if transform_data is None:
                    # Si no hay datos, continuamos esperando
                    time.sleep(0.01)
                    continue

                # La posición actual del coche en el espacio 3D
                current_x = transform_data.translation.x
                current_z = transform_data.translation.z # Usamos Z como el eje "hacia adelante"

                # Obtenemos los datos de la cuadrícula de ocupación si están disponibles
                occupancy_grid_data = q_occupancy_grid.tryGet()
                grid = convert_to_grid(occupancy_grid_data)

                # Definimos el tamaño y la resolución de la cuadrícula
                grid_size = 100
                resolution = 0.1
                origin_x = grid_size // 2
                origin_y = grid_size // 2

                # Convertimos la posición 3D del coche a una celda de la cuadrícula 2D
                current_position = (int(origin_x + current_x / resolution), int(origin_y - current_z / resolution))

                # --- Lógica para el tag ---
                # A diferencia del código anterior, el destino es relativo al coche.
                # Aquí simulamos la detección de un tag. En la vida real, lo obtendrías
                # de otro nodo de la pipeline o de un detector de AprilTags.
                # Por ejemplo, un tag 1 metro en X y 2 metros en Z (hacia adelante)
                tag_x_relative = 1.0
                tag_z_relative = 2.0
                
                # Calculamos la posición del tag en el sistema de coordenadas del mapa
                tag_x_global = current_x + tag_x_relative
                tag_z_global = current_z + tag_z_relative

                # Convertimos la posición global del tag a una celda de la cuadrícula
                destination = (int(origin_x + tag_x_global / resolution), int(origin_y - tag_z_global / resolution))

                # Ejecutamos el algoritmo A* para encontrar la ruta
                path = find_path_astar(grid, current_position, destination)
                
                if path:
                    # El próximo paso es la siguiente celda en la ruta
                    next_step = path[1] if len(path) > 1 else destination
                    print(f"Posición actual del coche: {current_position}")
                    print(f"Posición del tag (destino): {destination}")
                    print(f"El próximo paso es moverse a la celda: {next_step}")
                    # # Aquí iría la llamada a la función para mover el coche
                    # move_car_to(next_step)
                else:
                    print("No se encontró una ruta. El destino podría estar bloqueado.")
                    # # Aquí iría la lógica para detener o esperar a que la cuadrícula se actualice
                    # stop_car()

                time.sleep(0.01)
            
            except KeyboardInterrupt:
                break
