#!/usr/bin/env python3
import time
import math
import os
import sys
import heapq
import depthai as dai

# Asegurar import de módulos locales (../src)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.utils.controladores.uart_motor_controller import UARTMotorController


def find_path_astar(grid, start, end):
    if not (0 <= start[0] < len(grid) and 0 <= start[1] < len(grid[0])) or grid[start[0]][start[1]] == 1:
        return None
    if not (0 <= end[0] < len(grid) and 0 <= end[1] < len(grid[0])) or grid[end[0]][end[1]] == 1:
        return None
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}
    while open_list:
        _, current = heapq.heappop(open_list)
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nb = (current[0] + dx, current[1] + dy)
            if not (0 <= nb[0] < len(grid) and 0 <= nb[1] < len(grid[0])):
                continue
            if grid[nb[0]][nb[1]] == 1:
                continue
            ng = g_score[current] + 1
            if ng < g_score.get(nb, 1e12):
                came_from[nb] = current
                g_score[nb] = ng
                f_score[nb] = ng + heuristic(nb, end)
                heapq.heappush(open_list, (f_score[nb], nb))
    return None


def convert_to_grid(rtab_grid_data, grid_size=100, resolution=0.1):
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
    origin_x = grid_size // 2
    origin_y = grid_size // 2
    if rtab_grid_data is not None and getattr(rtab_grid_data, 'points', None) is not None:
        for point in rtab_grid_data.points:
            cx = int(origin_x + point.x / resolution)
            cy = int(origin_y - point.z / resolution)
            if 0 <= cx < grid_size and 0 <= cy < grid_size:
                grid[cx][cy] = 1
    return grid


# En v3 no se requiere XLinkOut: se crean colas directamente desde los outputs.


def get_translation_xz(tf_msg):
    """Obtiene (x, z) de un TransformData soportando distintas APIs (propiedad o métodos)."""
    # 1) Método getTranslation() -> objeto con .x/.z o secuencia (x,y,z)
    if hasattr(tf_msg, 'getTranslation'):
        t = tf_msg.getTranslation()
        if hasattr(t, 'x') and hasattr(t, 'z'):
            return float(t.x), float(t.z)
        # Secuencia o numpy-like
        try:
            return float(t[0]), float(t[2])
        except Exception:
            pass
    # 2) Propiedad translation con .x/.z
    if hasattr(tf_msg, 'translation'):
        tr = tf_msg.translation
        if hasattr(tr, 'x') and hasattr(tr, 'z'):
            return float(tr.x), float(tr.z)
        try:
            return float(tr[0]), float(tr[2])
        except Exception:
            pass
    # 3) Fallback: cero
    return 0.0, 0.0


def main():
    p = dai.Pipeline()
    fps = 30
    width = 640
    height = 400

    # Fuentes
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    featureTracker = p.create(dai.node.FeatureTracker)
    odom = p.create(dai.node.RTABMapVIO)
    slam = p.create(dai.node.RTABMapSLAM)

    params = {
        'RGBD/CreateOccupancyGrid': 'true',
        'Grid/3D': 'false',           # 2D reduce drásticamente el tamaño
        'Grid/DepthDecimation': '4',  # más decimación
        'Grid/CellSize': '0.25',      # celdas más grandes
        'Grid/RangeMax': '5.0',       # limitar alcance
        'Rtabmap/SaveWMState': 'true',
    }
    slam.setParams(params)

    # IMU, FT, Stereo
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

    # Salidas para host (v3): colas directas desde los outputs
    # Nota: no usamos Rerun ni XLinkOut

    # Enlaces
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

    # Crear colas de salida directamente desde los outputs (v3)
    q_grid = slam.occupancyGridMap.createOutputQueue()
    q_tf = odom.transform.createOutputQueue()

    # Mantener flujo
    slam.passthroughRect.link(slam.rect)

    # Iniciar pipeline y ejecutar
    p.start()
    with p:
        print('SLAM navegación: iniciado')

        # Motores
        mc = UARTMotorController(port='/dev/ttyAMA0', baudrate=500000)
        connected = mc.connect()
        if connected:
            mc.set_smoothing_parameters(enable=True, max_acceleration=30)
            mc.set_speed_limits(max_speed=180, min_speed=-180)
        else:
            print('Aviso: sin conexión al motor controller, simulación de navegación')

        # Parámetros mapa/control
        grid_size = 100
        resolution = 0.1
        origin_x = grid_size // 2
        origin_y = grid_size // 2
        base_speed = 120
        turn_gain = 0.8

        try:
            while p.isRunning():
                if not q_tf.has():
                    time.sleep(0.01)
                    continue
                tf = q_tf.get()
                cur_x, cur_z = get_translation_xz(tf)

                msg_grid = q_grid.get() if q_grid.has() else None
                grid = convert_to_grid(msg_grid, grid_size=grid_size, resolution=resolution)

                cur = (
                    int(origin_x + cur_x / resolution),
                    int(origin_y - cur_z / resolution),
                )

                # Destino de ejemplo: 1m derecha, 2m adelante
                tag_x_rel = 1.0
                tag_z_rel = 2.0
                dest = (
                    int(origin_x + (cur_x + tag_x_rel) / resolution),
                    int(origin_y - (cur_z + tag_z_rel) / resolution),
                )

                path = find_path_astar(grid, cur, dest)
                if path and len(path) > 0:
                    nxt = path[1] if len(path) > 1 else dest
                    dx = nxt[0] - cur[0]
                    dy = nxt[1] - cur[1]
                    angle = math.atan2(dx, -dy) if (dx or dy) else 0.0
                    turn = max(-1.0, min(1.0, (angle / (math.pi/2)) * turn_gain))
                    forward = int(base_speed * max(0.2, 1.0 - abs(turn)))
                    print(f'pos={cur} dest={dest} next={nxt} f={forward} t={turn:.2f}')
                    if connected:
                        mc.move_with_steering(forward, turn)
                else:
                    print('Sin ruta: STOP')
                    if connected:
                        mc.stop_motors()

                time.sleep(0.02)
        except KeyboardInterrupt:
            print('Deteniendo...')
        finally:
            if connected:
                mc.stop_motors()
                mc.disconnect()


if __name__ == '__main__':
    main()
