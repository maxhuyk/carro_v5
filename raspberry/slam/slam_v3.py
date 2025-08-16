import time
import numpy as np
from typing import List, Tuple, Optional

# Requiere depthai-sdk (lo tienes instalado: 1.15.1) y DepthAI v3
from depthai_sdk import OakCamera

# A* sobre grilla [fila][col] = [y][x]
def find_path_astar(grid: List[List[int]], start: Tuple[int,int], end: Tuple[int,int]) -> Optional[List[Tuple[int,int]]]:
    h, w = len(grid), len(grid[0]) if grid else 0
    sx, sy = start[1], start[0]  # start: (y,x)
    ex, ey = end[1], end[0]      # end: (y,x)
    if not (0 <= sy < h and 0 <= sx < w) or grid[sy][sx] == 1:
        return None
    if not (0 <= ey < h and 0 <= ex < w) or grid[ey][ex] == 1:
        return None
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    import heapq
    open_list = []
    heapq.heappush(open_list, (0, (sy, sx)))
    came_from = {}
    g = {(sy, sx): 0}
    f = {(sy, sx): heuristic((sy, sx), (ey, ex))}
    while open_list:
        _, cur = heapq.heappop(open_list)
        if cur == (ey, ex):
            path = []
            while cur in came_from:
                path.append((cur[0], cur[1]))
                cur = came_from[cur]
            path.append((sy, sx))
            path.reverse()
            # Convertir a (y,x)
            return [(r, c) for r,c in path]
        for dr, dc in ((1,0), (-1,0), (0,1), (0,-1)):
            nr, nc = cur[0]+dr, cur[1]+dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if grid[nr][nc] == 1:
                continue
            ng = g[cur] + 1
            if ng < g.get((nr, nc), 1e12):
                came_from[(nr, nc)] = cur
                g[(nr, nc)] = ng
                f[(nr, nc)] = ng + heuristic((nr, nc), (ey, ex))
                heapq.heappush(open_list, (f[(nr, nc)], (nr, nc)))
    return None

# Convierte un frame de profundidad (uint16 mm) en grilla 2D simple
# grid_h: celdas delante/detrás (vertical), grid_w: celdas izquierda/derecha (horizontal)
# resolution_m: metros por celda; max_range_m: límite de mapeo

def depth_to_grid(depth_mm: np.ndarray, grid_h=100, grid_w=100, resolution_m=0.05, max_range_m=5.0) -> List[List[int]]:
    if depth_mm is None:
        return [[0]*grid_w for _ in range(grid_h)]
    H, W = depth_mm.shape
    # Usamos mitad inferior (cerca del suelo) para detección simple
    roi = depth_mm[H//2:,:]
    grid = [[0]*grid_w for _ in range(grid_h)]
    # Mapear columnas de imagen a columnas de grilla
    for cx in range(W):
        col = roi[:, cx]
        # Ignorar ceros (sin dato); tomar el valor mínimo > 0
        nz = col[col > 0]
        if nz.size == 0:
            continue
        d_mm = int(np.min(nz))
        d_m = d_mm / 1000.0
        if d_m <= 0 or d_m > max_range_m:
            continue
        # Posición en la grilla: y más cerca es fila pequeña; más lejos es fila grande
        y = int((d_m / max_range_m) * (grid_h-1))
        x = int((cx / (W-1)) * (grid_w-1))
        # Marcar obstáculo; expandir un poco
        for dy in (-1,0,1):
            yy = max(0, min(grid_h-1, y+dy))
            for dx in (-1,0,1):
                xx = max(0, min(grid_w-1, x+dx))
                grid[yy][xx] = 1
    return grid


def main():
    # Parámetros de grilla y navegación
    GRID_H = 100
    GRID_W = 100
    RES_M = 0.05   # 5 cm por celda
    MAX_RANGE = 5.0

    latest_depth = {'frame': None, 'ts': 0.0}

    def on_depth(packet, visualizer=None):
        # depthai-sdk entrega packet.frame (np.ndarray) o packet.msg.getFrame()
        try:
            frame = getattr(packet, 'frame', None)
            if frame is None and hasattr(packet, 'msg'):
                frame = packet.msg.getFrame()
        except Exception:
            frame = None
        if isinstance(frame, np.ndarray):
            latest_depth['frame'] = frame
            latest_depth['ts'] = time.time()

    with OakCamera() as oak:
        # Crear estéreo a 400p @ 30 fps
        stereo = oak.create_stereo(resolution='400p', fps=30)
        # Registrar callback de profundidad
        oak.callback(stereo.depth, on_depth)
        # Iniciar sin bloquear para poder buclear
        oak.start(blocking=False)

        print('SLAM v3 demo: generando grilla simple desde profundidad y ejecutando A*')
        try:
            while True:
                depth = latest_depth['frame']
                if depth is None:
                    time.sleep(0.01)
                    continue
                grid = depth_to_grid(depth, grid_h=GRID_H, grid_w=GRID_W, resolution_m=RES_M, max_range_m=MAX_RANGE)
                # Posición actual: centro inferior de la grilla
                cur = (GRID_H-1, GRID_W//2)  # (y,x)
                # Destino simulado: 2m adelante y 0.5m a la derecha
                ahead_cells = int(2.0 / RES_M)
                right_cells = int(0.5 / RES_M)
                dest = (max(0, cur[0]-ahead_cells), min(GRID_W-1, cur[1]+right_cells))

                path = find_path_astar(grid, cur, dest)
                if path:
                    nxt = path[1] if len(path) > 1 else dest
                    print(f"pos={cur} dest={dest} next={nxt} ts={latest_depth['ts']:.2f}")
                else:
                    print(f"pos={cur} dest={dest} sin ruta (posible bloqueo)")
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
