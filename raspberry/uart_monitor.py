#pepito
import os
import csv
import time
import numpy as np
from datetime import datetime

from src.config.variables import DATA_PORT, BAUDRATE, TIEMPO_ESPERA
from src.utils.lectores.uart_data_receiver import UARTDataReceiver

LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs')
LOG_FILE = os.path.join(LOG_DIR, f"uart_errors_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

# CSV columns: timestamp_iso, event, sensor_index, value, duration_sec
# event: START(NaN detected), END(recovered), SAMPLE(optional periodic sample)

class SensorErrorTracker:
    def __init__(self, n_sensors: int = 3):
        self.n = n_sensors
        self.active = [False] * n_sensors
        self.start_ts = [0.0] * n_sensors

    def update(self, values, now_ts, writer):
        # Ensure correct length
        if values is None or len(values) < self.n:
            return
        for i in range(self.n):
            try:
                v = float(values[i])
                # Detectar NaN
                is_error = np.isnan(v)
            except (ValueError, TypeError):
                # Si no se puede convertir a float, considerar como error
                is_error = True
                v = np.nan
                
            if not self.active[i] and is_error:
                self.active[i] = True
                self.start_ts[i] = now_ts
                writer.writerow({
                    'timestamp_iso': datetime.fromtimestamp(now_ts).isoformat(),
                    'event': 'START',
                    'sensor_index': i,
                    'value': 'NaN' if np.isnan(v) else str(v),
                    'duration_sec': ''
                })
            elif self.active[i] and not is_error:
                duration = now_ts - self.start_ts[i]
                writer.writerow({
                    'timestamp_iso': datetime.fromtimestamp(now_ts).isoformat(),
                    'event': 'END',
                    'sensor_index': i,
                    'value': v,
                    'duration_sec': f"{duration:.3f}"
                })
                self.active[i] = False
                self.start_ts[i] = 0.0


def main():
    os.makedirs(LOG_DIR, exist_ok=True)

    receiver = UARTDataReceiver(port=DATA_PORT, baudrate=BAUDRATE)
    if not receiver.connect():
        print(f"Error: No se pudo conectar al puerto {DATA_PORT}")
        return

    with open(LOG_FILE, 'w', newline='') as f:
        fieldnames = ['timestamp_iso', 'event', 'sensor_index', 'value', 'duration_sec']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        tracker = SensorErrorTracker(n_sensors=3)

        print(f"Monitoreando UART. Log: {LOG_FILE}")
        try:
            while True:
                # Leer y tolerar versiones antiguas del receptor
                try:
                    data = receiver.read_data()
                except Exception as e:
                    print(f"Excepción al leer UART (monitor): {e}")
                    data = None
                now_ts = time.time()
                if data is None:
                    # Intentar reconectar si se perdió la conexión
                    if hasattr(receiver, 'is_connected') and not receiver.is_connected:
                        print("Reconectando UART...")
                        try:
                            receiver.disconnect()
                        except Exception:
                            pass
                        time.sleep(0.2)
                        receiver.connect()
                    if TIEMPO_ESPERA:
                        time.sleep(TIEMPO_ESPERA)
                    continue

                # Extraer distancias/sensores desde el receptor (array de 3 elementos)
                try:
                    values = receiver.get_distances_array()
                except Exception as e:
                    print(f"Excepción obteniendo distancias: {e}")
                    values = None
                tracker.update(values, now_ts, writer)
                f.flush()
        except KeyboardInterrupt:
            print("Finalizando monitoreo...")
        finally:
            receiver.disconnect()


if __name__ == '__main__':
    main()

