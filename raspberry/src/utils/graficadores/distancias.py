import numpy as np
import matplotlib.pyplot as plt

def graficar_distancias(tiempos, distancias_hist, distancias_filtradas_hist):
    """
    Grafica las distancias originales y filtradas de los sensores en función del tiempo.
    tiempos: lista o array de tiempos
    distancias_hist: array (n_samples, 3) de distancias originales
    distancias_filtradas_hist: array (n_samples, 3) de distancias filtradas
    """
    distancias_hist = np.array(distancias_hist)
    distancias_filtradas_hist = np.array(distancias_filtradas_hist)

    plt.figure(figsize=(10, 6))
    for i in range(3):
        plt.plot(tiempos, distancias_hist[:, i], label=f'Distancia Sensor {i+1} (raw)')
        plt.plot(tiempos, distancias_filtradas_hist[:, i], '--', label=f'Distancia Sensor {i+1} (filtrada)')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Distancia [mm]')
    plt.title('Distancias UWB vs Distancias Filtradas')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()
        
 

def graficar_distancia_sensor1(tiempos, distancia_s1_hist):
    """
    Grafica la distancia del sensor 1 al tag en función del tiempo.

    Parámetros:
    - tiempos: lista de tiempos [t0, t1, t2, ...]
    - distancia_s1_hist: lista de distancias del sensor 1 [d0, d1, d2, ...]
    """
    plt.figure(figsize=(8, 4))
    plt.plot(tiempos, distancia_s1_hist, color='blue', marker='o', linestyle='-')
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Distancia sensor 1 al tag [mm]")
    plt.title("Distancia del sensor 1 al tag vs Tiempo")
    plt.grid(True)
    plt.tight_layout()
    plt.show()