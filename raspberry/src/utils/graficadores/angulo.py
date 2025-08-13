import matplotlib.pyplot as plt

def graficar_angulos(tiempos, angulos):
    plt.figure(figsize=(10, 4))
    plt.plot(tiempos, angulos, marker='o', linestyle='-', color='mediumblue')
    plt.title("Variación del ángulo en el tiempo")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Ángulo (°)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()