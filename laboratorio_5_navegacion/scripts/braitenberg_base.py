# =======================================
# ACTIVIDAD A1: VEHÍCULO DE BRAITENBERG 
# =======================================

import numpy as np
import matplotlib.pyplot as plt

# 1. Definición del Campo Escalar (Objetivo por defecto en xc=60, yc=90)
def sensorfield(x, y):
    xc, yc = (60, 90)
    return 1000 / ((x - xc)**2 + (y - yc)**2 + 50)

# 2. Configuración de Parámetros Generales
dt = 0.1
Ks = 17
Kv = 14

# 3. Definición del Estado Inicial del Robot
x, y, theta = 10, 10, 0
traj_x, traj_y = [], []

# Encabezado estructurado para la terminal
print("=" * 65)
print(f"{'Iteración':<10} | {'s_left':<10} | {'s_right':<10} | {'v (m/s)':<10} | {'w (rad/s)':<10}")
print("=" * 65)

# 4. Bucle Principal de Simulación Cinemática (222 Iteraciones)
for i in range(222):
    # Sensores (offset lateral d=2)
    d = 2
    s_left = sensorfield(x - d*np.sin(theta), y + d*np.cos(theta))
    s_right = sensorfield(x + d*np.sin(theta), y - d*np.cos(theta))

    # Ley de Control
    v = -Kv * (s_left + s_right)
    omega = Ks * (s_right - s_left)

    # Imprimir resumen ordenado cada 20 iteraciones para no saturar la pantalla
    if i % 20 == 0 or i == 221:
        print(f"{i:<10} | {s_left:<10.3f} | {s_right:<10.3f} | {v:<10.3f} | {omega:<10.3f}")

    # Integración Cinemática
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    traj_x.append(x)
    traj_y.append(y)

print("=" * 65)
print(f"Posición Final Alcanzada: X = {x:.2f} m, Y = {y:.2f} m")
print("=" * 65)

# 5. Visualización Estructurada y Estética de Resultados
plt.figure(figsize=(8, 7))

# Trazo de la trayectoria del robot
plt.plot(traj_x, traj_y, 'b-', linewidth=2.5, label="Trayectoria del Robot (N=222)")

# Marcar punto de inicio y punto objetivo
plt.scatter([10], [10], c='green', s=120, zorder=5, label="Inicio (10, 10)")
plt.scatter([60], [90], c='red', s=150, marker='*', zorder=5, label="Objetivo (60, 90)")

# Formato visual
plt.title("Navegación Reactiva: Vehículo de Braitenberg (Caso Base)", fontsize=12, fontweight='bold')
plt.xlabel("Posición X [m]")
plt.ylabel("Posición Y [m]")
plt.grid(True, linestyle='--', alpha=0.7)
plt.axis('equal')
plt.legend(loc='upper left')
plt.tight_layout()

# Guardar figura en alta calidad
plt.savefig("braitenberg_base_oficial.png", dpi=300)
plt.show()
