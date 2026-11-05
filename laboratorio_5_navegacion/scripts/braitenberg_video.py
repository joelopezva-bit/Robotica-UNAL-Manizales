# ==============================================================================
# SCRIPT EXCLUSIVO PARA GRABACIÓN DE VIDEO: ANIMACIÓN BRAITENBERG
# ==============================================================================

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 1. Campo Escalar (Objetivo en xc=60, yc=90)
def sensorfield(x, y):
    return 1000 / ((x - 60)**2 + (y - 90)**2 + 50)

# 2. Simulación
x, y, theta = 10.0, 10.0, 0.0
dt, Kv, Ks, d = 0.1, 14, 17, 2
tx, ty = [x], [y]

for _ in range(250):
    sl = sensorfield(x - d*np.sin(theta), y + d*np.cos(theta))
    sr = sensorfield(x + d*np.sin(theta), y - d*np.cos(theta))
    v = -Kv * (sl + sr)
    w = Ks * (sr - sl)
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += w * dt
    tx.append(x)
    ty.append(y)

# 3. Configuración del Gráfico Dinámico
fig, ax = plt.subplots(figsize=(7, 6))
ax.scatter([60], [90], c='red', s=150, marker='*', zorder=5, label="Objetivo (60, 90)")
ax.scatter([10], [10], c='green', s=100, zorder=5, label="Inicio (10, 10)")

line, = ax.plot([], [], 'b-', linewidth=2.5, label="Trayectoria del Robot")
robot_pt, = ax.plot([], [], 'go', markersize=8, zorder=6, label="Robot Braitenberg")

ax.set_xlim(-10, 75)
ax.set_ylim(-10, 105)
ax.set_title("Simulación Dinámica: Vehículo de Braitenberg", fontweight='bold')
ax.set_xlabel("Posición X [m]")
ax.set_ylabel("Posición Y [m]")
ax.grid(True, linestyle='--', alpha=0.7)
ax.axis('equal')
ax.legend(loc='upper left')

def update_anim(frame):
    line.set_data(tx[:frame], ty[:frame])
    robot_pt.set_data([tx[frame]], [ty[frame]])
    return line, robot_pt

anim = FuncAnimation(fig, update_anim, frames=len(tx), interval=30, blit=True)

plt.tight_layout()
plt.show()
