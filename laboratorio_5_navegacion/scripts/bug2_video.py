import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. CÁLCULO PREVIO DE LA TRAYECTORIA BUG2
# ==========================================
# Se calcula la ruta antes de animar para asegurar una reproducción fluida
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1 # Obstáculo base de la guía[cite: 15]

start = np.array([2, 2])
goal = np.array([18, 18])

pos = start.copy()
path = [pos.copy()]

def is_free(p):
    if 0 <= p[0] < grid.shape[1] and 0 <= p[1] < grid.shape[0]:
        return grid[p[1], p[0]] == 0
    return False

for _ in range(500):
    direction = goal - pos
    if abs(direction[0]) > abs(direction[1]):
        step = np.array([np.sign(direction[0]), 0])
    else:
        step = np.array([0, np.sign(direction[1])])
        
    next_pos = pos + step
    if is_free(next_pos):
        pos = next_pos
    else:
        for move in [np.array([1,0]), np.array([0,1]), np.array([-1,0]), np.array([0,-1])]:
            if is_free(pos + move):
                pos = pos + move
                break
    path.append(pos.copy())
    if np.linalg.norm(pos - goal) < 1:
        break

path = np.array(path)

# ==========================================
# 2. CONFIGURACIÓN ESTÉTICA DEL GRÁFICO
# ==========================================
print("\nIniciando simulación de video...")
print("Presiona Ctrl+C en esta terminal para detener la ejecución forzosamente si es necesario.\n")

plt.figure(figsize=(8, 8))
plt.imshow(grid, origin='lower', cmap='gray_r') #[cite: 15]

# Dibujar elementos estáticos (Línea-m, Inicio, Meta)
plt.plot([start[0], goal[0]], [start[1], goal[1]], color='red', linestyle='--', alpha=0.4, label="Línea-m")
plt.scatter(start[0], start[1], c='blue', s=120, edgecolors='black', zorder=5, label="Inicio") #[cite: 15]
plt.scatter(goal[0], goal[1], c='red', s=120, edgecolors='black', zorder=5, label="Meta") #[cite: 15]

plt.gca().set_aspect('equal') #[cite: 16]
plt.grid(color='gray', linestyle=':', linewidth=0.5, alpha=0.7) #[cite: 16]
plt.title("Simulación Dinámica: Bug2 Simplificado", fontsize=14, fontweight='bold')
plt.legend(loc='lower right', shadow=True)

# Objetos dinámicos que se actualizarán en el bucle
linea_ruta, = plt.plot([], [], 'g-', linewidth=2.5)
punto_robot, = plt.plot([], [], 'go', markersize=10, markeredgecolor='black', zorder=6)

# ==========================================
# 3. BUCLE DE ANIMACIÓN (TIEMPO REAL)
# ==========================================
# Activar modo interactivo
plt.ion()
plt.show()

# Recorrer el array de la ruta y actualizar la gráfica cuadro por cuadro
for i in range(len(path)):
    # Actualiza la línea verde hasta el paso actual
    linea_ruta.set_data(path[:i+1, 0], path[:i+1, 1])
    
    # Actualiza la posición del "robot" (punto verde)
    punto_robot.set_data([path[i, 0]], [path[i, 1]])
    
    # Pausa de 0.05 segundos entre frames (ajusta este valor para cambiar la velocidad del video)
    plt.pause(0.05)

# Desactivar modo interactivo al terminar para que la ventana no se cierre sola
plt.ioff()
print("Simulación finalizada. Cierra la ventana gráfica para volver a la terminal.")
plt.show()
