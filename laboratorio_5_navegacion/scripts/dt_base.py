# Distance Transform
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# MAPA
# ==========================================
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1 # obstáculo

goal = (2, 2)

# ==========================================
# DISTANCE TRANSFORM (NO ÓPTIMO)
# ==========================================
dist = np.ones_like(grid) * np.inf
dist[goal[1], goal[0]] = 0

print("\nCalculando propagación de distancias (Distance Transform)...")
# propagación iterativa
for iteracion in range(100):
    for x in range(20):
        for y in range(20):
            if grid[y, x] == 0:
                neighbors = []
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < 20 and 0 <= ny < 20:
                        neighbors.append(dist[ny, nx])
                dist[y, x] = min(dist[y, x], min(neighbors) + 1)

print("Cálculo completado.")

# ==========================================
# SEGUIMIENTO DE CAMINO
# ==========================================
start = (18, 18)
pos = start
path = [pos]

print("\nTrazando ruta por descenso de gradiente...")
for _ in range(200):
    x, y = pos
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < 20 and 0 <= ny < 20:
            neighbors.append(((nx, ny), dist[ny, nx]))
            
    pos = min(neighbors, key=lambda item: item[1])[0]
    path.append(pos)
    
    if pos == goal:
        break

path = np.array(path)

print("\n" + "="*45)
print(f"| {'RESULTADO DISTANCE TRANSFORM':^41} |")
print("="*45)
print(f"| {'Inicio':<20} | {str(start):<18} |")
print(f"| {'Meta (Origen del campo)':<20} | {str(goal):<18} |")
print(f"| {'Pasos de la ruta':<20} | {len(path):<18} |")
print("-" * 45 + "\n")

# ==========================================
# VISUALIZACIÓN (MAPA DE CALOR)
# ==========================================
plt.figure(figsize=(6, 6))

# heatmap de distancias
plt.imshow(dist, origin='lower', cmap='hot')

# obstáculos (en negro)
obs_y, obs_x = np.where(grid == 1)
plt.scatter(obs_x, obs_y, c='black', s=20)

# trayectoria
plt.plot(path[:, 0], path[:, 1], 'cyan', linewidth=2, label="trayectoria")

# puntos inicio/fin
plt.scatter(start[0], start[1], c='blue', s=80, label="inicio")
plt.scatter(goal[0], goal[1], c='green', s=80, label="objetivo")

# detalles visuales
plt.colorbar(label="Distancia al objetivo")
plt.title("Distance Transform (Iterativo) - Heatmap")
plt.legend()
plt.grid(True)
plt.gca().set_aspect('equal')

plt.show()
