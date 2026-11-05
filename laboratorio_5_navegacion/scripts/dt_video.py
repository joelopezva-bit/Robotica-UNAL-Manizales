import numpy as np
import matplotlib.pyplot as plt

# 1. CÁLCULO PREVIO DE LA TRAYECTORIA Y DISTANCIAS
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1 
start, goal = (18, 18), (2, 2)

dist = np.ones_like(grid) * np.inf
dist[goal[1], goal[0]] = 0

for _ in range(100):
    for x in range(20):
        for y in range(20):
            if grid[y, x] == 0:
                neighbors = [dist[y+dy, x+dx] for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)] 
                             if 0 <= x+dx < 20 and 0 <= y+dy < 20]
                dist[y, x] = min(dist[y, x], min(neighbors) + 1)

pos = start
path = [pos]
for _ in range(200):
    x, y = pos
    neighbors = [((x+dx, y+dy), dist[y+dy, x+dx]) for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)] 
                 if 0 <= x+dx < 20 and 0 <= y+dy < 20]
    pos = min(neighbors, key=lambda item: item[1])[0]
    path.append(pos)
    if pos == goal: break

path = np.array(path)

# 2. ANIMACIÓN DINÁMICA
plt.figure(figsize=(6, 6))
plt.imshow(dist, origin='lower', cmap='hot')
obs_y, obs_x = np.where(grid == 1)
plt.scatter(obs_x, obs_y, c='black', s=20)
plt.plot(path[:, 0], path[:, 1], 'cyan', linewidth=2, alpha=0.3, label="Ruta Planeada")

plt.scatter(start[0], start[1], c='blue', s=80, label="Inicio", edgecolors='white', zorder=5)
plt.scatter(goal[0], goal[1], c='green', s=80, label="Meta", edgecolors='white', zorder=5)

plt.gca().set_aspect('equal')
plt.title("Simulación Dinámica: Distance Transform", fontsize=12, fontweight='bold')
plt.colorbar(label="Distancia al objetivo")

punto_robot, = plt.plot([], [], 'go', markersize=10, markeredgecolor='white', zorder=6, label="Robot")
plt.legend(loc='upper right')

plt.ion()
plt.show()

for i in range(len(path)):
    punto_robot.set_data([path[i, 0]], [path[i, 1]])
    plt.pause(0.08)

plt.ioff()
plt.show()
