import heapq
import numpy as np
import matplotlib.pyplot as plt

# 1. CÁLCULO DE LA RUTA OFFLINE (A*)
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1 #[cite: 20]
start, goal = (2, 2), (18, 18)

def astar(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost = {start: 0}
    
    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal: break
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < 20 and 0 <= neighbor[1] < 20 and grid[neighbor[1], neighbor[0]] == 0):
                new_cost = cost[current] + 1
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + np.linalg.norm(np.array(neighbor) - np.array(goal))
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current
                    
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from.get(node, start)
    path.append(start)
    path.reverse()
    return np.array(path), np.array(list(cost.keys()))

path, explored = astar(grid, start, goal)

# 2. ANIMACIÓN DE LA EJECUCIÓN DEL PLAN
plt.figure(figsize=(8, 8))
plt.imshow(grid, origin='lower', cmap='gray_r')
plt.scatter(explored[:,0], explored[:,1], c='lightblue', s=20, alpha=0.3) # Muestra la "mente" del algoritmo
plt.plot(path[:,0], path[:,1], 'r--', alpha=0.5, label="Plan A*")

plt.scatter(start[0], start[1], c='blue', s=120, edgecolors='black', zorder=5)
plt.scatter(goal[0], goal[1], c='red', s=120, edgecolors='black', zorder=5)

plt.gca().set_aspect('equal')
plt.grid(color='gray', linestyle=':', alpha=0.7)
plt.title("Simulación Dinámica: Seguimiento de Plan A*", fontsize=14, fontweight='bold')

punto_robot, = plt.plot([], [], 'go', markersize=10, markeredgecolor='black', zorder=6, label="Robot")
plt.legend(loc='lower right')

plt.ion()
plt.show()

for i in range(len(path)):
    punto_robot.set_data([path[i, 0]], [path[i, 1]])
    plt.pause(0.08)

plt.ioff()
plt.show()
