import heapq
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. DEFINICIÓN DEL ENTORNO
# ==========================================
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1 # Obstáculo

start = (2, 2)
goal = (18, 18)

# ==========================================
# 2. ALGORITMO A*
# ==========================================
def heuristic(a, b):
    # Distancia Euclidiana
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start)) 
    
    came_from = {}
    cost = {start: 0}
    
    # Nodos explorados para análisis de rendimiento
    nodos_explorados = 0

    while open_list:
        _, current = heapq.heappop(open_list)
        nodos_explorados += 1
        
        if current == goal:
            break
            
        # Movimientos ortogonales (4-conectado)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Verificación de límites y obstáculo
            if (0 <= neighbor[0] < grid.shape[0] and
                0 <= neighbor[1] < grid.shape[1] and
                grid[neighbor[1], neighbor[0]] == 0):
                
                new_cost = cost[current] + 1 
                
                # Actualización de costos y cola de prioridad
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current
                    
    # Reconstrucción de la ruta
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from.get(node, start) # Evita errores si no hay ruta
    path.append(start)
    path.reverse()
    
    return path, nodos_explorados, list(cost.keys())

# Ejecución
path, nodos_exp, explored_nodes = astar(grid, start, goal)
path = np.array(path) 

# ==========================================
# 3. SALIDA POR TERMINAL (TABLA DE RENDIMIENTO)
# ==========================================
print("\n" + "="*50)
print(f"| {'MÉTRICA DE PLANIFICACIÓN A*':^46} |")
print("="*50)
print(f"| {'Punto de Inicio':<25} | {str(start):<18} |")
print(f"| {'Punto de Meta':<25} | {str(goal):<18} |")
print(f"| {'Longitud de la Ruta':<25} | {len(path)} celdas{' ':<7} |")
print(f"| {'Nodos Explorados':<25} | {nodos_exp} celdas{' ':<7} |")
print("-" * 50 + "\n")

# ==========================================
# 4. VISUALIZACIÓN GRÁFICA
# ==========================================
plt.figure(figsize=(8, 8))
plt.imshow(grid, origin='lower', cmap='gray_r')

# Graficar espacio explorado (opcional, visualmente educativo)
exp_arr = np.array(explored_nodes)
plt.scatter(exp_arr[:,0], exp_arr[:,1], c='lightblue', s=20, alpha=0.3, label="Espacio Explorado")

plt.plot(path[:,0], path[:,1], 'r-', linewidth=2.5, label="A* Path") 
plt.scatter(start[0], start[1], c='blue', s=120, edgecolors='black', zorder=5, label="Inicio")
plt.scatter(goal[0], goal[1], c='red', s=120, edgecolors='black', zorder=5, label="Meta")

plt.gca().set_aspect('equal')
plt.grid(color='gray', linestyle=':', linewidth=0.5, alpha=0.7)
plt.title("Planificación Global Óptima con A*", fontsize=14, fontweight='bold')
plt.legend(loc='lower right')
plt.show() 
