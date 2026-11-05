import heapq
import numpy as np
import matplotlib.pyplot as plt
import os

def astar_modificado(grid, start, goal, tipo_heuristica='euclidiana', peso=1.0):
    open_list = []
    heapq.heappush(open_list, (0, start)) #[cite: 20]
    came_from = {}
    cost = {start: 0}
    
    def calc_heuristica(a, b):
        if tipo_heuristica == 'cero': 
            return 0
        elif tipo_heuristica == 'euclidiana': 
            return np.linalg.norm(np.array(a) - np.array(b)) * peso #[cite: 20]
            
    while open_list:
        _, current = heapq.heappop(open_list) #[cite: 21]
        if current == goal: break
            
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]: #[cite: 21]
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0] and grid[neighbor[1], neighbor[0]] == 0):
                new_cost = cost[current] + 1 #[cite: 21]
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + calc_heuristica(neighbor, goal)
                    heapq.heappush(open_list, (priority, neighbor)) #[cite: 22]
                    came_from[neighbor] = current
                    
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from.get(node, start)
    path.append(start)
    path.reverse()
    return np.array(path), np.array(list(cost.keys()))

# ==========================================
# 1. ESCENARIOS DE PRUEBA
# ==========================================
grid_u = np.zeros((20, 20))
grid_u[6:14, 10:12] = 1; grid_u[12:14, 6:12] = 1; grid_u[6:8, 6:12] = 1
start_1, goal_1 = (2, 10), (18, 10)
path_1, exp_1 = astar_modificado(grid_u, start_1, goal_1, 'euclidiana')

grid_base = np.zeros((20, 20)); grid_base[8:12, 5:15] = 1 #[cite: 20]
start_2, goal_2 = (2, 2), (18, 18)
path_2, exp_2 = astar_modificado(grid_base, start_2, goal_2, 'cero')
path_3, exp_3 = astar_modificado(grid_base, start_2, goal_2, 'euclidiana', peso=5.0)

# ==========================================
# 2. CONFIGURACIÓN DE GRÁFICOS ESTÁTICOS
# ==========================================
print("\nGenerando escenarios...")
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

def plot_escenario(ax, grid, path, exp, start, goal, titulo):
    ax.imshow(grid, origin='lower', cmap='gray_r')
    ax.scatter(exp[:,0], exp[:,1], c='lightblue', s=20, alpha=0.4)
    ax.plot(path[:,0], path[:,1], 'r--', alpha=0.5) # Ruta como guía tenue
    ax.scatter(start[0], start[1], c='blue', s=80, edgecolors='black', zorder=5)
    ax.scatter(goal[0], goal[1], c='red', s=80, edgecolors='black', zorder=5)
    ax.set_title(titulo, fontsize=11, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(color='gray', linestyle=':', alpha=0.5)
    
    # Retorna el objeto del punto (robot) para animarlo luego
    punto_robot, = ax.plot([], [], 'go', markersize=10, markeredgecolor='black', zorder=6)
    return punto_robot

p1 = plot_escenario(ax1, grid_u, path_1, exp_1, start_1, goal_1, f"Var. Coord + Trampa U\n(Explorados: {len(exp_1)})")
p2 = plot_escenario(ax2, grid_base, path_2, exp_2, start_2, goal_2, f"P1/P2: Sin Heurística (UCS)\n(Explorados: {len(exp_2)})")
p3 = plot_escenario(ax3, grid_base, path_3, exp_3, start_2, goal_2, f"P3: Heurística Inadecuada\n(Explorados: {len(exp_3)})")

plt.tight_layout()

# Guardar la imagen fija para el informe antes de iniciar el video
img_name = "astar_analisis_comparativo.png"
plt.savefig(img_name, dpi=300, bbox_inches='tight')
print(f"✅ Imagen estática guardada exitosamente como '{img_name}'.")

# ==========================================
# 3. BUCLE DE ANIMACIÓN INFINITO
# ==========================================
print("▶️ Iniciando simulación dinámica. Cierra la ventana gráfica para detener.")
plt.ion()
plt.show()

max_pasos = max(len(path_1), len(path_2), len(path_3))

# Bucle infinito que se detiene solo si cierras la ventana
while plt.fignum_exists(fig.number):
    for i in range(max_pasos):
        # Evitar errores si cierras la ventana a mitad de la animación
        if not plt.fignum_exists(fig.number): 
            break
            
        # Actualizar Robot 1
        if i < len(path_1): p1.set_data([path_1[i, 0]], [path_1[i, 1]])
        # Actualizar Robot 2
        if i < len(path_2): p2.set_data([path_2[i, 0]], [path_2[i, 1]])
        # Actualizar Robot 3
        if i < len(path_3): p3.set_data([path_3[i, 0]], [path_3[i, 1]])
        
        plt.pause(0.06) # Velocidad de la animación
        
    # Pausa de 1.5 segundos al terminar la ruta antes de reiniciar el bucle
    if plt.fignum_exists(fig.number):
        plt.pause(1.5)

plt.ioff()
