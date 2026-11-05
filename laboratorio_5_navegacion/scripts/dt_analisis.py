import numpy as np
import matplotlib.pyplot as plt

def calcular_dt_y_ruta(grid, start, goal, iteraciones=100):
    # 1. Calcular Transformada de Distancia
    dist = np.ones_like(grid) * np.inf
    dist[goal[1], goal[0]] = 0
    
    for _ in range(iteraciones):
        for x in range(grid.shape[1]):
            for y in range(grid.shape[0]):
                if grid[y, x] == 0:
                    neighbors = []
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                            neighbors.append(dist[ny, nx])
                    dist[y, x] = min(dist[y, x], min(neighbors) + 1)
                    
    # 2. Calcular Ruta (Descenso de gradiente)
    pos = start
    path = [pos]
    for _ in range(200):
        x, y = pos
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                neighbors.append(((nx, ny), dist[ny, nx]))
        
        pos = min(neighbors, key=lambda item: item[1])[0]
        path.append(pos)
        if pos == goal:
            break
            
    return dist, np.array(path)

# ==========================================
# ESCENARIOS (VARIACIÓN DE PARÁMETROS)
# ==========================================
# Escenario 1: Base de la guía
grid_base = np.zeros((20, 20))
grid_base[8:12, 5:15] = 1
dist_1, path_1 = calcular_dt_y_ruta(grid_base, (18, 18), (2, 2))

# Escenario 2: Trampa en U (Donde Bug2 falla, DT lo resuelve)
grid_u = np.zeros((20, 20))
grid_u[6:14, 10:12] = 1
grid_u[12:14, 6:12] = 1
grid_u[6:8, 6:12] = 1
dist_2, path_2 = calcular_dt_y_ruta(grid_u, (18, 10), (2, 10))

# Escenario 3: Variación Compleja (Laberinto)
grid_var = np.zeros((20, 20))
grid_var[5:15, 5:7] = 1
grid_var[10:12, 5:15] = 1
grid_var[2:8, 14:16] = 1
dist_3, path_3 = calcular_dt_y_ruta(grid_var, (2, 18), (18, 2))

# ==========================================
# VISUALIZACIÓN MULTIPANEL Y ANIMACIÓN
# ==========================================
print("\nGenerando comparativa gráfica y calculando campos de distancia...")
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

def configurar_subplot(ax, grid, dist, path, titulo, show_quiver=False):
    ax.imshow(dist, origin='lower', cmap='hot')
    obs_y, obs_x = np.where(grid == 1)
    ax.scatter(obs_x, obs_y, c='black', s=20)
    
    # Campo vectorial (flechas) para evidenciar por qué no hay mínimos locales
    if show_quiver:
        Y, X = np.mgrid[0:20, 0:20]
        vY, vX = np.gradient(dist)
        ax.quiver(X, Y, -vX, -vY, color='white', alpha=0.5, scale=40)

    # Ruta tenue de fondo
    ax.plot(path[:, 0], path[:, 1], 'cyan', linewidth=2, alpha=0.3)
    ax.scatter(path[0, 0], path[0, 1], c='blue', s=80, edgecolors='white', zorder=5)
    ax.scatter(path[-1, 0], path[-1, 1], c='green', s=80, edgecolors='white', zorder=5)
    
    ax.set_title(titulo, fontsize=11, fontweight='bold')
    ax.set_aspect('equal')
    
    punto_robot, = ax.plot([], [], 'go', markersize=10, markeredgecolor='white', zorder=6)
    return punto_robot

p1 = configurar_subplot(ax1, grid_base, dist_1, path_1, "Escenario 1: Base\n(Muestra Gradiente Vectorial)", show_quiver=True)
p2 = configurar_subplot(ax2, grid_u, dist_2, path_2, "Escenario 2: Trampa en U\n(Resuelve mínimos locales)")
p3 = configurar_subplot(ax3, grid_var, dist_3, path_3, "Escenario 3: Laberinto\n(Variación de parámetros)")

plt.tight_layout()

# Guardar la imagen estática para el informe antes de arrancar el video
plt.savefig("dt_analisis_comparativo.png", dpi=300, bbox_inches='tight')
print("✅ Imagen estática guardada como 'dt_analisis_comparativo.png'.")

# BUCLE DINÁMICO INFINITO
print("▶️ Iniciando comportamiento dinámico. Cierra la ventana gráfica para detener.")
plt.ion()
plt.show()

max_pasos = max(len(path_1), len(path_2), len(path_3))

while plt.fignum_exists(fig.number):
    for i in range(max_pasos):
        if not plt.fignum_exists(fig.number): 
            break
            
        if i < len(path_1): p1.set_data([path_1[i, 0]], [path_1[i, 1]])
        if i < len(path_2): p2.set_data([path_2[i, 0]], [path_2[i, 1]])
        if i < len(path_3): p3.set_data([path_3[i, 0]], [path_3[i, 1]])
        
        plt.pause(0.06)
    
    # Pausa al llegar a la meta antes de reiniciar
    if plt.fignum_exists(fig.number):
        plt.pause(1.5)

plt.ioff()
