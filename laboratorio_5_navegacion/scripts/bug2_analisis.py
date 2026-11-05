import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. ENCAPSULAMIENTO DEL ALGORITMO BUG2
# ==========================================
def simular_bug2(grid, start, goal, max_iter=800):
    pos = start.copy()
    path = [pos.copy()]
    
    def is_free(p):
        if 0 <= p[0] < grid.shape[1] and 0 <= p[1] < grid.shape[0]:
            return grid[p[1], p[0]] == 0
        return False

    for _ in range(max_iter):
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
    return np.array(path)

# ==========================================
# 2. DEFINICIÓN DE ESCENARIOS (VARIACIÓN)
# ==========================================
# Escenario 1: Base original de la guía
grid_base = np.zeros((20, 20))
grid_base[8:12, 5:15] = 1  
start_1, goal_1 = np.array([2, 2]), np.array([18, 18])
path_1 = simular_bug2(grid_base, start_1, goal_1)

# Escenario 2: Variación de Coordenadas (Requisito Profesor)
grid_var = np.zeros((20, 20))
grid_var[8:12, 5:15] = 1  
start_2, goal_2 = np.array([2, 17]), np.array([18, 3])
path_2 = simular_bug2(grid_var, start_2, goal_2)

# Escenario 3: Trampa en U (Entorno Complejo)
grid_trampa = np.zeros((20, 20))
grid_trampa[6:14, 12:14] = 1  
grid_trampa[12:14, 8:14] = 1  
grid_trampa[6:8, 8:14] = 1    
start_3, goal_3 = np.array([2, 10]), np.array([18, 10])
path_3 = simular_bug2(grid_trampa, start_3, goal_3)

# ==========================================
# 3. VISUALIZACIÓN MULTIPANEL Y ANIMACIÓN
# ==========================================
print("\nGenerando evidencia gráfica con variaciones de parámetros (Bug2)...")
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

def configurar_subplot(ax, grid, path, start, goal, titulo):
    ax.imshow(grid, origin='lower', cmap='gray_r')
    # Línea-m directa
    ax.plot([start[0], goal[0]], [start[1], goal[1]], 'r--', alpha=0.3)
    # Estela tenue de la ruta para la imagen estática
    ax.plot(path[:,0], path[:,1], 'g-', linewidth=2, alpha=0.3)
    
    ax.scatter(start[0], start[1], c='blue', s=80, edgecolors='black', zorder=5)
    ax.scatter(goal[0], goal[1], c='red', s=80, edgecolors='black', zorder=5)
    ax.set_title(titulo, fontsize=11, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(color='gray', linestyle=':', linewidth=0.5, alpha=0.7)
    
    # Objeto del robot para animar
    punto_robot, = ax.plot([], [], 'go', markersize=10, markeredgecolor='white', zorder=6)
    linea_animada, = ax.plot([], [], 'g-', linewidth=2.5)
    return punto_robot, linea_animada

p1, l1 = configurar_subplot(ax1, grid_base, path_1, start_1, goal_1, f"Escenario 1: Base\n({len(path_1)} pasos)")
p2, l2 = configurar_subplot(ax2, grid_var, path_2, start_2, goal_2, f"Escenario 2: Var. Coordenadas\n({len(path_2)} pasos)")
p3, l3 = configurar_subplot(ax3, grid_trampa, path_3, start_3, goal_3, f"Escenario 3: Trampa U\n¡Atrapado! ({len(path_3)} pasos)")

plt.tight_layout()

# Guardar imagen fija para el informe
plt.savefig("bug2_analisis_comparativo.png", dpi=300, bbox_inches='tight')
print("✅ Imagen estática guardada exitosamente como 'bug2_analisis_comparativo.png'.")

# ==========================================
# 4. BUCLE DINÁMICO INFINITO
# ==========================================
print("▶️ Iniciando comportamiento dinámico. Cierra la ventana gráfica para detener.")
plt.ion()
plt.show()

max_pasos = max(len(path_1), len(path_2), len(path_3))

while plt.fignum_exists(fig.number):
    for i in range(max_pasos):
        if not plt.fignum_exists(fig.number): break
        
        if i < len(path_1): 
            p1.set_data([path_1[i, 0]], [path_1[i, 1]])
            l1.set_data(path_1[:i+1, 0], path_1[:i+1, 1])
            
        if i < len(path_2): 
            p2.set_data([path_2[i, 0]], [path_2[i, 1]])
            l2.set_data(path_2[:i+1, 0], path_2[:i+1, 1])
            
        if i < len(path_3): 
            p3.set_data([path_3[i, 0]], [path_3[i, 1]])
            l3.set_data(path_3[:i+1, 0], path_3[:i+1, 1])
            
        plt.pause(0.04) # Ajusta la velocidad aquí
        
    if plt.fignum_exists(fig.number):
        plt.pause(1.5)

plt.ioff()
