import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. DEFINICIÓN DEL ENTORNO
# ==========================================
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1

start = np.array([2, 2])
goal = np.array([18, 18])

pos = start.copy()
path = [pos.copy()]

def is_free(p):
    return grid[p[1], p[0]] == 0

# ==========================================
# 2. CABECERA DE LA TABLA RESUMIDA
# ==========================================
print("\n" + "="*55)
print(f"| {'Paso':^6} | {'Coordenada':^12} | {'Evento Registrado':^25} |")
print("="*55)
print(f"| {'0':^6} | [{pos[0]:2d}, {pos[1]:2d}]     | {'Inicio de ruta':^25} |")

# Estado anterior para detectar cambios (inicia asumiendo que está libre)
estado_anterior_libre = True 

# ==========================================
# 3. BUCLE DE NAVEGACIÓN Y REGISTRO DE EVENTOS
# ==========================================
for paso in range(1, 501):
    direction = goal - pos
    
    if abs(direction[0]) > abs(direction[1]):
        step = np.array([np.sign(direction[0]), 0])
    else:
        step = np.array([0, np.sign(direction[1])])
        
    next_pos = pos + step
    estado_actual_libre = is_free(next_pos)
    
    # --- LOGICA DE IMPRESIÓN RESUMIDA ---
    if not estado_actual_libre and estado_anterior_libre:
        print(f"| {paso:^6} | [{pos[0]:2d}, {pos[1]:2d}]     | {'Obstáculo detectado':^25} |")
    elif estado_actual_libre and not estado_anterior_libre:
        print(f"| {paso:^6} | [{pos[0]:2d}, {pos[1]:2d}]     | {'Retoma ruta directa':^25} |")
        
    estado_anterior_libre = estado_actual_libre
    # ------------------------------------
    
    if estado_actual_libre:
        pos = next_pos
    else:
        for move in [np.array([1,0]), np.array([0,1]), np.array([-1,0]), np.array([0,-1])]:
            if is_free(pos + move):
                pos = pos + move
                break
                
    path.append(pos.copy())
    
    if np.linalg.norm(pos - goal) < 1:
        print(f"| {paso:^6} | [{pos[0]:2d}, {pos[1]:2d}]     | {'Meta alcanzada':^25} |")
        break

print("-" * 55 + "\n")
path = np.array(path)

# ==========================================
# 4. VISUALIZACIÓN GRÁFICA MEJORADA
# ==========================================
plt.figure(figsize=(8, 8))
plt.imshow(grid, origin='lower', cmap='gray_r') 
plt.plot([start[0], goal[0]], [start[1], goal[1]], color='red', linestyle='--', alpha=0.4, label="Línea-m (Directa)")
plt.plot(path[:,0], path[:,1], 'g-', linewidth=2.5, label="Trayectoria Bug2") 
plt.scatter(start[0], start[1], c='blue', s=120, edgecolors='black', zorder=5, label="Inicio") 
plt.scatter(goal[0], goal[1], c='red', s=120, edgecolors='black', zorder=5, label="Meta") 

plt.gca().set_aspect('equal') #
plt.grid(color='gray', linestyle=':', linewidth=0.5, alpha=0.7) 
plt.title("Bug2 Simplificado - Navegación Reactiva", fontsize=14, fontweight='bold')
plt.legend(loc='lower right', shadow=True)
plt.show() 
