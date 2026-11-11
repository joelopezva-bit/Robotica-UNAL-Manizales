import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath

print("\nPre-computando simulaciones EKF para animación fluida...")

# ==============================================================================
# 1. CONFIGURACIÓN DE ESCENARIOS
# ==============================================================================
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
T_sim = 20

# Escenario B: Nominal (Guía)
V_nom = np.diag([0.020, np.deg2rad(0.50)])**2
robot_nom = Bicycle(covar=V_nom, animation=None)
robot_nom.control = RandomPath(workspace=10)
ekf_nom = EKF(robot=(robot_nom, V_nom), P0=P0)
ekf_nom.run(T=T_sim)
t_arr = ekf_nom.get_t()
p_norm_nom = ekf_nom.get_Pnorm()

# Escenario C: Alto Ruido (Terreno Irregular)
# Forzamos la misma semilla de RandomPath para que intenten hacer la misma ruta
V_al = np.diag([0.080, np.deg2rad(1.50)])**2
robot_al = Bicycle(covar=V_al, animation=None)
robot_al.control = robot_nom.control 
ekf_al = EKF(robot=(robot_al, V_al), P0=P0)
ekf_al.run(T=T_sim)
p_norm_al = ekf_al.get_Pnorm()

# ==============================================================================
# 2. EXTRACCIÓN SEGURA DE HISTORIALES (A prueba de fallos de la librería)
# ==============================================================================
fig_temp = plt.figure()

# Extraer Nominal
robot_nom.plot_xy()
ekf_nom.plot_xy()
lineas_nom = plt.gca().get_lines()
rx_nom, ry_nom = lineas_nom[0].get_xdata(), lineas_nom[0].get_ydata()
ex_nom, ey_nom = lineas_nom[1].get_xdata(), lineas_nom[1].get_ydata()
plt.clf()

# Extraer Alto Ruido
robot_al.plot_xy()
ekf_al.plot_xy()
lineas_al = plt.gca().get_lines()
rx_al, ry_al = lineas_al[0].get_xdata(), lineas_al[0].get_ydata()
ex_al, ey_al = lineas_al[1].get_xdata(), lineas_al[1].get_ydata()

plt.close(fig_temp)

total_pasos = len(t_arr)

# ==============================================================================
# 3. CONFIGURACIÓN DE VENTANA MULTIPANEL (1x3)
# ==============================================================================
plt.ion()
fig, (ax_nom, ax_al, ax_inc) = plt.subplots(1, 3, figsize=(18, 6))
fig.canvas.manager.set_window_title('Animación EKF: Comparativa de Ruido')

def configurar_eje_tray(ax, x_todos, y_todos):
    margen = 2.0
    ax.set_xlim(min(x_todos) - margen, max(x_todos) + margen)
    ax.set_ylim(min(y_todos) - margen, max(y_todos) + margen)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=10)
    ax.set_ylabel('Y [m]', fontsize=10)
    
    lr, = ax.plot([], [], 'b-', linewidth=2, label='Trayectoria Real')
    le, = ax.plot([], [], 'r--', linewidth=2, label='Estimación EKF')
    pr, = ax.plot([], [], 'bo', markersize=7)
    pe, = ax.plot([], [], 'ro', markersize=7)
    ax.legend(loc='upper right', fontsize=8, frameon=True)
    return lr, le, pr, pe

# Configurar Ejes de Trayectoria
l_r_nom, l_e_nom, p_r_nom, p_e_nom = configurar_eje_tray(ax_nom, np.concatenate([rx_nom, ex_nom]), np.concatenate([ry_nom, ey_nom]))
l_r_al, l_e_al, p_r_al, p_e_al = configurar_eje_tray(ax_al, np.concatenate([rx_al, ex_al]), np.concatenate([ry_al, ey_al]))

# Configurar Eje de Incertidumbre
l_inc_nom, = ax_inc.plot([], [], 'darkorange', linewidth=2.2, label='Nominal (Guía)')
l_inc_al, = ax_inc.plot([], [], 'crimson', linewidth=2.2, label='Alto Ruido')
p_inc_nom, = ax_inc.plot([], [], marker='o', color='darkorange', markersize=6)
p_inc_al, = ax_inc.plot([], [], marker='o', color='crimson', markersize=6)

ax_inc.set_xlim(0, max(t_arr))
ax_inc.set_ylim(0, max(p_norm_al) * 1.15)
ax_inc.grid(True, linestyle=':', alpha=0.7)
ax_inc.set_xlabel('Tiempo [s]', fontsize=10)
ax_inc.set_ylabel(r'Norma de Covarianza $\|P\|$', fontsize=10)
ax_inc.legend(loc='upper left', fontsize=9, frameon=True)

# ==============================================================================
# 4. BUCLE DE RENDERIZADO DINÁMICO
# ==============================================================================
print("\n" + "=" * 65)
print("▶️ Renderizado en tiempo real listo.")
print("=" * 65)

for k in range(total_pasos):
    if not plt.fignum_exists(fig.number):
        break
    
    # Actualizar Nominal
    l_r_nom.set_data(rx_nom[:k+1], ry_nom[:k+1])
    l_e_nom.set_data(ex_nom[:k+1], ey_nom[:k+1])
    p_r_nom.set_data([rx_nom[k]], [ry_nom[k]])
    p_e_nom.set_data([ex_nom[k]], [ey_nom[k]])
    err_inst_nom = np.linalg.norm(np.array([rx_nom[k], ry_nom[k]]) - np.array([ex_nom[k], ey_nom[k]]))
    ax_nom.set_title(f"Nominal | t={t_arr[k]:.1f}s\nError Estimación: {err_inst_nom:.2f} m", fontsize=10, fontweight='bold')
    
    # Actualizar Alto Ruido
    l_r_al.set_data(rx_al[:k+1], ry_al[:k+1])
    l_e_al.set_data(ex_al[:k+1], ey_al[:k+1])
    p_r_al.set_data([rx_al[k]], [ry_al[k]])
    p_e_al.set_data([ex_al[k]], [ey_al[k]])
    err_inst_al = np.linalg.norm(np.array([rx_al[k], ry_al[k]]) - np.array([ex_al[k], ey_al[k]]))
    ax_al.set_title(f"Alto Ruido | t={t_arr[k]:.1f}s\nError Estimación: {err_inst_al:.2f} m", fontsize=10, fontweight='bold')
    
    # Actualizar Incertidumbres
    l_inc_nom.set_data(t_arr[:k+1], p_norm_nom[:k+1])
    p_inc_nom.set_data([t_arr[k]], [p_norm_nom[k]])
    
    l_inc_al.set_data(t_arr[:k+1], p_norm_al[:k+1])
    p_inc_al.set_data([t_arr[k]], [p_norm_al[k]])
    
    ax_inc.set_title(rf"Evolución de Incertidumbre $\|P\|$" + "\n" + rf"Nominal: {p_norm_nom[k]:.2f} | Alto Ruido: {p_norm_al[k]:.2f}", fontsize=10, fontweight='bold')
    
    plt.pause(0.04)

print("✅ Animación completada.")
plt.ioff()
plt.show()
