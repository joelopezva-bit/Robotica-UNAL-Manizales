import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor
from math import pi

print("\nPre-computando simulaciones SLAM para la animación de 3 paneles...")
print("Esto puede tardar unos segundos, por favor espera...")

# ==============================================================================
# 1. CONFIGURACIÓN COMPARTIDA
# ==============================================================================
V = np.diag([0.02, np.deg2rad(0.5)])**2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
T_sim = 40

np.random.seed(42)
mapa = LandmarkMap(20, workspace=10)

# ==============================================================================
# 2. ESCENARIO 1: SENSOR NOMINAL (Caso Base)
# ==============================================================================
W_nom = np.diag([0.1, np.deg2rad(1)])**2

robot_nom = Bicycle(covar=V, animation=None)
robot_nom.control = RandomPath(workspace=10, seed=15)
sensor_nom = RangeBearingSensor(robot=robot_nom, map=mapa, covar=W_nom, angle=[-pi/2, pi/2], range=4)

slam_nom = EKF(robot=(robot_nom, V), P0=P0, sensor=(sensor_nom, W_nom), map=None)
slam_nom.run(T=T_sim)
t_arr = slam_nom.get_t()
p_norm_nom = slam_nom.get_Pnorm()

# ==============================================================================
# 3. ESCENARIO 2: SENSOR RUIDOSO (Alta Incertidumbre en LiDAR)
# ==============================================================================
W_ruido = np.diag([0.5, np.deg2rad(10)])**2

robot_ruido = Bicycle(covar=V, animation=None)
robot_ruido.control = robot_nom.control # Usar la misma ruta
sensor_ruido = RangeBearingSensor(robot=robot_ruido, map=mapa, covar=W_ruido, angle=[-pi/2, pi/2], range=4)

slam_ruido = EKF(robot=(robot_ruido, V), P0=P0, sensor=(sensor_ruido, W_ruido), map=None)
slam_ruido.run(T=T_sim)
p_norm_ruido = slam_ruido.get_Pnorm()

# ==============================================================================
# 4. EXTRACCIÓN SEGURA DE HISTORIALES (A prueba de fallos)
# ==============================================================================
fig_temp = plt.figure()

# Extraer Nominal
robot_nom.plot_xy()
slam_nom.plot_xy()
lineas_nom = plt.gca().get_lines()
rx_n, ry_n = lineas_nom[0].get_xdata(), lineas_nom[0].get_ydata()
ex_n, ey_n = lineas_nom[1].get_xdata(), lineas_nom[1].get_ydata()
plt.clf()

# Extraer Ruidoso
robot_ruido.plot_xy()
slam_ruido.plot_xy()
lineas_ruido = plt.gca().get_lines()
rx_r, ry_r = lineas_ruido[0].get_xdata(), lineas_ruido[0].get_ydata()
ex_r, ey_r = lineas_ruido[1].get_xdata(), lineas_ruido[1].get_ydata()
plt.close(fig_temp)

total_pasos = len(t_arr)

# ==============================================================================
# 5. CONFIGURACIÓN DEL MULTIPANEL ANIMADO (1x3)
# ==============================================================================
plt.ion()
fig, (ax_nom, ax_ruido, ax_inc) = plt.subplots(1, 3, figsize=(18, 6))
fig.canvas.manager.set_window_title('Animación SLAM: Comparativa de Ruido de Sensor')

for ax in [ax_nom, ax_ruido]:
    plt.sca(ax)
    mapa.plot() # Dibuja la realidad oculta
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=10)

ax_nom.set_ylabel('Y [m]', fontsize=10)

def init_lineas_slam(ax):
    l_r, = ax.plot([], [], 'b-', linewidth=2, label='Real')
    l_e, = ax.plot([], [], 'r--', linewidth=2, label='Estimación SLAM')
    p_r, = ax.plot([], [], 'bo', markersize=6)
    p_e, = ax.plot([], [], 'ro', markersize=6)
    ax.legend(loc='upper right', fontsize=8, frameon=True)
    return l_r, l_e, p_r, p_e

lr_n, le_n, pr_n, pe_n = init_lineas_slam(ax_nom)
lr_r, le_r, pr_r, pe_r = init_lineas_slam(ax_ruido)

# Configuración del panel de incertidumbre
l_inc_n, = ax_inc.plot([], [], 'darkorange', linewidth=2.2, label='Sensor Nominal')
l_inc_r, = ax_inc.plot([], [], 'crimson', linewidth=2.2, label='Sensor Ruidoso')
p_inc_n, = ax_inc.plot([], [], marker='o', color='darkorange', markersize=5)
p_inc_r, = ax_inc.plot([], [], marker='o', color='crimson', markersize=5)

ax_inc.set_xlim(0, max(t_arr))
ax_inc.set_ylim(0, max(p_norm_ruido) * 1.1)
ax_inc.grid(True, linestyle=':', alpha=0.7)
ax_inc.set_xlabel('Tiempo [s]', fontsize=10)
ax_inc.set_ylabel(r'Norma de Covarianza $\|P\|$', fontsize=10)
ax_inc.legend(loc='upper left', fontsize=9, frameon=True)

print("\n" + "=" * 65)
print("▶️ Renderizado en tiempo real listo.")
print("=" * 65)

# ==============================================================================
# 6. BUCLE DE RENDERIZADO DINÁMICO
# ==============================================================================
for k in range(total_pasos):
    if not plt.fignum_exists(fig.number):
        break
    
    # --- Actualizar Panel Nominal ---
    lr_n.set_data(rx_n[:k+1], ry_n[:k+1])
    le_n.set_data(ex_n[:k+1], ey_n[:k+1])
    pr_n.set_data([rx_n[k]], [ry_n[k]])
    pe_n.set_data([ex_n[k]], [ey_n[k]])
    err_n = np.linalg.norm(np.array([rx_n[k], ry_n[k]]) - np.array([ex_n[k], ey_n[k]]))
    ax_nom.set_title(r"SLAM Nominal | W: $\sigma_r=0.1$m, $\sigma_\theta=1^\circ$" + f"\nt={t_arr[k]:.1f}s | Error: {err_n:.2f} m", fontsize=10, fontweight='bold')
    
    # --- Actualizar Panel Ruidoso ---
    lr_r.set_data(rx_r[:k+1], ry_r[:k+1])
    le_r.set_data(ex_r[:k+1], ey_r[:k+1])
    pr_r.set_data([rx_r[k]], [ry_r[k]])
    pe_r.set_data([ex_r[k]], [ey_r[k]])
    err_r = np.linalg.norm(np.array([rx_r[k], ry_r[k]]) - np.array([ex_r[k], ey_r[k]]))
    ax_ruido.set_title(r"SLAM Sensor Ruidoso | W: $\sigma_r=0.5$m, $\sigma_\theta=10^\circ$" + f"\nt={t_arr[k]:.1f}s | Error: {err_r:.2f} m", fontsize=10, fontweight='bold')
    
    # --- Actualizar Panel de Incertidumbre ---
    l_inc_n.set_data(t_arr[:k+1], p_norm_nom[:k+1])
    p_inc_n.set_data([t_arr[k]], [p_norm_nom[k]])
    l_inc_r.set_data(t_arr[:k+1], p_norm_ruido[:k+1])
    p_inc_r.set_data([t_arr[k]], [p_norm_ruido[k]])
    ax_inc.set_title(rf"Evolución de Incertidumbre $\|P\|$" + "\n" + rf"Nominal: {p_norm_nom[k]:.2f} | Ruidoso: {p_norm_ruido[k]:.2f}", fontsize=10, fontweight='bold')
    
    plt.pause(0.04)

print("✅ Animación completada.")
plt.ioff()
plt.show()
