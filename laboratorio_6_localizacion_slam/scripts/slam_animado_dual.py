import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor
from math import pi

print("\nPre-computando simulaciones SLAM para animación fluida (esto puede tardar unos segundos)...")

# ==============================================================================
# 1. CONFIGURACIÓN COMPARTIDA
# ==============================================================================
V = np.diag([0.02, np.deg2rad(0.5)])**2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
T_sim = 40

# Crear un mapa común de landmarks
np.random.seed(42) # Semilla para asegurar la misma ubicación de las balizas
mapa = LandmarkMap(20, workspace=10)

# ==============================================================================
# 2. ESCENARIO 1: SENSOR NOMINAL (Caso Base de la Guía)
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
W_ruido = np.diag([0.5, np.deg2rad(10)])**2 # Ruido aumentado: 0.5m y 10 grados

robot_ruido = Bicycle(covar=V, animation=None)
robot_ruido.control = robot_nom.control # Usar exactamente la misma ruta

sensor_ruido = RangeBearingSensor(robot=robot_ruido, map=mapa, covar=W_ruido, angle=[-pi/2, pi/2], range=4)
slam_ruido = EKF(robot=(robot_ruido, V), P0=P0, sensor=(sensor_ruido, W_ruido), map=None)

slam_ruido.run(T=T_sim)
p_norm_ruido = slam_ruido.get_Pnorm()

# ==============================================================================
# 4. EXTRACCIÓN SEGURA DE TRAYECTORIAS
# ==============================================================================
fig_temp = plt.figure()

# Extraer Nominal
robot_nom.plot_xy()
slam_nom.plot_xy()
lineas_nom = plt.gca().get_lines()
rx_nom, ry_nom = lineas_nom[0].get_xdata(), lineas_nom[0].get_ydata()
ex_nom, ey_nom = lineas_nom[1].get_xdata(), lineas_nom[1].get_ydata()
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
# 5. CONFIGURACIÓN DEL MULTIPANEL ANIMADO (1x2)
# ==============================================================================
plt.ion()
fig, (ax_nom, ax_ruido) = plt.subplots(1, 2, figsize=(16, 7))
fig.canvas.manager.set_window_title('Animación SLAM: Impacto del Ruido del Sensor')

# Dibujar la realidad (estrellas grises) usando el método oficial
for ax in [ax_nom, ax_ruido]:
    plt.sca(ax)
    mapa.plot()
    
    # Sobre-escribimos el estilo del mapa para que quede en gris y no resalte tanto
    col_mapa = ax.collections[0]
    col_mapa.set_color('lightgray')
    col_mapa.set_label('Balizas Reales (Ocultas)')
    
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=10)

ax_nom.set_ylabel('Y [m]', fontsize=10)

# ==============================================================================
# 6. DIBUJAR MAPAS ESTIMADOS (BALIZAS VERDES) ESTÁTICAMENTE
# ==============================================================================
# El vector de estado final 'x' tiene la forma: [x, y, theta, lm1_x, lm1_y, ...]
if len(slam_nom.x) > 3:
    bx_nom = slam_nom.x[3::2]
    by_nom = slam_nom.x[4::2]
    ax_nom.scatter(bx_nom, by_nom, marker='+', color='green', s=100, label='Balizas Mapeadas (SLAM)')

if len(slam_ruido.x) > 3:
    bx_r = slam_ruido.x[3::2]
    by_r = slam_ruido.x[4::2]
    ax_ruido.scatter(bx_r, by_r, marker='+', color='green', s=100, label='Balizas Mapeadas (SLAM)')

def init_lineas_slam(ax):
    lr, = ax.plot([], [], 'b-', linewidth=2, label='Trayectoria Real')
    le, = ax.plot([], [], 'r--', linewidth=2, label='Estimación SLAM')
    pr, = ax.plot([], [], 'bo', markersize=7)
    pe, = ax.plot([], [], 'ro', markersize=7)
    ax.legend(loc='upper right', fontsize=8, frameon=True)
    return lr, le, pr, pe

lr_n, le_n, pr_n, pe_n = init_lineas_slam(ax_nom)
lr_r, le_r, pr_r, pe_r = init_lineas_slam(ax_ruido)

print("\n" + "=" * 65)
print("▶️ Renderizado en tiempo real listo.")
print("=" * 65)

for k in range(total_pasos):
    if not plt.fignum_exists(fig.number):
        break
    
    # --- Actualizar Panel Nominal ---
    lr_n.set_data(rx_nom[:k+1], ry_nom[:k+1])
    le_n.set_data(ex_nom[:k+1], ey_nom[:k+1])
    pr_n.set_data([rx_nom[k]], [ry_nom[k]])
    pe_n.set_data([ex_nom[k]], [ey_nom[k]])
    
    err_inst_nom = np.linalg.norm(np.array([rx_nom[k], ry_nom[k]]) - np.array([ex_nom[k], ey_nom[k]]))
    
    titulo_nom = (r"SLAM Nominal (Guía) | W: $\sigma_r=0.1$m, $\sigma_\theta=1^\circ$" + "\n" +
                  f"t={t_arr[k]:.1f}s | Error: {err_inst_nom:.2f} m | " + r"$\|P\| = $" + f"{p_norm_nom[k]:.2f}")
    ax_nom.set_title(titulo_nom, fontsize=10, fontweight='bold')
    
    # --- Actualizar Panel Ruidoso ---
    lr_r.set_data(rx_r[:k+1], ry_r[:k+1])
    le_r.set_data(ex_r[:k+1], ey_r[:k+1])
    pr_r.set_data([rx_r[k]], [ry_r[k]])
    pe_r.set_data([ex_r[k]], [ey_r[k]])
    
    err_inst_r = np.linalg.norm(np.array([rx_r[k], ry_r[k]]) - np.array([ex_r[k], ey_r[k]]))
    
    titulo_ruido = (r"SLAM Sensor Ruidoso | W: $\sigma_r=0.5$m, $\sigma_\theta=10^\circ$" + "\n" +
                    f"t={t_arr[k]:.1f}s | Error: {err_inst_r:.2f} m | " + r"$\|P\| = $" + f"{p_norm_ruido[k]:.2f}")
    ax_ruido.set_title(titulo_ruido, fontsize=10, fontweight='bold')
    
    plt.pause(0.04)

print("✅ Animación SLAM completada.")
plt.ioff()
plt.show()
