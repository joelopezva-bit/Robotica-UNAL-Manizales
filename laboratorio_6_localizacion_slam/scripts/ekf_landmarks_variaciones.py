import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor
from math import pi

# ==============================================================================
# 1. CONFIGURACIÓN COMPARTIDA
# ==============================================================================
V = np.diag([0.02, np.deg2rad(0.5)])**2
W = np.diag([0.1, np.deg2rad(1)])**2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
T_sim = 40

# Crear un mapa común de landmarks para ambos casos
np.random.seed(42) # Semilla para mapa consistente
mapa = LandmarkMap(20, workspace=10)

# ==============================================================================
# 2. ESCENARIO 1: LIDAR CORTO ALCANCE (MIOPE) - r = 2 metros
# ==============================================================================
robot_corto = Bicycle(covar=V, animation=None)
robot_corto.control = RandomPath(workspace=10, seed=10)

sensor_corto = RangeBearingSensor(robot=robot_corto, map=mapa, covar=W, angle=[-pi/2, pi/2], range=2)
ekf_corto = EKF(robot=(robot_corto, V), P0=P0, map=mapa, sensor=(sensor_corto, W))

print("Pre-computando EKF (LiDAR Corto)...")
ekf_corto.run(T=T_sim)
t_corto = ekf_corto.get_t()
p_norm_corto = ekf_corto.get_Pnorm()

# ==============================================================================
# 3. ESCENARIO 2: LIDAR LARGO ALCANCE - r = 6 metros
# ==============================================================================
robot_largo = Bicycle(covar=V, animation=None)
robot_largo.control = robot_corto.control # Misma ruta

sensor_largo = RangeBearingSensor(robot=robot_largo, map=mapa, covar=W, angle=[-pi/2, pi/2], range=6)
ekf_largo = EKF(robot=(robot_largo, V), P0=P0, map=mapa, sensor=(sensor_largo, W))

print("Pre-computando EKF (LiDAR Largo)...")
ekf_largo.run(T=T_sim)
p_norm_largo = ekf_largo.get_Pnorm()

# ==============================================================================
# EXTRACCIÓN SEGURA DE COORDENADAS
# ==============================================================================
fig_temp = plt.figure()
robot_corto.plot_xy()
ekf_corto.plot_xy()
l_corto = plt.gca().get_lines()
rc_x, rc_y = l_corto[0].get_xdata(), l_corto[0].get_ydata()
ec_x, ec_y = l_corto[1].get_xdata(), l_corto[1].get_ydata()
plt.clf()

robot_largo.plot_xy()
ekf_largo.plot_xy()
l_largo = plt.gca().get_lines()
rl_x, rl_y = l_largo[0].get_xdata(), l_largo[0].get_ydata()
el_x, el_y = l_largo[1].get_xdata(), l_largo[1].get_ydata()
plt.close(fig_temp)

# ==============================================================================
# 4. CONFIGURACIÓN DEL MULTIPANEL ANIMADO (1x2)
# ==============================================================================
plt.ion()
fig, (ax_corto, ax_largo) = plt.subplots(1, 2, figsize=(16, 7))

# Pintar landmarks estáticos en ambos usando el método nativo de la librería
for ax in [ax_corto, ax_largo]:
    plt.sca(ax)      # Le indicamos a matplotlib cuál es el eje activo actual
    mapa.plot()      # La librería se encarga de dibujar los landmarks correctamente
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=10)

ax_corto.set_ylabel('Y [m]', fontsize=10)

def init_lineas(ax):
    lr, = ax.plot([], [], 'b-', linewidth=2, label='Real')
    le, = ax.plot([], [], 'r--', linewidth=2, label='Estimación EKF')
    pr, = ax.plot([], [], 'bo', markersize=7)
    pe, = ax.plot([], [], 'ro', markersize=7)
    ax.legend(loc='upper left', fontsize=8)
    return lr, le, pr, pe

lr_c, le_c, pr_c, pe_c = init_lineas(ax_corto)
lr_l, le_l, pr_l, pe_l = init_lineas(ax_largo)

print("\n▶️ Renderizado en tiempo real listo.")

for k in range(len(t_corto)):
    if not plt.fignum_exists(fig.number):
        break
    
    # Animación Corto
    lr_c.set_data(rc_x[:k+1], rc_y[:k+1])
    le_c.set_data(ec_x[:k+1], ec_y[:k+1])
    pr_c.set_data([rc_x[k]], [rc_y[k]])
    pe_c.set_data([ec_x[k]], [ec_y[k]])
    ax_corto.set_title(r"LiDAR Rango = $2$m (Miope) | t=" + f"{t_corto[k]:.1f}s\n" + rf"Incertidumbre $\|P\| = {p_norm_corto[k]:.3f}$", fontsize=10, fontweight='bold')
    
    # Animación Largo
    lr_l.set_data(rl_x[:k+1], rl_y[:k+1])
    le_l.set_data(el_x[:k+1], el_y[:k+1])
    pr_l.set_data([rl_x[k]], [rl_y[k]])
    pe_l.set_data([el_x[k]], [el_y[k]])
    ax_largo.set_title(r"LiDAR Rango = $6$m (Visión Lejana) | t=" + f"{t_corto[k]:.1f}s\n" + rf"Incertidumbre $\|P\| = {p_norm_largo[k]:.3f}$", fontsize=10, fontweight='bold')
    
    plt.pause(0.04)

print("✅ Animación completada.")
plt.ioff()
plt.show()
