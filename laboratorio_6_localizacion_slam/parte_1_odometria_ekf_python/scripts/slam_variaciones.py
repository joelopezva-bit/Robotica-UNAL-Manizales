import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor
from math import pi

V = np.diag([0.02, np.deg2rad(0.5)])**2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
T_sim = 40

np.random.seed(42)
mapa = LandmarkMap(20, workspace=10)

# Escenario 1: Sensor de Alta Precisión (Bajo Ruido W)
W_preciso = np.diag([0.01, np.deg2rad(0.1)])**2
robot_p = Bicycle(covar=V, animation=None)
robot_p.control = RandomPath(workspace=10, seed=20)
sensor_p = RangeBearingSensor(robot=robot_p, map=mapa, covar=W_preciso, angle=[-pi/2, pi/2], range=4)
slam_preciso = EKF(robot=(robot_p, V), P0=P0, sensor=(sensor_p, W_preciso), map=None)
slam_preciso.run(T=T_sim)

# Escenario 2: Sensor Ruidoso (Alto Ruido W)
W_ruido = np.diag([0.5, np.deg2rad(10)])**2
robot_r = Bicycle(covar=V, animation=None)
robot_r.control = robot_p.control # Misma ruta
sensor_r = RangeBearingSensor(robot=robot_r, map=mapa, covar=W_ruido, angle=[-pi/2, pi/2], range=4)
slam_ruidoso = EKF(robot=(robot_r, V), P0=P0, sensor=(sensor_r, W_ruido), map=None)
slam_ruidoso.run(T=T_sim)

# Extracción de coordenadas a prueba de fallos
def extraer_datos(robot, slam):
    fig_temp = plt.figure()
    robot.plot_xy()
    slam.plot_xy()
    lineas = plt.gca().get_lines()
    rx, ry = lineas[0].get_xdata(), lineas[0].get_ydata()
    ex, ey = lineas[1].get_xdata(), lineas[1].get_ydata()
    plt.close(fig_temp)
    return rx, ry, ex, ey

rx_p, ry_p, ex_p, ey_p = extraer_datos(robot_p, slam_preciso)
rx_r, ry_r, ex_r, ey_r = extraer_datos(robot_r, slam_ruidoso)

# Configuración Multipanel
fig = plt.figure(figsize=(16, 10))
gs = fig.add_gridspec(2, 2)
ax_p = fig.add_subplot(gs[0, 0])
ax_r = fig.add_subplot(gs[0, 1])
ax_inc = fig.add_subplot(gs[1, :])

for ax in [ax_p, ax_r]:
    plt.sca(ax)
    mapa.plot()
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_xlabel('X [m]', fontsize=9)

# Panel Preciso
ax_p.plot(rx_p, ry_p, 'b-', linewidth=2, label='Real')
ax_p.plot(ex_p, ey_p, 'g--', linewidth=2, label='Estimación SLAM')
err_p = np.linalg.norm(np.array([rx_p[-1], ry_p[-1]]) - np.array([ex_p[-1], ey_p[-1]]))
ax_p.set_title(rf"Sensor Preciso (W bajo) | $\sigma_r=0.01$m, $\sigma_\theta=0.1^\circ$"+f"\nError Final: {err_p:.2f}m", fontsize=10, fontweight='bold')
ax_p.legend(loc='best', fontsize=8)

# Panel Ruidoso
ax_r.plot(rx_r, ry_r, 'b-', linewidth=2, label='Real')
ax_r.plot(ex_r, ey_r, 'r--', linewidth=2, label='Estimación SLAM')
err_r = np.linalg.norm(np.array([rx_r[-1], ry_r[-1]]) - np.array([ex_r[-1], ey_r[-1]]))
ax_r.set_title(rf"Sensor Ruidoso (W alto) | $\sigma_r=0.5$m, $\sigma_\theta=10^\circ$"+f"\nError Final: {err_r:.2f}m", fontsize=10, fontweight='bold')
ax_r.legend(loc='best', fontsize=8)

# Panel Incertidumbre
t_arr = slam_preciso.get_t()
ax_inc.plot(t_arr, slam_preciso.get_Pnorm(), 'g-', linewidth=2, label='SLAM Sensor Preciso')
ax_inc.plot(t_arr, slam_ruidoso.get_Pnorm(), 'r-', linewidth=2, label='SLAM Sensor Ruidoso')
ax_inc.set_title(r'Comparativa de Incertidumbre $\|P\|_2$', fontsize=11, fontweight='bold')
ax_inc.set_xlabel('Tiempo [s]')
ax_inc.set_ylabel(r'Norma $\|P\|$')
ax_inc.grid(True, linestyle=':', alpha=0.7)
ax_inc.legend(loc='upper left', frameon=True)

plt.tight_layout()
plt.savefig('slam_variaciones_ruido.png', dpi=300)
print("✅ Gráfica comparativa de SLAM guardada.")
plt.show()
