import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle

# ==============================================================================
# 1. CONFIGURACIÓN DE MODELOS Y MATRICES DE COVARIANZA
# ==============================================================================
# Parámetros de ruido nominal[cite: 307]
sigma_d_nom = 0.20
sigma_th_nom = np.deg2rad(0.5)
V_nom = np.diag([sigma_d_nom, sigma_th_nom])**2

# Parámetros de ruido alto (deslizamiento severo)
sigma_d_alto = 0.50
sigma_th_alto = np.deg2rad(2.0)
V_alto = np.diag([sigma_d_alto, sigma_th_alto])**2

# Instanciación de robots[cite: 307]
robot_ideal = Bicycle(covar=np.zeros((2, 2)))
robot_nom = Bicycle(covar=V_nom)
robot_alto = Bicycle(covar=V_alto)

qp_ideal = [0.0, 0.0, 0.0]
qp_nom = [0.0, 0.0, 0.0]
qp_alto = [0.0, 0.0, 0.0]

u = (1.0, 0.2)  # Control constante: (v = 1.0 m/s, delta = 0.2 rad)[cite: 307]
pasos = 200

# ==============================================================================
# 2. PRE-COMPUTAR TRAYECTORIAS (Garantiza fluidez en la máquina virtual)
# ==============================================================================
x_id, y_id = [], []
x_nom, y_nom = [], []
x_al, y_al = [], []

for _ in range(pasos):
    # Paso ideal
    odo_id = robot_ideal.step(u)
    qp_ideal = robot_ideal.f(qp_ideal, odo_id)
    x_id.append(qp_ideal[0])
    y_id.append(qp_ideal[1])

    # Paso nominal
    odo_nom = robot_nom.step(u)
    qp_nom = robot_nom.f(qp_nom, odo_nom)
    x_nom.append(qp_nom[0])
    y_nom.append(qp_nom[1])

    # Paso ruido alto
    odo_al = robot_alto.step(u)
    qp_alto = robot_alto.f(qp_alto, odo_al)
    x_al.append(qp_alto[0])
    y_al.append(qp_alto[1])

# ==============================================================================
# 3. CONFIGURACIÓN DE LA VENTANA GRÁFICA MULTIPANEL
# ==============================================================================
plt.ion()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# Límites automáticos con margen de seguridad
todos_x = x_id + x_nom + x_al
todos_y = y_id + y_nom + y_al
margen = 3.0
xlims = (min(todos_x) - margen, max(todos_x) + margen)
ylims = (min(todos_y) - margen, max(todos_y) + margen)

def configurar_eje(ax, titulo_base, color_real, label_real):
    ax.set_xlim(xlims)
    ax.set_ylim(ylims)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=10)
    ax.set_ylabel('Y [m]', fontsize=10)
    
    line_id, = ax.plot([], [], 'b--', linewidth=2, label='Trayectoria Ideal')
    line_re, = ax.plot([], [], color=color_real, linewidth=2, label=label_real)
    pt_id, = ax.plot([], [], 'bo', markersize=7)
    pt_re, = ax.plot([], [], marker='o', color=color_real, markersize=7)
    
    ax.scatter([0], [0], color='black', s=70, zorder=5, label='Inicio (0,0)')
    ax.legend(loc='upper right', fontsize=8)
    return line_id, line_re, pt_id, pt_re

l_id1, l_nom, p_id1, p_nom = configurar_eje(
    ax1, "Condiciones Nominales", "orange", "Odometría Nominal"
)
l_id2, l_al, p_id2, p_al = configurar_eje(
    ax2, "Ruido Alto (Derrape)", "crimson", "Odometría Ruido Alto"
)

# ==============================================================================
# 4. BUCLE DE RENDERIZADO DINÁMICO EN TIEMPO REAL
# ==============================================================================
print("\n" + "=" * 65)
print("▶️ Renderizado dinámico listo.")
print("=" * 65)

for k in range(pasos):
    if not plt.fignum_exists(fig.number):
        break

    # Actualizar Panel 1 (Nominal)
    l_id1.set_data(x_id[:k+1], y_id[:k+1])
    l_nom.set_data(x_nom[:k+1], y_nom[:k+1])
    p_id1.set_data([x_id[k]], [y_id[k]])
    p_nom.set_data([x_nom[k]], [y_nom[k]])
    err_nom = np.sqrt((x_id[k] - x_nom[k])**2 + (y_id[k] - y_nom[k])**2)
    ax1.set_title(f"Condiciones Nominales (Paso {k+1}/{pasos})\n$\\sigma_d=0.20$m, $\\sigma_\\delta=0.5^\\circ$ | Deriva: {err_nom:.2f} m", fontsize=10, fontweight='bold')

    # Actualizar Panel 2 (Ruido Alto)
    l_id2.set_data(x_id[:k+1], y_id[:k+1])
    l_al.set_data(x_al[:k+1], y_al[:k+1])
    p_id2.set_data([x_id[k]], [y_id[k]])
    p_al.set_data([x_al[k]], [y_al[k]])
    err_al = np.sqrt((x_id[k] - x_al[k])**2 + (y_id[k] - y_al[k])**2)
    ax2.set_title(f"Terreno Deslizante / Ruido Alto (Paso {k+1}/{pasos})\n$\\sigma_d=0.50$m, $\\sigma_\\delta=2.0^\\circ$ | Deriva: {err_al:.2f} m", fontsize=10, fontweight='bold')

    plt.pause(0.04)

print("✅ Renderizado completado exitosamente.")
plt.ioff()
plt.show()
