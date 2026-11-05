import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath

casos = [
    {
        "nombre": "Escenario A: Bajo Ruido",
        "sigma_d": 0.005,
        "sigma_th": 0.15,
        "color": "darkgreen"
    },
    {
        "nombre": "Escenario B: Condiciones Nominales (Guía)",
        "sigma_d": 0.020,
        "sigma_th": 0.50,
        "color": "darkorange"
    },
    {
        "nombre": "Escenario C: Alto Ruido",
        "sigma_d": 0.080,
        "sigma_th": 1.50,
        "color": "crimson"
    }
]

P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2
T_sim = 20

print("\n" + "=" * 78)
print("EVALUACIÓN COMPARATIVA DE ESCENARIOS EKF (VARIACIÓN DE COVARIANZA V)")
print("=" * 78)

fig = plt.figure(figsize=(18, 10))
gs = fig.add_gridspec(2, 3)

ax_incertidumbre = fig.add_subplot(gs[1, :])

resumen = []

for idx, caso in enumerate(casos):
    V = np.diag([caso["sigma_d"], np.deg2rad(caso["sigma_th"])])**2
    
    robot = Bicycle(covar=V, animation=None)
    robot.control = RandomPath(workspace=10)
    
    ekf = EKF(robot=(robot, V), P0=P0)
    ekf.run(T=T_sim)
    
    t_arr = ekf.get_t()
    p_norm = ekf.get_Pnorm()
    incertidumbre_final = p_norm[-1]
    
    # =====================================================================
    # MÉTODO A PRUEBA DE FALLOS: Extraer el punto final directamente 
    # de la gráfica generada por el toolbox para evitar errores de índices.
    # =====================================================================
    fig_temp = plt.figure()
    robot.plot_xy()
    ekf.plot_xy()
    lineas = plt.gca().get_lines()
    real_final = np.array([lineas[0].get_xdata()[-1], lineas[0].get_ydata()[-1]])
    est_final = np.array([lineas[1].get_xdata()[-1], lineas[1].get_ydata()[-1]])
    plt.close(fig_temp)
    
    error_final = np.linalg.norm(real_final - est_final)
    
    resumen.append((caso["nombre"], caso["sigma_d"], caso["sigma_th"], error_final, incertidumbre_final))
    
    ax_tray = fig.add_subplot(gs[0, idx])
    robot.plot_xy(color='blue', label='Trayectoria Real')
    ekf.plot_xy(color=caso["color"], label='Estimación EKF')
    ax_tray.set_title(f"{caso['nombre']}\n$\\sigma_d={caso['sigma_d']}$m, $\\sigma_\\delta={caso['sigma_th']}^\\circ$\nError final: {error_final:.2f} m", fontsize=9, fontweight='bold')
    ax_tray.set_xlabel('X [m]', fontsize=9)
    ax_tray.set_ylabel('Y [m]', fontsize=9)
    ax_tray.grid(True, linestyle=':', alpha=0.6)
    ax_tray.axis('equal')
    ax_tray.legend(loc='best', fontsize=8)
    
    ax_incertidumbre.plot(t_arr, p_norm, color=caso["color"], linewidth=2, label=f"{caso['nombre']} (Final: {incertidumbre_final:.2f})")

# Solución al SyntaxWarning: Se agregaron las letras 'r' antes de los textos de los títulos
ax_incertidumbre.set_title(r'Evolución Comparativa de la Incertidumbre $\|P\|_2$ en el Tiempo', fontsize=11, fontweight='bold')
ax_incertidumbre.set_xlabel('Tiempo [s]', fontsize=10)
ax_incertidumbre.set_ylabel(r'Norma de Covarianza $\|P\|$', fontsize=10)
ax_incertidumbre.grid(True, linestyle=':', alpha=0.7)
ax_incertidumbre.legend(loc='upper left', fontsize=9, frameon=True)

plt.tight_layout()
plt.savefig('ekf_analisis_comparativo.png', dpi=300, bbox_inches='tight')
print("✅ Figura comparativa guardada como 'ekf_analisis_comparativo.png'")

print("\n" + "=" * 85)
print(f"{'Escenario':<42} | {'sigma_d [m]':<12} | {'Error Pos. [m]':<15} | {'Incertidumbre Final':<15}")
print("=" * 85)
for nom, sd, st, err, inc in resumen:
    print(f"{nom:<42} | {sd:<12.3f} | {err:<15.3f} | {inc:<15.3f}")
print("=" * 85 + "\n")

plt.show()
