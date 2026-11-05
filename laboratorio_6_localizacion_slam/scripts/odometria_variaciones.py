import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle

def simular_odometria(sigma_dist, sigma_steer_deg, pasos=200):
    # Matriz de covarianza V
    V = np.diag([sigma_dist, np.deg2rad(sigma_steer_deg)])**2
    
    # Modelos: con ruido e ideal
    robot = Bicycle(covar=V)
    robot_ideal = Bicycle(covar=np.zeros((2, 2)))
    
    qp_real = [0.0, 0.0, 0.0]
    qp_ideal = [0.0, 0.0, 0.0]
    
    x_real, y_real = [], []
    x_ideal, y_ideal = [], []
    
    u = (1.0, 0.2)
    
    for _ in range(pasos):
        odo_real = robot.step(u)
        qp_real = robot.f(qp_real, odo_real)
        
        odo_ideal = robot_ideal.step(u)
        qp_ideal = robot_ideal.f(qp_ideal, odo_ideal)
        
        x_real.append(qp_real[0])
        y_real.append(qp_real[1])
        x_ideal.append(qp_ideal[0])
        y_ideal.append(qp_ideal[1])
        
    error_pos_final = np.linalg.norm(np.array(qp_real[:2]) - np.array(qp_ideal[:2]))
    error_theta_final = np.rad2deg(abs(qp_real[2] - qp_ideal[2])) % 360
    
    return V, (x_ideal, y_ideal), (x_real, y_real), (qp_ideal, qp_real), (error_pos_final, error_theta_final)

# Definición de escenarios con nombres descriptivos
casos = [
    {
        "nombre": "Caso con Ruido Bajo",
        "sigma_d": 0.04,
        "sigma_th": 0.1,
        "color": "darkgreen"
    },
    {
        "nombre": "Caso con Condiciones Nominales de Operación (Sensores Estándar)",
        "sigma_d": 0.20,
        "sigma_th": 0.5,
        "color": "orange"
    },
    {
        "nombre": "Caso con Ruido Alto",
        "sigma_d": 0.50,
        "sigma_th": 2.0,
        "color": "crimson"
    }
]

fig, axs = plt.subplots(1, 3, figsize=(18, 6))
metricas_resumen = []

# ==============================================================================
# IMPRESIÓN DETALLADA DE MATRICES DE COVARIANZA EN TERMINAL
# ==============================================================================
print("\n" + "=" * 78)
print("ANÁLISIS ESTOCÁSTICO DE INCERTIDUMBRE: MATRICES DE COVARIANZA (V)")
print("=" * 78)

for i, caso in enumerate(casos):
    V, (x_id, y_id), (x_re, y_re), (qp_id, qp_re), (err_pos, err_th) = simular_odometria(
        caso["sigma_d"], caso["sigma_th"]
    )
    metricas_resumen.append((caso["nombre"], err_pos, err_th))
    
    print(f"\n[{caso['nombre']}]")
    print(f"Parámetros asignados: sigma_d = {caso['sigma_d']:.2f} m, sigma_delta = {caso['sigma_th']:.2f} deg")
    print("Matriz de Covarianza V (m^2 y rad^2):")
    print(np.array2string(V, formatter={'float_kind': lambda x: f"{x:12.6e}"}))
    print(f"-> Varianza lineal V[0,0] : {V[0, 0]:.6f} m^2")
    print(f"-> Varianza angular V[1,1]: {V[1, 1]:.8f} rad^2")
    print("-" * 78)
    
    # Configuración de gráficas
    ax = axs[i]
    ax.plot(x_id, y_id, 'b--', linewidth=2, label='Ideal')
    ax.plot(x_re, y_re, color=caso["color"], linewidth=1.8, label='Odometría')
    ax.scatter([0], [0], color='black', s=80, zorder=5, label='Inicio')
    ax.scatter([qp_id[0]], [qp_id[1]], color='blue', s=60, zorder=5, label='Fin Ideal')
    ax.scatter([qp_re[0]], [qp_re[1]], color=caso["color"], s=70, marker='x', zorder=5, label='Fin Real')
    
    ax.set_title(f"{caso['nombre']}\n$\\sigma_d={caso['sigma_d']}$ m, $\\sigma_\\delta={caso['sigma_th']}^\\circ$\nError final: {err_pos:.2f} m", fontsize=9, fontweight='bold')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.set_aspect('equal')
    ax.legend(loc='best', fontsize=8)

# Tabla de resumen cuantitativo
print("\n" + "=" * 78)
print(f"{'Escenario Evaluado':<45} | {'Error Pos. [m]':<15} | {'Error Ang. [°]':<12}")
print("=" * 78)
for nombre, err_pos, err_th in metricas_resumen:
    print(f"{nombre:<45} | {err_pos:<15.3f} | {err_th:<12.2f}")
print("=" * 78 + "\n")

plt.tight_layout()
plt.savefig("odometria_analisis_comparativo.png", dpi=300, bbox_inches='tight')
plt.show()
