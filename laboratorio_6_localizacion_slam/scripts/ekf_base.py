import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath

print("=" * 70)
print("CAPÍTULO 6 - PARTE 2: ESTIMACIÓN PROBABILÍSTICA CON EKF (SOLO PREDICCIÓN)")
print("=" * 70)

# ==============================================================================
# 1. PARÁMETROS Y MATRICES DE COVARIANZA
# ==============================================================================
# Covarianza de odometría V: sigma_d = 0.02 m, sigma_delta = 0.5 grados
sigma_d = 0.02
sigma_steer = np.deg2rad(0.5)
V = np.diag([sigma_d, sigma_steer])**2

# Covarianza inicial del estado P0: incertidumbre en [x, y, theta]
sigma_x0 = 0.05
sigma_y0 = 0.05
sigma_th0 = np.deg2rad(0.5)
P0 = np.diag([sigma_x0, sigma_y0, sigma_th0])**2

print("\nMatriz de Covarianza del Ruido de Odometría (V):")
print(np.array2string(V, formatter={'float_kind': lambda x: f"{x:12.6e}"}))
print(f"-> Varianza lineal V[0,0] : {V[0, 0]:.6f} m^2")
print(f"-> Varianza angular V[1,1]: {V[1, 1]:.8f} rad^2")

print("\nMatriz de Covarianza Inicial de Estimación (P0):")
print(np.array2string(P0, formatter={'float_kind': lambda x: f"{x:12.6e}"}))
print(f"-> Varianza pos X P0[0,0] : {P0[0, 0]:.6f} m^2")
print(f"-> Varianza pos Y P0[1,1] : {P0[1, 1]:.6f} m^2")
print(f"-> Varianza theta P0[2,2] : {P0[2, 2]:.8f} rad^2")
print("-" * 70)

# ==============================================================================
# 2. INSTANCIACIÓN Y EJECUCIÓN DEL EKF
# ==============================================================================
# Robot físico con ruido de proceso
robot = Bicycle(covar=V, animation=None)
robot.control = RandomPath(workspace=10)

# Filtro de Kalman Extendido (sin sensor exteroceptivo)
ekf = EKF(robot=(robot, V), P0=P0)

print("\nEjecutando simulación EKF durante T = 20 segundos...")
ekf.run(T=20)
print("Simulación finalizada exitosamente.")

# ==============================================================================
# 3. GENERACIÓN DE EVIDENCIAS GRÁFICAS CON LEYENDAS
# ==============================================================================
# Figura 1: Trayectoria Real vs Estimada
plt.figure(figsize=(9, 7))
robot.plot_xy(color='blue', label='Trayectoria Real (Robot)')
ekf.plot_xy(color='red', label='Trayectoria Estimada (EKF Predicción)')
plt.title(' Trayectoria Real vs Estimada', fontsize=12, fontweight='bold')
plt.xlabel('Posición X [m]', fontsize=11)
plt.ylabel('Posición Y [m]', fontsize=11)
plt.grid(True, linestyle=':', alpha=0.7)
plt.axis('equal')
plt.legend(loc='best', fontsize=10, frameon=True, shadow=True)
plt.savefig('ekf_base_trayectoria.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica de trayectoria guardada como 'ekf_base_trayectoria.png'")

# Figura 2: Crecimiento de Incertidumbre
plt.figure(figsize=(9, 6))
plt.plot(ekf.get_t(), ekf.get_Pnorm(), color='crimson', linewidth=2.2, label=r'Incertidumbre acumulada $\|P\|_2$')
plt.title('Crecimiento Continuo de la Incertidumbre', fontsize=12, fontweight='bold')
plt.xlabel('Tiempo [s]', fontsize=11)
plt.ylabel(r'Norma de la Matriz de Covarianza $\|P\|$', fontsize=11)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='upper left', fontsize=10, frameon=True, shadow=True)
plt.savefig('ekf_base_incertidumbre.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica de incertidumbre guardada como 'ekf_base_incertidumbre.png'")

print("=" * 70)
plt.show()
