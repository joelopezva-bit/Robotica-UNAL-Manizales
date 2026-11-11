import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor
from math import pi

print("=" * 70)
print("CAPÍTULO 6 - PARTE 3: EKF CON CORRECCIÓN POR LANDMARKS")
print("=" * 70)

# ==============================================================================
# 1. PARÁMETROS Y MATRICES DE COVARIANZA
# ==============================================================================
# Covarianza de odometría (Ruido de Proceso)
V = np.diag([0.02, np.deg2rad(0.5)])**2

# Covarianza inicial de la estimación
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2

# Covarianza del sensor de rango y marcación (Ruido de Medición)
# sigma_r = 0.1 m (distancia), sigma_theta = 1 grado (ángulo)
W = np.diag([0.1, np.deg2rad(1)])**2

print("\nMatriz de Covarianza de Odometría (V):")
print(np.array2string(V, formatter={'float_kind': lambda x: f"{x:12.6e}"}))

print("\nMatriz de Covarianza del Sensor LiDAR (W):")
print(np.array2string(W, formatter={'float_kind': lambda x: f"{x:12.6e}"}))
print("-" * 70)

# ==============================================================================
# 2. INSTANCIACIÓN DE MODELOS, MAPA Y SENSOR
# ==============================================================================
# Robot con movimiento aleatorio
robot = Bicycle(covar=V, animation=None)
robot.control = RandomPath(workspace=10)

# Crear un mapa con 20 landmarks esparcidos en el área de trabajo
mapa = LandmarkMap(20, workspace=10)

# Simular un LiDAR montado en el robot con alcance de 4 metros y apertura de 180°
sensor = RangeBearingSensor(
    robot=robot,
    map=mapa,
    covar=W,
    angle=[-pi/2, pi/2],
    range=4
)

# Filtro de Kalman Extendido (Lazo cerrado: Conoce el mapa de landmarks previamente)
ekf = EKF(
    robot=(robot, V),
    P0=P0,
    map=mapa,
    sensor=(sensor, W)
)

print("\nEjecutando simulación EKF con Landmarks (T = 40 segundos)...")
ekf.run(T=40)
print("Simulación finalizada exitosamente.")

# ==============================================================================
# 3. GENERACIÓN DE EVIDENCIAS GRÁFICAS
# ==============================================================================
# Figura 1: Trayectoria Real vs Estimada con Elipses de Confianza
fig_tray = plt.figure(figsize=(9, 7))
mapa.plot()
robot.plot_xy(color='blue', label='Trayectoria Real (Robot)')
ekf.plot_xy(color='red', label='Trayectoria Estimada (EKF)')
ekf.plot_ellipse(color='red', alpha=0.3)

plt.title('Parte 3 - EKF con Corrección por Landmarks (Mapa Conocido)', fontsize=12, fontweight='bold')
plt.xlabel('Posición X [m]', fontsize=11)
plt.ylabel('Posición Y [m]', fontsize=11)
plt.grid(True, linestyle=':', alpha=0.7)
plt.axis('equal')
plt.legend(loc='best', fontsize=9, frameon=True, shadow=True)
plt.savefig('ekf_landmarks_trayectoria.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica de trayectoria guardada como 'ekf_landmarks_trayectoria.png'")

# Figura 2: Evolución de Incertidumbre
plt.figure(figsize=(9, 6))
plt.plot(ekf.get_t(), ekf.get_Pnorm(), color='crimson', linewidth=2.0, label=r'Incertidumbre acumulada $\|P\|_2$')
plt.title('Parte 3 - Evolución de Incertidumbre con Correcciones', fontsize=12, fontweight='bold')
plt.xlabel('Tiempo [s]', fontsize=11)
plt.ylabel(r'Norma de Covarianza $\|P\|$', fontsize=11)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='upper right', fontsize=10, frameon=True)
plt.savefig('ekf_landmarks_incertidumbre.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica de incertidumbre guardada como 'ekf_landmarks_incertidumbre.png'")

print("=" * 70)
plt.show()
