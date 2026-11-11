import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle, EKF, RandomPath, LandmarkMap, RangeBearingSensor
from math import pi

print("=" * 70)
print("CAPÍTULO 6 - PARTE 4: SLAM USANDO EKF (MAPA DESCONOCIDO)")
print("=" * 70)

# ==============================================================================
# 1. PARÁMETROS Y MATRICES DE COVARIANZA
# ==============================================================================
# Covarianza de odometría V
V = np.diag([0.02, np.deg2rad(0.5)])**2

# Mapa real (Oculto para el EKF, solo para el sensor físico)
np.random.seed(42)
mapa = LandmarkMap(20, workspace=10)

# Covarianza sensor de rango W
W = np.diag([0.1, np.deg2rad(1)])**2

# Covarianza inicial P0
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2

# ==============================================================================
# 2. INSTANCIACIÓN DE MODELOS
# ==============================================================================
robot = Bicycle(covar=V, animation=None)
robot.control = RandomPath(workspace=10, seed=10)

sensor = RangeBearingSensor(
    robot=robot,
    map=mapa,
    covar=W,
    angle=[-pi/2, pi/2],
    range=4
)

# EKF SLAM: map=None indica que el EKF desconoce las balizas al inicio
slam = EKF(
    robot=(robot, V),
    P0=P0,
    sensor=(sensor, W),
    map=None
)

print("\nEjecutando SLAM (T = 40 segundos)...")
slam.run(T=40)
print("Simulación SLAM finalizada.")

# ==============================================================================
# 3. GENERACIÓN DE GRÁFICAS
# ==============================================================================
# Figura 1: Mapeo y Trayectoria
fig_tray = plt.figure(figsize=(10, 8))
plt.sca(plt.gca())
mapa.plot()
robot.plot_xy(color='blue', label='Trayectoria Real')
slam.plot_xy(color='red', label='Trayectoria Estimada (SLAM)')
slam.plot_ellipse(color='red', alpha=0.3)

plt.title('SLAM usando EKF', fontsize=12, fontweight='bold')
plt.xlabel('Posición X [m]')
plt.ylabel('Posición Y [m]')
plt.grid(True, linestyle=':', alpha=0.7)
plt.axis('equal')
plt.legend(loc='best', fontsize=9, frameon=True)
plt.savefig('slam_base_trayectoria.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica de trayectoria SLAM guardada.")

# Figura 2: Incertidumbre
plt.figure(figsize=(9, 6))
plt.plot(slam.get_t(), slam.get_Pnorm(), color='purple', linewidth=2.0, label=r'Incertidumbre acumulada $\|P\|_2$')
plt.title('Evolución de Incertidumbre en SLAM', fontsize=12, fontweight='bold')
plt.xlabel('Tiempo [s]')
plt.ylabel(r'Norma de Covarianza $\|P\|$', fontsize=11)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='upper right', frameon=True)
plt.savefig('slam_base_incertidumbre.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica de incertidumbre SLAM guardada.")

plt.show()
