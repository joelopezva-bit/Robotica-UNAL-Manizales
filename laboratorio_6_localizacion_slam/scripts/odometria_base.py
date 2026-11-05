import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import Bicycle

# ==============================================================================
# 1. DEFINICIÓN DE MATRIZ DE COVARIANZA DE RUIDO (V)
# ==============================================================================
# Desviaciones estándar: 0.2 m en distancia, 0.5 grados en ángulo de dirección
sigma_dist = 0.2
sigma_steer = np.deg2rad(0.5)

# Matriz de covarianza V (dispersión del error en cada paso)
V = np.diag([sigma_dist, sigma_steer])**2


print("MATRIZ DE COVARIANZA DE RUIDO V:")
print(V)


# ==============================================================================
# 2. INSTANCIACIÓN DE MODELOS CINEMÁTICOS
# ==============================================================================
# Robot real con perturbación aleatoria gobernada por V
robot = Bicycle(covar=V)

# Robot ideal determinístico sin ruido
robot_ideal = Bicycle(covar=np.zeros((2, 2)))

# Estados iniciales [x, y, theta]
qp_real = [0.0, 0.0, 0.0]
qp_ideal = [0.0, 0.0, 0.0]

x_real, y_real = [], []
x_ideal, y_ideal = [], []

# Entrada de control constante: (velocidad v = 1.0 m/s, ángulo dirección delta = 0.2 rad)
u = (1.0, 0.2)

# ==============================================================================
# 3. BUCLE DE SIMULACIÓN Y PROPAGACIÓN CINEMÁTICA (200 PASOS)
# ==============================================================================
for _ in range(200):
    # Paso cinemático con ruido
    odo_real = robot.step(u)
    qp_real = robot.f(qp_real, odo_real)

    # Paso cinemático ideal
    odo_ideal = robot_ideal.step(u)
    qp_ideal = robot_ideal.f(qp_ideal, odo_ideal)

    # Registro de posiciones
    x_real.append(qp_real[0])
    y_real.append(qp_real[1])
    x_ideal.append(qp_ideal[0])
    y_ideal.append(qp_ideal[1])

# ==============================================================================
# 4. GENERACIÓN DE EVIDENCIA GRÁFICA
# ==============================================================================
plt.figure(figsize=(9, 7))
plt.plot(x_ideal, y_ideal, 'b--', linewidth=2, label='Trayectoria estimada sin ruido (Ideal)')
plt.plot(x_real, y_real, 'r-', linewidth=1.5, label='Trayectoria con odometría y ruido (Real)')

plt.scatter([0], [0], color='green', s=100, zorder=5, label='Inicio (0,0)')
plt.scatter([x_ideal[-1]], [y_ideal[-1]], color='blue', s=80, marker='o', zorder=5, label='Fin Ideal')
plt.scatter([x_real[-1]], [y_real[-1]], color='red', s=80, marker='x', zorder=5, label='Fin Real')

plt.xlabel('X [m]', fontsize=11)
plt.ylabel('Y [m]', fontsize=11)
plt.title('Comparación de Trayectorias', fontsize=12, fontweight='bold')
plt.grid(True, linestyle=':', alpha=0.7)
plt.axis('equal')
plt.legend(loc='best')

# Guardar figura en alta calidad
plt.savefig('odometria_comparacion.png', dpi=300, bbox_inches='tight')
print("✅ Gráfica generada y guardada como 'odometria_comparacion.png'")
plt.show()
