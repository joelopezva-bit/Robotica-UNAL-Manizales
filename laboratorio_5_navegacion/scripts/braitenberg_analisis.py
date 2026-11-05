# ==============================================================================
# SCRIPT COMPLEMENTARIO: ANÁLISIS DE PARÁMETROS Y PREGUNTAS (BRAITENBERG)
# ==============================================================================

import numpy as np
import matplotlib.pyplot as plt

# 1. Definición del Campo Escalar
def sensorfield(x, y, xc=60, yc=90):
    return 1000 / ((x - xc)**2 + (y - yc)**2 + 50)

# 2. Función de Simulación Modular
def simulate(Kv, Ks, n_iter=222, x0=10, y0=10, theta0=0, d=2, xc=60, yc=90):
    x, y, theta = x0, y0, theta0
    tx, ty = [x], [y]
    sl_list, sr_list = [], []
    v_list, w_list = [], []

    for _ in range(n_iter):
        sl = sensorfield(x - d*np.sin(theta), y + d*np.cos(theta), xc, yc)
        sr = sensorfield(x + d*np.sin(theta), y - d*np.cos(theta), xc, yc)
        
        v = -Kv * (sl + sr)
        w = Ks * (sr - sl)
        
        sl_list.append(sl)
        sr_list.append(sr)
        v_list.append(v)
        w_list.append(w)
        
        x += v * np.cos(theta) * 0.1
        y += v * np.sin(theta) * 0.1
        theta += w * 0.1
        
        tx.append(x)
        ty.append(y)
        
    return np.array(tx), np.array(ty), np.array(sl_list), np.array(sr_list), np.array(v_list), np.array(w_list)

print("=" * 70)
print("EJECUTANDO ANÁLISIS EXPERIMENTAL Y GENERACIÓN DE GRÁFICAS")
print("=" * 70)

# ------------------------------------------------------------------------------
# FIGURA PREGUNTA 1: Diferencia entre sensores (s_left vs s_right)
# ------------------------------------------------------------------------------
tx_base, ty_base, sl, sr, v_hist, w_hist = simulate(Kv=14, Ks=17, n_iter=222)

plt.figure(figsize=(8, 4))
plt.plot(sl, 'g-', linewidth=2, label="Sensor Izquierdo ($s_{left}$)")
plt.plot(sr, 'r--', linewidth=2, label="Sensor Derecho ($s_{right}$)")
plt.title("Pregunta 1: Diferencia de Lectura entre Sensores", fontweight='bold')
plt.xlabel("Iteración (Paso de Tiempo)")
plt.ylabel("Intensidad del Campo Detectada")
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.tight_layout()
plt.savefig("pregunta1_sensores.png", dpi=300)
plt.show()

# ------------------------------------------------------------------------------
# FIGURA PREGUNTA 2: Evolución de Comandos de Control (Frenado Repulsivo)
# ------------------------------------------------------------------------------
plt.figure(figsize=(8, 4))
plt.plot(v_hist, 'b-', linewidth=2, label="Velocidad Lineal ($v$)")
plt.plot(w_hist, 'm--', linewidth=2, label=r"Velocidad Angular ($\omega$)")
plt.title("Pregunta 2: Comportamiento de Velocidades al Aproximarse a la Meta", fontweight='bold')
plt.xlabel("Iteración (Paso de Tiempo)")
plt.ylabel("Magnitud del Comando de Control")
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.tight_layout()
plt.savefig("pregunta2_velocidades.png", dpi=300)
plt.show()

# ------------------------------------------------------------------------------
# FIGURA PREGUNTA 3: Efecto de Aumentar Iteraciones (N=600 - Bucle/Órbita)
# ------------------------------------------------------------------------------
tx_600, ty_600, _, _, _, _ = simulate(Kv=14, Ks=17, n_iter=600)

plt.figure(figsize=(7, 6))
plt.plot(tx_600, ty_600, 'g-', linewidth=2, label="Trayectoria Extendida (N=600)")
plt.scatter([60], [90], c='red', s=150, marker='*', zorder=5, label="Objetivo (60, 90)")
plt.scatter([10], [10], c='blue', s=100, zorder=5, label="Inicio (10, 10)")
plt.title("Pregunta 3: Comportamiento para Iteraciones Extendidas (N=600)", fontweight='bold')
plt.xlabel("Posición X [m]")
plt.ylabel("Posición Y [m]")
plt.grid(True, linestyle='--', alpha=0.7)
plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.savefig("pregunta3_iteraciones_600.png", dpi=300)
plt.show()

# ------------------------------------------------------------------------------
# FIGURA EXPERIMENTAL: Variación de Parámetros (Kv y Ks)
# ------------------------------------------------------------------------------
tx_ks_alta, ty_ks_alta, _, _, _, _ = simulate(Kv=14, Ks=40, n_iter=222)
tx_kv_baja, ty_kv_baja, _, _, _, _ = simulate(Kv=4, Ks=17, n_iter=222)

plt.figure(figsize=(8, 6))
plt.plot(tx_base, ty_base, 'b-', linewidth=2, label="Base ($K_v=14, K_s=17$)")
plt.plot(tx_ks_alta, ty_ks_alta, 'm-.', linewidth=2, label="Alta Ganancia Angular ($K_s=40$)")
plt.plot(tx_kv_baja, ty_kv_baja, 'c:', linewidth=2.5, label="Baja Ganancia Lineal ($K_v=4$)")
plt.scatter([60], [90], c='red', s=150, marker='*', zorder=5, label="Objetivo (60, 90)")
plt.title("Análisis Experimental: Variación de Ganancias ($K_v$ y $K_s$)", fontweight='bold')
plt.xlabel("Posición X [m]")
plt.ylabel("Posición Y [m]")
plt.grid(True, linestyle='--', alpha=0.7)
plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.savefig("variacion_parametros_gains.png", dpi=300)
plt.show()

# ------------------------------------------------------------------------------
# RESUMEN CUANTITATIVO DE RESULTADOS EN TERMINAL
# ------------------------------------------------------------------------------
print("\n" + "=" * 70)
print("RESUMEN DE POSICIONES FINALES ALCANZADAS")
print("=" * 70)
print(f"1. Caso Base (Kv=14, Ks=17, N=222)   : X = {tx_base[-1]:.2f} m, Y = {ty_base[-1]:.2f} m")
print(f"2. N Extendidas (Kv=14, Ks=17, N=600): X = {tx_600[-1]:.2f} m, Y = {ty_600[-1]:.2f} m (Orbita la meta)")
print(f"3. Alta Ks (Kv=14, Ks=40, N=222)     : X = {tx_ks_alta[-1]:.2f} m, Y = {ty_ks_alta[-1]:.2f} m")
print(f"4. Baja Kv (Kv=4, Ks=17, N=222)      : X = {tx_kv_baja[-1]:.2f} m, Y = {ty_kv_baja[-1]:.2f} m (Incompleto)")
print("=" * 70 + "\n")
