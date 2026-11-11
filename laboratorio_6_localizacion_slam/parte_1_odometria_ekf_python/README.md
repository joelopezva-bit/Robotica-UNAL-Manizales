# Parte 1: Odometría, Filtro de Kalman Extendido (EKF) y EKF-SLAM en Python

**Curso:** Robótica Móvil  
**Institución:** Universidad Nacional de Colombia - Sede Manizales  
**Estudiantes:** Joel Américo López Valdez & Cleyson Valerio Yauri Cupara  
**Origen:** Movilidad Académica UNSAAC (Perú)  
**Herramientas:** Python 3.12 | Robotics Toolbox for Python (Peter Corke) | Matplotlib  

---

## 📌 Descripción Técnica
Esta sección aborda el modelado cinemático probabilístico y la estimación de estado estocástica. Se evalúa la divergencia temporal de la odometría pura en lazo abierto (*Dead Reckoning*), la propagación de incertidumbre mediante la covarianza del EKF (etapa de predicción), la corrección y reducción de incertidumbre por balizas (*landmarks*), y finalmente el algoritmo completo de **EKF-SLAM** sin mapa previo.

---

## 📂 Códigos Desarrollados (`scripts/`)

| Archivo | Experimento / Algoritmo | Parámetros Clave |
| :--- | :--- | :--- |
| `odometria_base.py` | Modelo de bicicleta cinemática con perturbación gaussiana. | $V = \text{diag}([0.2, 0.5^\circ])^2$ |
| `odometria_variaciones.py` | Evaluación de 3 escenarios (baja dispersión, nominal y derrape severo). | $\sigma_d \in [0.04, 0.20]$ |
| `odometria_animada.py` | Animación temporal de la divergencia acumulativa de trayectoria. | Renderizado dual $1\times 2$ |
| `ekf_base.py` | Predicción EKF en lazo abierto (sin observaciones sensoriales). | $P_0$, $V$, $T=20\text{ s}$ |
| `ekf_variaciones.py` | Evolución analítica de la norma de incertidumbre $\Vert{}P\Vert{}_2$. | 3 matrices de covarianza $V$ |
| `ekf_animado.py` | Renderizado dinámico de trayectoria estimada vs. real y evolución de $\Vert{}P\Vert{}$. | Panel multipanel $1\times 3$ |
| `ekf_landmarks.py` | EKF en lazo cerrado con actualización recurrente por balizas conocidas. | $W = \text{diag}([0.1, 1^\circ])^2$, 20 balizas |
| `ekf_landmarks_variaciones.py` | Comparativa del radio de cobertura del sensor ($2\text{ m}$ vs. $6\text{ m}$). | Alcance de sensado corto vs. largo |
| `slam_base.py` | EKF-SLAM en línea donde el mapa es desconocido a priori (`map=None`). | Vector aumentado $[x, y, \theta, m]^T$ |
| `slam_variaciones.py` | Impacto cuantitativo del ruido del sensor en la precisión del mapa. | $W$ nominal vs. $W$ degradado |
| `slam_animado.py` | Animación sincronizada de descubrimiento de balizas y convergencia. | Panel multipanel $1\times 3$ |

---

## 🎥 Evidencias en Video

* 🎬 **Parte 1 (Odometría Dinámica):** [Ver demostración en YouTube](https://youtu.be/4wWRN-S94Nk)
* 🎬 **Parte 2 (EKF Predicción y Crecimiento de Incertidumbre):** [Ver demostración en YouTube](https://youtu.be/xUk7XJ4gZhA)
* 🎬 **Parte 3 (EKF con Corrección por Landmarks):** [Ver demostración en YouTube](https://youtu.be/UyQhqJd7JJI)
* 🎬 **Parte 4 (EKF-SLAM Descubrimiento y Mapeo en Tiempo Real):** [Ver demostración en YouTube](https://youtu.be/ldF7eufHWA8)

---

## 💻 Instrucciones de Ejecución

```bash
cd ~/robotica-unal-manizales/laboratorio_6_localizacion_slam/parte_1_odometria_ekf_python/scripts
python3 slam_animado.py
