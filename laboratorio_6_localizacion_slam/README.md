# Laboratorio 6: Localización y Mapeo (Odometría, EKF y SLAM)

**Curso:** Robótica Móvil  
**Institución:** Universidad Nacional de Colombia - Sede Manizales  
**Estudiantes:** Joel Américo López Valdez & Cleyson Valerio Yauri Cupara  
**Origen:** Movilidad Académica UNSAAC (Perú)  
**Entorno de Ejecución:** Ubuntu 24.04 LTS | Python 3.12 | Robotics Toolbox para Python  

---

## 📌 Descripción General
Este repositorio contiene el desarrollo experimental del Capítulo 6 sobre localización probabilística y mapeo simultáneo en robótica móvil. Se analizan las limitaciones de la odometría en lazo abierto (*Dead Reckoning*), la estimación de pose mediante Filtro de Kalman Extendido (EKF), la corrección por balizas externas (*landmarks*) y el algoritmo completo de EKF-SLAM sin mapa previo.

---

## 📂 Estructura de Scripts y Códigos Desarrollados

Los archivos de código fuente se encuentran organizados en la carpeta `scripts/`:

| Archivo | Descripción del Experimento | Parámetros Clave |
| :--- | :--- | :--- |
| `odometria_base.py` | Simulación base de cinemática de bicicleta con y sin ruido de proceso. | $V = \text{diag}([0.2, 0.5^\circ])^2$ |
| `odometria_variaciones.py` | Evaluación de 3 escenarios (baja dispersión, condiciones nominales y derrape severo). | $\sigma_d \in [0.04, 0.50]\text{ m}$ |
| `odometria_animada.py` | Animación sincronizada en tiempo real de la divergencia de trayectoria. | Renderizado dual $1\times 2$ |
| `ekf_base.py` | Implementación del EKF en lazo abierto (solo etapa de predicción). | $P_0$, $V$, $T=20\text{ s}$ |
| `ekf_variaciones.py` | Análisis comparativo del crecimiento de incertidumbre $\|P\|_2$. | 3 niveles de covarianza $V$ |
| `ekf_animado.py` | Renderizado animado de la trayectoria estimada vs real y curva $\|P\|$. | Panel multipanel $1\times 3$ |
| `ekf_landmarks.py` | EKF en lazo cerrado con reducción periódica de incertidumbre por balizas. | $W = \text{diag}([0.1, 1^\circ])^2$, 20 balizas |
| `ekf_landmarks_variaciones.py`| Comparativa dinámica del alcance del sensor LiDAR (rango 2 m vs 6 m). | Rango visual corto vs largo |
| `slam_base.py` | EKF-SLAM exploratorio donde el robot desconoce el mapa (`map=None`). | Estimación conjunta $[x, y, \theta, m]$ |
| `slam_variaciones.py` | Evaluación cuantitativa del impacto del ruido de sensor en el mapeo. | $W$ bajo vs $W$ alto |
| `slam_animado.py` | Animación dinámica comparativa de SLAM y evolución de incertidumbre. | Panel multipanel $1\times 3$ |


## 🎥 Evidencias en Video (Demostraciones Dinámicas)

Las grabaciones en tiempo real de las trayectorias dinámicas y la convergencia de los algoritmos están disponibles en los siguientes enlaces:

* 🎬 **Parte 1 (Odometría Dinámica):** [Ver video en YouTube](https://youtu.be/4wWRN-S94Nk)
* 🎬 **Parte 2 (EKF Predicción y Crecimiento de Incertidumbre):** [Ver video en YouTube](https://youtu.be/xUk7XJ4gZhA)
* 🎬 **Parte 3 (EKF con Corrección por Landmarks):** [Ver video en YouTube](https://youtu.be/UyQhqJd7JJI)
* 🎬 **Parte 4 (EKF-SLAM Descubrimiento y Mapeo en Tiempo Real):** [Ver video en YouTube](https://youtu.be/ldF7eufHWA8)

---

## 📊 Instrucciones para Reproducir los Códigos

Para ejecutar cualquiera de las simulaciones desde la terminal de Ubuntu:

```bash
# 1. Clonar el repositorio
git clone [https://github.com/joelopezva-bit/robotica-unal-manizales.git](https://github.com/joelopezva-bit/robotica-unal-manizales.git)
cd robotica-unal-manizales/laboratorio_6_localizacion_slam/scripts

# 2. Ejecutar la simulación deseada (Ejemplo: SLAM Animado)
python3 slam_animado.py


---

## 🚀 Parte 5 (Segunda Mitad): SLAM con Cartographer en ROS 2 Jazzy

Esta sección complementa las simulaciones matemáticas iniciales desplegando un pipeline real de **Localización y Mapeo Simultáneo (SLAM)** utilizando **Cartographer**, integrado con el simulador **Gazebo Harmonic** y **RViz2** bajo el middleware **ROS 2 Jazzy Jalisco**.

### 📂 Estructura del Paquete ROS 2 y Mapas

Los archivos desarrollados para esta fase están separados de los scripts básicos y se ubican en la carpeta `paquete_ros2_slam/localization_lab/`:

| Directorio / Archivo | Función en el Proyecto |
| :--- | :--- |
| `config/cartographer.lua` | Sintonización de SLAM 2D y definición de los marcos de referencia (`base_scan`, `odom`, `map`). |
| `launch/cartographer.launch.py` | Orquestación paralela del nodo principal de Cartographer y la cuadrícula de ocupación (`occupancy_grid_node`). |
| `localization_lab/simple_controller.py` | Controlador reactivo autónomo. Se suscribe al LiDAR (`/scan`), detecta obstáculos a $<0.5\text{ m}$ y ejecuta giros evasivos. |
| `mapas_generados/` | Contiene el resultado final de la exploración (`my_map.pgm` y `my_map.yaml`). |

### 🎥 Evidencias en Video (Implementación en ROS 2)

Las demostraciones del entorno gráfico de Gazebo Harmonic, el controlador reactivo y la construcción del mapa están disponibles aquí:

- 🎬 **Ejecución del Controlador Reactivo y Entorno Gazebo:** [Enlace pendiente - Video 1]
- 🎬 **Prueba de Evasión Autónoma de Obstáculos:** [Enlace pendiente - Video 2]
- 🎬 **Mapeo SLAM en Tiempo Real (RViz2):** [Enlace pendiente - Video 3]

---

## 📊 Instrucciones para Reproducir el Entorno ROS 2

Para desplegar este sistema en Ubuntu 24.04, es necesario compilar el paquete en un *workspace* válido de ROS 2 y ejecutar los nodos en **5 terminales simultáneas** (asegurándose de ejecutar `source install/setup.bash` en cada una):

**1. Despliegue de Simulación Física:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch localization_lab cartographer.launch.py
rviz2
# (Configurar Fixed Frame a 'map' y agregar /map, /scan, TF y /submap_list)
ros2 run localization_lab simple_controller
ros2 run nav2_map_server map_saver_cli -f my_map

