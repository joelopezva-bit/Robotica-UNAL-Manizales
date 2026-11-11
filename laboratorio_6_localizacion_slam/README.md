# Laboratorio 6: Localización y Mapeo (Odometría, EKF y SLAM)

**Curso:** Robótica Móvil  
**Institución:** Universidad Nacional de Colombia - Sede Manizales  
**Estudiantes:** Joel Américo López Valdez & Cleyson Valerio Yauri Cupara  
**Entorno de Ejecución:** Ubuntu 24.04 LTS | Python 3.12 | ROS 2 Jazzy Jalisco | Gazebo Harmonic | RViz2 | Robotics Toolbox para Python  

---

## 📌 Descripción General

Este repositorio reúne el desarrollo teórico, algorítmico y experimental correspondiente al estudio de **Localización y Mapeo Simultáneo (SLAM)** en robótica móvil, dividiéndose en dos etapas complementarias:

1. **Parte 1 (Simulación Algorítmica en Python):** Evaluación probabilística de la divergencia de odometría en lazo abierto (*Dead Reckoning*), filtrado estocástico mediante Filtro de Kalman Extendido (EKF solo predicción y con corrección de balizas/*landmarks*), y algoritmo completo de **EKF-SLAM** sobre entornos desconocidos.
2. **Parte 2 (Pipeline Experimental en ROS 2 Jazzy):** Implementación de SLAM 2D mediante **Cartographer**, integrado con el simulador físico **Gazebo Harmonic**, visualización y depuración en **RViz2**, y diseño de un **controlador reactivo autónomo** en Python con evasión de colisiones vía LiDAR (`/scan`).

---

## 📂 Organización del Repositorio

La jerarquía del directorio se encuentra estructurada para separar las implementaciones matemáticas de los paquetes del middleware:

```text
laboratorio_6_localizacion_slam/
├── scripts/                      # Códigos de simulación y animación (Robotics Toolbox)
├── paquete_ros2_slam/            # Paquete ROS 2 Jazzy completo
│   └── localization_lab/
│       ├── config/               # Configuración paramétrica de Cartographer (.lua)
│       ├── launch/               # Lanzadores de nodos y cuadrícula (.launch.py)
│       ├── localization_lab/     # Lógica del controlador reactivo en Python
│       ├── package.xml           # Definición de dependencias
│       └── setup.py              # Enrutamiento de puntos de entrada y ejecutables
├── mapas_generados/              # Resultados del mapeo en escala de grises (.pgm y .yaml)
└── README.md                     # Documentación técnica completa
```

---

## 🔬 PARTE 1: Simulaciones con Robotics Toolbox (Python)

### 📂 Estructura de Scripts

Los códigos fuente de esta primera fase están ubicados en la carpeta `scripts/`:

| Archivo | Descripción del Experimento | Parámetros Clave |
| :--- | :--- | :--- |
| `odometria_base.py` | Simulación base de cinemática diferencial/bicicleta con y sin perturbación estocástica. | $V = \text{diag}([0.2, 0.5^\circ])^2$ |
| `odometria_variaciones.py` | Evaluación de 3 escenarios (baja dispersión, condiciones nominales y derrape severo). | $\sigma_d \in [0.04, 0.20]$ |
| `odometria_animada.py` | Animación temporal sincronizada de la divergencia acumulativa de trayectoria. | Renderizado dual $1\times 2$ |
| `ekf_base.py` | Implementación del EKF en lazo abierto (etapa exclusiva de predicción cinemática). | $P_0$, $V$, $T=20\text{ s}$ |
| `ekf_variaciones.py` | Análisis comparativo del crecimiento de la norma de incertidumbre $\|P\|_2$. | 3 matrices de covarianza $V$ |
| `ekf_animado.py` | Renderizado dinámico de la trayectoria estimada vs. real y evolución de $\|P\|$. | Panel multipanel $1\times 3$ |
| `ekf_landmarks.py` | EKF en lazo cerrado con reducción periódica de covarianza mediante balizas. | $W = \text{diag}([0.1, 1^\circ])^2$, 20 balizas |
| `ekf_landmarks_variaciones.py` | Comparativa del radio de cobertura del sensor visual/LiDAR ($2\text{ m}$ vs. $6\text{ m}$). | Rango de sensado corto vs. largo |
| `slam_base.py` | EKF-SLAM en línea donde el robot desconoce el mapa de balizas a priori (`map=None`). | Vector aumentado $[x, y, \theta, m]^T$ |
| `slam_variaciones.py` | Evaluación cuantitativa del impacto del ruido sensorial en la precisión del mapa. | $W$ nominal vs. $W$ degradado |
| `slam_animado.py` | Animación comparativa de descubrimiento espacial y convergencia de incertidumbre. | Panel multipanel $1\times 3$ |

### 🎥 Evidencias en Video (Parte 1)

Las grabaciones dinámicas de convergencia y propagación de incertidumbre se encuentran en:

* 🎬 **Parte 1 (Odometría Dinámica):** [Ver demostración en YouTube](https://youtu.be/4wWRN-S94Nk)
* 🎬 **Parte 2 (EKF Predicción y Crecimiento de Incertidumbre):** [Ver demostración en YouTube](https://youtu.be/xUk7XJ4gZhA)
* 🎬 **Parte 3 (EKF con Corrección por Landmarks):** [Ver demostración en YouTube](https://youtu.be/UyQhqJd7JJI)
* 🎬 **Parte 4 (EKF-SLAM Descubrimiento y Mapeo en Tiempo Real):** [Ver demostración en YouTube](https://youtu.be/ldF7eufHWA8)

### 💻 Instrucciones de Ejecución (Parte 1)

```bash
# 1. Clonar el repositorio y posicionarse en la carpeta de scripts
git clone [https://github.com/joelopezva-bit/robotica-unal-manizales.git](https://github.com/joelopezva-bit/robotica-unal-manizales.git)
cd robotica-unal-manizales/laboratorio_6_localizacion_slam/scripts

# 2. Ejecutar la simulación deseada
python3 slam_animado.py
```

---

## 🚀 PARTE 2: SLAM con Cartographer en ROS 2 Jazzy

Esta sección valida las técnicas de localización probabilística sobre una arquitectura de robótica distribuida, integrando el sensor LiDAR 2D del TurtleBot3 Burger con el solucionador de grafos de optimización de Cartographer.

### 📂 Estructura del Paquete `localization_lab` y Mapas

| Directorio / Archivo | Función en el Sistema | Parámetros / Detalles Técnicos |
| :--- | :--- | :--- |
| `config/cartographer.lua` | Configuración del *trajectory builder* y optimización de poses del grafo. | Frames: `base_scan`, `odom`, `map` \| `use_odometry = false` |
| `launch/cartographer.launch.py` | Orquestación coordinada de los nodos de procesamiento espacial. | Lanza `cartographer_node` y `occupancy_grid_node` (resolución $0.05\text{ m}$) |
| `localization_lab/simple_controller.py` | Nodo de control reactivo autónomo programado en Python. | Suscripción a `/scan`, cono de detección de 60°, frenado/rotación si $d < 0.50\text{ m}$ |
| `mapas_generados/my_map.pgm` | Matriz bidimensional de probabilidad de ocupación rasterizada. | Espacio libre (blanco), ocupado (negro) y desconocido (gris) |
| `mapas_generados/my_map.yaml` | Metadatos geométricos y georreferenciación del mapa generado. | Resolución $0.05\text{ m/pixel}$, límites y origen cartográfico |

### 🎥 Evidencias en Video (Parte 2)

Las pruebas experimentales de exploración, reactividad y mapeo se documentan en:

* 🎬 **Parte 5.1 (Ejecución del Controlador Reactivo y Entorno Gazebo):** [Ver demostración en YouTube (Enlace pendiente)]()
* 🎬 **Parte 5.2 (Prueba de Evasión Autónoma de Obstáculos):** [Ver demostración en YouTube (Enlace pendiente)]()
* 🎬 **Parte 5.3 (Mapeo SLAM en Tiempo Real en RViz2):** [Ver demostración en YouTube (Enlace pendiente)]()

### 💻 Instrucciones de Ejecución y Pipeline ROS 2 (Parte 2)

Para compilar y ejecutar el ecosistema en Ubuntu 24.04 LTS, copie el paquete `localization_lab` al espacio de trabajo local `~/ros2_ws/src/`, compile con `colcon build --symlink-install` y ejecute cada bloque en terminales independientes:

#### Terminal 1: Simulación Física (Gazebo Harmonic)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### Terminal 2: Verificación de Tópicos de Sensores
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
# Validar la presencia de /scan, /odom, /tf y /cmd_vel
```

#### Terminal 3: Nodo SLAM (Cartographer)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch localization_lab cartographer.launch.py
```

#### Terminal 4: Visualizador Gráfico (RViz2)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```
* **Configuración de visualización:**
  * Establecer **Global Options $\rightarrow$ Fixed Frame** en `map`.
  * Añadir mediante **Add $\rightarrow$ By topic**: `/map` (Map) y `/scan` (LaserScan).
  * Añadir mediante **Add $\rightarrow$ By display type**: `TF` y `Submaps` (configurando su Topic en `/submap_list`).

#### Terminal 5: Controlador Reactivo Autónomo
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run localization_lab simple_controller
```

#### Terminal 6: Persistencia y Exportación del Mapa
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/mapas && cd ~/ros2_ws/mapas
ros2 run nav2_map_server map_saver_cli -f my_map
```

---







