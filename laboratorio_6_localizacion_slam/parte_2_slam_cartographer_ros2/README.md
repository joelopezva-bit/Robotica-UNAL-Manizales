# Parte 2: SLAM 2D con Cartographer en ROS 2 Jazzy y Controlador Reactivo

**Curso:** Robótica Móvil  
**Institución:** Universidad Nacional de Colombia - Sede Manizales  
**Estudiantes:** Joel Américo López Valdez & Cleyson Valerio Yauri Cupara  
**Origen:** Movilidad Académica UNSAAC (Perú)  
**Entorno de Ejecución:** Ubuntu 24.04 LTS | ROS 2 Jazzy Jalisco | Gazebo Harmonic | RViz2  

---

## 📌 Descripción Técnica
Esta sección valida las técnicas de localización y mapeo sobre un sistema distribuido real en ROS 2. Se implementó el algoritmo **Cartographer** acoplado al modelo cinemático del **TurtleBot3 Burger** simulado en **Gazebo Harmonic**. 

Adicionalmente, se desarrolló un **controlador reactivo en Python** que procesa en tiempo real los datos del sensor LiDAR (`/scan`), detecta obstáculos frontales en un cono de $60^\circ$ a distancias menores a $0.50\text{ m}$, y comanda maniobras de giro autónomas para recorrer el entorno de forma continua sin colisiones.

---

## 📂 Archivos del Paquete ROS 2 y Mapas

Todos los scripts y configuraciones de esta fase se encuentran dentro de las carpetas `paquete_ros2_slam/` y `mapas_generados/`:

| Archivo / Directorio | Ubicación Relativa | Función Técnica |
| :--- | :--- | :--- |
| `simple_controller.py` | `paquete_ros2_slam/localization_lab/` | Nodo de control reactivo en Python. Suscrito a `/scan`, publica `TwistStamped` en `/cmd_vel`. |
| `cartographer.lua` | `paquete_ros2_slam/config/` | Configuración de Cartographer: frames (`base_scan`, `odom`, `map`), optimización de submapas y pose graph. |
| `cartographer.launch.py` | `paquete_ros2_slam/launch/` | Lanzador que ejecuta `cartographer_node` y `cartographer_occupancy_grid_node` (resolución 0.05 m). |
| `package.xml` | `paquete_ros2_slam/` | Definición de dependencias (`rclpy`, `sensor_msgs`, `geometry_msgs`, `cartographer_ros`). |
| `setup.py` | `paquete_ros2_slam/` | Registro de puntos de entrada de consola para el comando `ros2 run`. |
| `my_map.pgm` | `mapas_generados/` | Mapa rasterizado bidimensional en escala de grises generado por Cartographer. |
| `my_map.yaml` | `mapas_generados/` | Metadatos espaciales, resolución ($0.05\text{ m/px}$) y origen cartográfico. |

---

## 🎥 Evidencias en Video

* 🎬 **Parte 5.1 (Ejecución del Controlador Reactivo en Gazebo):** [Ver en YouTube (Pendiente)]()
* 🎬 **Parte 5.2 (Prueba de Evasión Autónoma de Obstáculos):** [Ver en YouTube (Pendiente)]()
* 🎬 **Parte 5.3 (Mapeo SLAM en Tiempo Real con RViz2):** [Ver en YouTube (Pendiente)]()

---

## 💻 Instrucciones de Reproducción (Pipeline de ROS 2)

Para reproducir este laboratorio, copie el contenido de `paquete_ros2_slam` hacia su workspace local (`~/ros2_ws/src/localization_lab`), compile con `colcon build --symlink-install` y ejecute los nodos en terminales independientes:

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
