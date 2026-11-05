<div align="center">

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/0a/Logotipo_de_la_Universidad_Nacional_de_Colombia.svg/600px-Logotipo_de_la_Universidad_Nacional_de_Colombia.svg.png" width="150" alt="Logo UNAL">

# Proyecto Final: Robot Autónomo de Inspección con Visión Artificial
### Sistema de Patrullaje Industrial en Simulación 100% con TurtleBot3, ROS 2 Jazzy y Gazebo Harmonic

**Asignatura:** Robótica Móvil (2026-2S)  
**Profesor:** Jaime Arango, PhD  
**Institución:** Universidad Nacional de Colombia - Sede Manizales  
* **Cleyson Valerio Yauri Cupara** — [@cyauri-hub](https://github.com/cyauri-hub)  
* **Joel Américo López Valdez** — [@joelopezva-bit](https://github.com/joelopezva-bit)  

---

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white" alt="ROS 2">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic%20(gz--sim8)-FF6F00?logo=gazebo&logoColor=white" alt="Gazebo">
  <img src="https://img.shields.io/badge/OpenCV-4.x%20(cv2.aruco)-5C3EE8?logo=opencv&logoColor=white" alt="OpenCV">
  <img src="https://img.shields.io/badge/Status-Fase%201%3A%20Diseño%20y%20Fundamentación-blue" alt="Status">
</p>

</div>

---

## 📄 Documentación y Memoria Técnica Oficial (Avance 1)

El reporte formal de fundamentación conceptual, estudio comparativo de sensores y selección metodológica se encuentra catalogado para visualización digital directa en:

👉 **[Ver Documento Completo: Avance 1 - Fundamentación y Pipeline de Visión Artificial (PDF)](./documentacion/Avance_1_Fundamentacion_Vision_Artificial.pdf)**

---

## 📌 Planteamiento del Problema y Propuesta de Solución

* **Problemática Operativa:** Las inspecciones rutinarias de tableros industriales o paneles energéticos suelen realizarse mediante rondas manuales que exponen al personal a riesgos y carecen de registros auditables. En robótica móvil convencional, muchas soluciones sufren de "patrullaje ciego": el robot llega a coordenadas $(X, Y)$ sin garantizar que el sensor enfoque el panel, analiza un único fotograma en movimiento (produciendo falsos positivos por desenfoque) y no distingue entre un estado operativo normal y un fallo en la cámara.
* **Solución Propuesta:** Implementar un sistema de patrullaje e inspección autónoma en simulación donde un TurtleBot3 recorre secuencialmente tres estaciones de control ($P_1$, $P_2$, $P_3$). En cada punto, el robot alinea activamente su orientación angular ($\text{yaw}$), detiene su marcha para estabilizar la captura y ejecuta algoritmos de **Visión Artificial Clásica con OpenCV**:
  1. Identificación unívoca de la estación mediante **marcadores fiduciales ArUco**.
  2. Diagnóstico operativo mediante segmentación cromática en **espacio de color HSV** (Verde = Operación Normal, Rojo = Alerta/Falla).
  3. Clasificación robusta por **votación temporal sobre 5 fotogramas** y registro de trazabilidad con volcado de evidencias en CSV/JSON e imágenes anotadas.

---

## 🧠 Justificación Metodológica: ¿Por qué iniciar por Visión Artificial?

Por recomendación docente y criterios de diseño en ingeniería, el desarrollo se inició atacando el componente de mayor fricción técnica:
1. **Superación del Salto Tecnológico Gazebo Classic $\rightarrow$ Harmonic:** En Ubuntu 24.04 y ROS 2 Jazzy, el plugin clásico `libgazebo_ros_camera.so` está obsoleto. Gazebo Harmonic requiere declarar sensores desacoplados (`<sensor type="camera">`) procesados por el plugin de sistema `gz::sim::systems::Sensors` y canalizados mediante puentes dedicados (`ros_gz_image` / `ros_gz_bridge`).
2. **Principio de Aislamiento de Fallos:** Desacoplar la percepción visual del sistema de navegación (Nav2 + AMCL) permite calibrar iluminación, distancias focales y diccionarios ArUco sin que la deriva odometría distorsione las pruebas.
3. **Presupuesto Computacional en Entorno Virtual:** Dimensionar la resolución y tasa de refresco de la cámara evita sobrecargar la GPU/CPU de la máquina virtual VirtualBox antes de cargar los planificadores globales y locales de Nav2.

---

## 🛠️ Pila Tecnológica y Ecosistema de Librerías

| Componente / Paquete | Capa Arquitectónica | Función Específica en el Proyecto |
| :--- | :--- | :--- |
| **Gazebo Harmonic (`gz-sim8`)** | Simulación Física | Entorno 3D con motor gráfico `ogre2` y sistema `gz::sim::systems::Sensors`. |
| **`ros_gz_bridge` / `ros_gz_image`** | Puente Middleware | Enlace unidireccional de alto rendimiento entre `gz.msgs.Image` y `sensor_msgs/msg/Image`. |
| **`cv_bridge` (`ros-jazzy-cv-bridge`)** | Adaptador de Datos | Deserialización de mensajes ROS 2 a arreglos multidimensionales `numpy.ndarray` en formato BGR8. |
| **OpenCV 4.x (`cv2` + `cv2.aruco`)** | Visión Artificial | Detección de esquinas/IDs ArUco, máscaras morfológicas y segmentación HSV. |
| **NumPy (`python3-numpy`)** | Cálculo Matricial | Operaciones de umbralización vectorial y cálculo de razones de área de píxeles activos. |
| **`rqt_image_view` / RViz2** | Diagnóstico y Auditoría | Herramientas de telemetría gráfica para inspeccionar el flujo visual crudo y las imágenes procesadas. |

---

## 📷 Estudio de Selección y Geometría de la Cámara

* **Tipo de Sensor:** **Cámara RGB Monocular (`type="camera"`)**. Se descartaron las cámaras de profundidad pura (no capturan texturas ni color) y las cámaras RGB-D (costo computacional prohibitivo en VirtualBox; la estimación de distancia se realizará mediante `camera_info` y `cv2.solvePnP`).
* **Configuración del Sensor:** Resolución de $640 \times 480$ píxeles a $15\text{ Hz}$ con un campo visual horizontal ($\text{HFOV}$) de $1.085\text{ rad}$ ($\approx 62.2^\circ$).
* **Ubicación Física en Chasis:** Frente del robot a $x \approx 0.07\text{ m}$ del centro de rotación y $z \approx 0.12\text{ m}$ de altura, ubicándose estratégicamente debajo del plano de barrido del LiDAR (`base_scan`) para evitar oclusiones.
* **Transformación Cinemática:** Diferenciación explícita entre el soporte mecánico `camera_link` (convención robótica FLU: $X$ adelante, $Y$ izquierda, $Z$ arriba) y el marco óptico `camera_optical_frame` (convención RDF de OpenCV: $Z$ profundidad/lente, $X$ ancho de imagen, $Y$ altura de imagen).

---

## 🗺️ Hoja de Ruta Progresiva de Trabajo
