<div align="center">

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/0a/Logotipo_de_la_Universidad_Nacional_de_Colombia.svg/600px-Logotipo_de_la_Universidad_Nacional_de_Colombia.svg.png" width="180" alt="Logo UNAL">

# UNIVERSIDAD NACIONAL DE COLOMBIA
### SEDE MANIZALES
**Facultad de Ingeniería y Arquitectura**  
**Departamento de Ingeniería Eléctrica, Electrónica y Computación**  

---

## 🤖 Laboratorio de Robótica Móvil y ROS 2 (2026-2S)
**Profesor:** Jaime Arango, PhD  

---

### 👥 Integrantes del Equipo
* **Joel Américo López Valdez** — [@joelopezva-bit](https://github.com/joelopezva-bit)  
* **Cleyson Valerio Yauri Cupara** — [@cyauri-hub](https://github.com/cyauri-hub)  


---

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white" alt="ROS 2">
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?logo=gazebo&logoColor=white" alt="Gazebo">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white" alt="Python">
</p>

</div>

---

## 📚 Índice de Laboratorios Desarrollados

A continuación se presentan los diferentes módulos prácticos desarrollados a lo largo del curso. Haz clic en el título de cada laboratorio para acceder a sus códigos fuente, gráficas comparativas y análisis técnico:

| Módulo / Laboratorio | Temas Principales | Herramientas Utilizadas | Enlace de Acceso |
| :--- | :--- | :--- | :---: |
| **Capítulo 2: Pose y Cuaterniones** | Representación espacial en $SO(3)$, álgebra de cuaterniones, no conmutatividad y transformaciones dinámicas con TF2. | Python, SciPy, ROS 2 TF2, RViz2 | [🔗 Ver Práctica](./capitulo_2_cuaterniones/) |
| **Capítulo 3: Cinemática y Trayectorias** | Integración de velocidad angular, integración inercial tipo IMU, trayectorias polinómicas y trapezoidales, SLERP. | Robotics Toolbox, SciPy, RViz2 | [🔗 Ver Práctica](./capitulo_3_trayectorias/) |
| **Capítulo 4: Modelado y Control** | Cinemática de modelo bicicleta y uniciclo, control Go-to-Goal, seguimiento de línea y persecución pura (*Pure Pursuit*). | TurtleBot3, `/cmd_vel`, `/odom`, Gazebo | [🔗 Ver Práctica](./capitulo_4_control_movil/) |
| **Capítulo 5: De lo Reactivo a Nav2** | Vehículos de Braitenberg, evasión con Bug2, planificación $A^*$, Distance Transform e integración con Nav2. | Python, Algoritmos de Grafos, Nav2 Stack | [👉 **Ir al Laboratorio 5**](./laboratorio_5_navegacion/) |
| **Capítulo 6: Localización y SLAM** | Deriva en odometría en lazo abierto, estimación con EKF (predicción), corrección con landmarks y EKF-SLAM completo. | Robotics Toolbox, EKF, Mapas de Balizas | [👉 **Ir al Laboratorio 6**](./laboratorio_6_localizacion_slam/) |

---

## 🛠️ Especificaciones del Entorno de Simulación
* **Sistema Anfitrión:** Windows 11 (64 bits) | Intel Core i5-11400H | 16 GB RAM | NVIDIA RTX Serie 30
* **Virtualizador:** Oracle VM VirtualBox 7.x con Aceleración 3D habilitada
* **Máquina Virtual:** Ubuntu 24.04 LTS (8 GB RAM asignados, 4 vCPUs, 80 GB almacenamiento dinámico)
* **Middleware Robótico:** ROS 2 Jazzy Jalisco instalado de forma nativa desde repositorios oficiales

---

## 📌 Metodología de Trabajo y Versionamiento
1. **Desarrollo Experimental:** Implementación de scripts base y formulación de casos de variación de parámetros para análisis de robustez estocástica.
2. **Registro Audiovisual:** Animaciones y grabaciones dinámicas en tiempo real publicadas en canales de video para demostración de convergencia.
3. **Documentación:** Respaldo completo de código y formulación de reportes técnicos formales en formato PDF.
