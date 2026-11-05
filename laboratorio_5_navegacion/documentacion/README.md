# 📚 Acervo Documental y Reportes Técnicos — Laboratorio 5

Este repositorio documental centraliza los informes académicos, memorias de cálculo y análisis formal correspondientes a las prácticas de **Navegación de Robots Móviles**.

---

## 📑 Índice de Documentos Disponibles

### 1. Reporte Técnico Consolidado (Parte 1: Módulos A al D)
* **Archivo:** [`Informe_Laboratorio_5_Navegacion.pdf`](./Informe_Laboratorio_5_Navegacion.pdf)
* **Formato:** PDF / Compilado oficial (21 páginas)
* **Estado:** Aprobado / Versión Final
* **Autores:** Cleyson Valerio Yauri Cupara & Joel Américo López Valdez
* **Filiación:**  UNAL Sede Manizales

👉 **[Haga clic aquí para visualizar el PDF completo en GitHub](./Informe_Laboratorio_5_Navegacion.pdf)**

---

### 🔍 Síntesis Temática del Informe

| Sección | Algoritmo / Enfoque | Principio Matemático / Físico | Conclusiones Clave |
| :--- | :--- | :--- | :--- |
| **Parte A** | Vehículo de Braitenberg | Control proporcional acoplado por campo de potencial escalar: $f(x,y)=\frac{1000}{(x-x_c)^2+(y-y_c)^2+50}$ | Demostró convergencia reactiva sin mapa; se evidenció riesgo de sobrepaso (*overshoot*) por saturación de velocidad lineal ($v \propto \sum s$). |
| **Parte B** | Algoritmo Bug2 | Evasión local mediante seguimiento de bordes ortogonales y retorno a la línea $m$ | Resuelve obstáculos convexos en 32 pasos; susceptible a atraparse en mínimos locales ante geometrías en "U". |
| **Parte C** | Planificación $A^*$ | Búsqueda en grafos con heurística euclidiana admisible: $f(n) = g(n) + h(n)$ | Garantizó ruta óptima explorando 305 nodos, reduciendo drásticamente el espacio de búsqueda frente a UCS (358 nodos). |
| **Parte D** | Transformada de Distancia | Inundación iterativa de grilla (4-conectividad) y descenso de gradiente | Genera una función libre de mínimos locales resolviendo laberintos complejos; alto costo computacional ($\approx 40,000$ operaciones). |

---

## 📌 Historial de Actualizaciones y Próximas Incorporaciones
- [x] **v1.0 (Noviembre 2026):** Incorporación del reporte técnico de algoritmos clásicos y heurísticos (Partes A, B, C y D).
- [ ] **v2.0 (Pendiente):** Anexo de la Parte E (Navegación Autónoma en ROS 2 Jazzy, Gazebo Harmonic y Nav2 Stack).
