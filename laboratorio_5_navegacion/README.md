# Laboratorio 5: Navegación de Robots Móviles — De lo Reactivo a ROS 2 Nav2 (Parte 1)

**Curso:** Robótica Móvil  
**Institución:** Universidad Nacional de Colombia - Sede Manizales  
**Estudiantes:** Cleyson Valerio Yauri Cupara & Joel Américo López Valdez   
**Profesor:** Jaime Arango, PhD  

---

## 📌 Descripción General
Este directorio contiene el desarrollo de las actividades correspondientes a la **Parte 1 (Módulos A al D)** del Capítulo 5. Se exploran las filosofías fundamentales de navegación robótica:
1. **Navegación Reactiva:** Vehículo de Braitenberg guiado por gradiente escalar de estímulo sin mapa.
2. **Navegación con Evasión Local:** Algoritmo Bug2 sobre grilla discreta con seguimiento de contornos.
3. **Planificación Global en Grafos:** Algoritmo $A^*$ y análisis de admisibilidad heurística.
4. **Planificación de Campos Escalares:** Transformada de Distancia (*Distance Transform*) y descenso de gradiente.

---

## 📂 Organización de Scripts y Códigos Fuente

Todos los scripts ejecutables validados se encuentran en la carpeta `scripts/`:

| Archivo | Actividad | Algoritmo / Principio | Parámetros Clave |
| :--- | :--- | :--- | :--- |
| `braitenberg_base.py` | Parte A | Vehículo reactivo de Braitenberg | $K_v=14, K_s=17, dt=0.1, N=222$ |
| `bug2_base.py` | Parte B | Algoritmo Bug2 en grilla con línea-m | Grid $20\times 20$, obstáculo central |
| `astar_base.py` | Parte C | Planificación de trayectoria $A^*$ | Conectividad 4-vecinos, heurística euclidiana |
| `dt_base.py` | Parte D | Transformada de distancia (Distance Transform) | Inundación iterativa y descenso de gradiente |

---

## 🎥 Demostraciones Dinámicas en Video

Las evidencias dinámicas de la ejecución de los algoritmos se encuentran documentadas en los siguientes enlaces:

* 🎬 **Parte A — Comportamiento y Trayectoria del Vehículo Braitenberg:** [Ver video en YouTube](https://youtu.be/-zC6gXDeC58)
* 🎬 **Parte B — Evasión de Obstáculo y Retorno a Línea-m (Bug2):** [Ver video en YouTube](https://youtu.be/YZrKQCMLAZY)

---

## 📊 Síntesis de Resultados y Preguntas de Análisis

### Parte A: Navegación Reactiva (Braitenberg)
* **Influencia de la diferencia sensorial:** La discrepancia $s_{\text{left}} - s_{\text{right}}$ modula la velocidad angular $\omega = K_s(s_{\text{right}} - s_{\text{left}})$. Cuando el sensor izquierdo percibe mayor intensidad, el robot rota en sentido horario para orientarse al foco.
* **Comportamiento en proximidad al objetivo:** Al acercarse al objetivo, la intensidad en ambos sensores se incrementa drásticamente, disparando la velocidad lineal $v$ hacia valores elevados mientras $\omega \to 0$. Esto puede inducir sobrepaso (*overshoot*) si no se implementa una ley de frenado.
* **Limitaciones de iteraciones extendidas:** Al aumentar de $N=222$ a $N=600$, el robot sobrepasa la meta y entra en órbitas divergentes al carecer de una condición de parada explícita.

### Parte B: Evasión con Bug2
* **Optimalidad del camino:** No es óptimo. El algoritmo toma decisiones locales reactivas siguiendo el contorno al encontrar un obstáculo, recorriendo más celdas que un planificador global.
* **Entornos complejos:** En obstáculos cóncavos (trampas en "U") o laberintos cerrados, las reglas locales provocan atrapamiento en mínimos locales o ciclos oscilatorios.

### Parte C: Planificación Óptima con $A^*$
* **Diferencia entre BFS, UCS y $A^*$:** BFS asume costos idénticos sin noción de meta. UCS expande nodos evaluando el costo acumulado $g(n)$ de forma radial. $A^*$ evalúa $f(n) = g(n) + h(n)$, combinando el costo real con una heurística que orienta la búsqueda hacia la meta, reduciendo los nodos explorados de 358 (UCS) a 305 ($A^*$).
* **Efecto de heurística sobrestimada:** Si $h(n)$ sobrestima el costo real, la búsqueda se vuelve voraz y rápida (72 nodos explorados), pero pierde la garantía de optimalidad matemática, ciñéndose de manera riesgosa a las esquinas de los obstáculos.

### Parte D: Transformada de Distancia
* **Garantía de optimalidad:** La función de navegación generada es monótona y libre de mínimos locales; cualquier celda libre desciende estrictamente hacia el sumidero global.
* **Costo computacional:** Complejidad de orden $\mathcal{O}(N \times M \times \text{iteraciones})$, requiriendo barrer toda la matriz de ocupación repetidamente ($\approx 40,000$ operaciones en una grilla de $20\times 20$), siendo menos eficiente que $A^*$ en mapas de gran escala.

---

## 🚀 Instrucciones de Ejecución

Para reproducir cualquiera de los experimentos:

```bash
# Clonar repositorio
git clone [https://github.com/joelopezva-bit/Robotica-UNAL-Manizales.git](https://github.com/joelopezva-bit/Robotica-UNAL-Manizales.git)
cd Robotica-UNAL-Manizales/laboratorio_5_navegacion/scripts

# Ejecutar el algoritmo deseado (ejemplo: A*)
python3 astar_base.py
