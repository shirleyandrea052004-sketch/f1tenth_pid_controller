# F1TENTH Global Path Planner — Dijkstra + Cubic/PCHIP Smoothing (AutoDRIVE Simulator)

Proyecto de Mapeo y Planificación Global de Trayectorias, desarrollado sobre el simulador **AutoDRIVE** (F1TENTH) usando **ROS 2 (Humble)**. El sistema construye un mapa 2D del entorno mediante **SLAM Toolbox**, planifica una ruta global entre un punto de inicio y una meta usando **Dijkstra**, y suaviza la trayectoria resultante con un **spline PCHIP** (shape-preserving) para obtener una curva continua y cinemáticamente viable.

## 🎥 Demostraciones Visuales

📺 **[Generación de mapa con SLAM](https://www.youtube.com/watch?v=9VNgWsDzoU8)**

📺 **[Movimiento del auto en AutoDRIVE sobre el mapa y trayectoria generada](https://www.youtube.com/watch?v=JdyhFhaaJMo)**

A continuación, se observa el funcionamiento de los algoritmos:

**Generación de la trayectoria mediante Dijkstra:**  
![Generación de trayectoria](docs/media/generacion_trayectoria.gif)

**Comparación visual: Ruta cruda (Dijkstra) vs. Ruta suavizada (PCHIP):**  
![Comparación cruda vs suavizada](docs/media/comparacion_cruda_vs_suavizada.gif)

> **Nota:** La visualización final del `Path` superpuesto sobre el vehículo en movimiento dentro del simulador AutoDRIVE se puede apreciar a detalle en los videos enlazados arriba.

## 📂 Estructura del repositorio

```text
.
├── docs/
│   ├── debug_images/            # Capturas de diagnóstico usadas durante el desarrollo
│   └── media/                   # GIFs y recursos multimedia del README
├── scripts/                     # Herramientas de desarrollo (no forman parte del paquete ROS2)
│   ├── visualize_map.py         # Visualiza el mapa (.pgm/.yaml) con grilla de coordenadas del mundo
│   └── pick_start_goal.py       # Elige visualmente start/goal haciendo clic sobre el mapa inflado
└── src/
    └── f1tenth_global_planner/  # Paquete ROS2 (ament_python)
        ├── f1tenth_global_planner/
        │   ├── global_planner_node.py     # Dijkstra sobre el OccupancyGrid inflado
        │   ├── path_smoother_node.py      # Simplificación (Douglas-Peucker) + spline PCHIP
        │   └── sim_tf_broadcaster_node.py # Publica TF map->f1tenth_1 (ips + imu del simulador)
        ├── launch/
        │   ├── map_launch.py         # Solo mapa (map_server + lifecycle_manager)
        │   ├── planning_launch.py    # Mapa + Dijkstra + suavizado + RViz
        │   └── demo_launch.py        # Pipeline completo + TF broadcaster (para grabar con el simulador)
        ├── maps/
        │   ├── F1tenth_Map.{pgm,yaml}         # Mapa crudo generado con SLAM Toolbox
        │   └── F1tenth_Map_walled.{pgm,yaml}  # Copia con una "pared virtual" (ver explicación abajo)
        ├── config/
        │   └── mapper_params_online_async.yaml  # Parámetros de SLAM Toolbox usados para el mapeo
        ├── package.xml
        └── setup.py
```

## 🔧 Dependencias

Probado en **Ubuntu 22.04 + ROS 2 Humble**.

| Paquete | Notas |
|---|---|
| `ros-humble-desktop` | Instalación base de ROS 2 |
| `ros-humble-nav2-map-server` | Sirve el mapa como `OccupancyGrid` |
| `ros-humble-nav2-lifecycle-manager` | Activa `map_server` automáticamente |
| `ros-humble-slam-toolbox` | Usado en la etapa de mapeo (Parte A) |
| `python3-numpy`, `python3-scipy` | Dijkstra, transformada de distancia, interpolación PCHIP |
| `python3-matplotlib`, `python3-pillow`, `python3-yaml` | Solo para los scripts de `scripts/` (no se instalan con el paquete ROS2) |
| [AutoDRIVE Simulator + `autodrive_ros2`](https://github.com/Tinker-Twins/AutoDRIVE) | Simulador y bridge ROS2 (ver instrucciones oficiales del devkit) |

Instalación rápida de dependencias de sistema:

```bash
sudo apt update
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
                 ros-humble-slam-toolbox ros-humble-rviz2

pip3 install numpy scipy matplotlib pillow pyyaml --user
# Si tu Ubuntu da error de "externally managed environment":
# pip3 install numpy scipy matplotlib pillow pyyaml --break-system-packages
```

## 🚀 Instalación

1. Clona este repositorio dentro del `src/` de tu workspace de ROS 2 (o usa un symlink, como
   se hizo durante el desarrollo):

   ```bash
   cd ~/autodrive_ws/src
   git clone https://github.com/shirleyandrea052004-sketch/f1tenth-dijkstra-global-planner.git
   ln -s ~/f1tenth-dijkstra-global-planner/src/f1tenth_global_planner f1tenth_global_planner
   ```

   *(Alternativa sin symlink: copia directamente `f1tenth-dijkstra-global-planner/src/f1tenth_global_planner`
   a `~/autodrive_ws/src/`).*

2. Compila el paquete:

   ```bash
   cd ~/autodrive_ws
   colcon build --packages-select f1tenth_global_planner --symlink-install
   source install/setup.bash
   ```

## ▶️ Uso

### Opción A — Solo planificación (mapa estático, sin simulador)

Prueba rápida de Dijkstra + suavizado sobre el mapa guardado, sin necesidad de levantar
AutoDRIVE:

```bash
ros2 launch f1tenth_global_planner planning_launch.py
```

Esto levanta `map_server` (con la "pared virtual", ver más abajo), el nodo de Dijkstra, el
nodo de suavizado, y RViz. En RViz, agrega manualmente (si no aparecen ya en la config):

- **Map** → topic `/map`
- **Path** → topic `/raw_path` (ruta cruda de Dijkstra)
- **Path** → topic `/smoothed_path` (ruta suavizada)

Puedes disparar una nueva planificación en cualquier momento usando la herramienta
**"2D Nav Goal"** de la barra superior de RViz y haciendo clic sobre el mapa.

### Opción B — Demo completa con el simulador AutoDRIVE

1. Levanta el simulador Unity:
   ```bash
   cd ~/Downloads/AutoDRIVE_Sim
   ./"AutoDRIVE Simulator.x86_64"
   ```
2. Levanta el bridge ROS2:
   ```bash
   cd ~/autodrive_ws
   source install/setup.bash
   ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
   ```
3. Levanta el pipeline de planificación + suavizado + TF del vehículo:
   ```bash
   ros2 launch f1tenth_global_planner demo_launch.py
   ```
4. (Opcional) Mueve el vehículo para ver el `Path` alineado con su posición real:
   ```bash
   ros2 run autodrive_f1tenth teleop_keyboard
   ```

En el RViz que abre `demo_launch.py`, agrega el display **TF** (By display type → TF) para ver
el frame `f1tenth_1` (posición real del vehículo en el simulador) moviéndose sobre el mapa y
el `Path` planificado.

## 🧠 Algoritmos y decisiones de diseño

### Parte A — Mapeo (SLAM Toolbox)

Se siguió el tutorial oficial *["Generating a 2D Map of the F1TENTH in AutoDRIVE Using SLAM
Toolbox"](#)*, moviendo manualmente el vehículo por el circuito con teleoperación y
guardando el mapa resultante (`F1tenth_Map.pgm` / `.yaml`) con `map_saver_cli`. Los
parámetros de SLAM Toolbox usados están en `config/mapper_params_online_async.yaml`.

### Parte B — Planificación global (Dijkstra)

`global_planner_node.py`:
- Se suscribe a `/map` (QoS `TRANSIENT_LOCAL`, ya que `map_server` publica en modo *latched*).
- **Infla los obstáculos** según el radio del robot (`robot_radius`, por defecto 0.20 m)
  usando una transformada de distancia (`scipy.ndimage.distance_transform_edt`): cualquier
  celda libre a menos de `robot_radius` metros de un obstáculo se marca como ocupada.
- Convierte el grid en un grafo implícito con **conectividad-8** (permite movimientos
  diagonales, costo `√2`) y corre **Dijkstra** (con `heapq` como cola de prioridad) desde el
  `start` hasta el `goal`.
- El `start`/`goal` se pueden fijar por parámetro (`start_x`, `start_y`, `goal_x`, `goal_y`)
  o sobrescribir en vivo desde RViz (`2D Pose Estimate` / `2D Nav Goal`).
- Publica la ruta resultante en `/raw_path` (`nav_msgs/Path`).

**Sobre la "pared virtual" (`F1tenth_Map_walled.pgm`):** el circuito de este mapa es un
*loop* cerrado (dos carriles paralelos conectados por curvas en ambos extremos). Para forzar
que Dijkstra recorra (casi) el circuito completo en vez de tomar el atajo más corto entre dos
puntos cercanos, se editó una copia del mapa (con GIMP) agregando una pared corta que cierra
un carril justo en el punto de inicio/meta. Así, el `start` y el `goal` quedan geométricamente
muy cerca pero *separados* por esa pared, obligando a la ruta a dar toda la vuelta. El mapa
original (`F1tenth_Map.pgm`), sin modificar, se conserva como evidencia cruda de SLAM.

### Parte B — Suavizado (Douglas-Peucker + PCHIP)

`path_smoother_node.py`:
1. **Simplificación (Douglas-Peucker):** en vez de ajustar una curva que pase por los ~400+
   puntos crudos de Dijkstra (lo cual produciría una curva casi idéntica al zigzag original),
   se reduce la ruta a un subconjunto mínimo de puntos de control que aún representan
   fielmente su forma (tolerancia `simplify_epsilon`, por defecto 0.15 m).
2. **Interpolación PCHIP** (`scipy.interpolate.PchipInterpolator`), parametrizada por
   longitud de arco acumulada (`x(s)`, `y(s)`), en vez de un Cubic Spline "natural". Se eligió
   PCHIP porque es *shape-preserving*: no genera *overshoot* (sobre-oscilación) en giros
   cerrados, un problema real que apareció en las primeras pruebas de este proyecto cuando la
   curva suavizada se salía de la pista en la esquina cerrada cerca del punto de inicio/meta.
3. **Validación + refinamiento local:** el nodo valida cada punto de la curva suavizada contra
   el mapa (con un margen de seguridad `safety_radius`, menor al `robot_radius` del
   planificador ya que aquí solo se busca evitar cruzar una pared, no mantener distancia de
   maniobra). Si algún tramo invade una zona ocupada, se agrega **localmente** un punto de
   control adicional (el punto crudo más cercano a esa zona) y se reajusta el spline — sin
   afectar la simplificación del resto de la ruta, que puede mantenerse ampliamente suavizada.
4. Publica el resultado en `/smoothed_path`, incluyendo orientación (yaw) tangente a la curva
   en cada punto.

### Variables/parámetros importantes

| Nodo | Parámetro | Default | Descripción |
|---|---|---|---|
| `global_planner_node` | `robot_radius` | `0.20` | Margen de seguridad (m) para inflar obstáculos antes de correr Dijkstra |
| `global_planner_node` | `start_x/y`, `goal_x/y` | ver `demo_launch.py` | Punto de inicio/meta por defecto |
| `global_planner_node` | `use_rviz_goals` | `True` | Permite sobrescribir start/goal desde RViz |
| `path_smoother_node` | `points_per_meter` | `10.0` | Densidad de muestreo de la curva final |
| `path_smoother_node` | `simplify_epsilon` | `0.15` | Tolerancia (m) de Douglas-Peucker |
| `path_smoother_node` | `safety_radius` | `0.08` | Margen de seguridad (m) usado solo para validar el suavizado |
| `path_smoother_node` | `max_local_refinements` | `25` | Máximo de iteraciones de refinamiento local antes de usar la ruta cruda como respaldo |

## 🛠️ Herramientas de desarrollo (`scripts/`)

Estos scripts corren de forma independiente (sin ROS2) y se usaron para depurar la elección
de coordenadas de start/goal directamente sobre el mapa:

- **`visualize_map.py`**: genera una imagen del mapa con grilla de coordenadas del mundo real
  superpuesta.
  ```bash
  python3 scripts/visualize_map.py src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml
  ```

- **`pick_start_goal.py`**: carga el mapa, aplica el mismo inflado de obstáculos que usa el
  nodo de Dijkstra, y permite hacer clic directamente sobre la imagen para elegir start/goal,
  validando en tiempo real si el punto es libre.
  ```bash
  python3 scripts/pick_start_goal.py src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml --radius 0.20
  ```

## 📌 Notas y limitaciones conocidas

- El mapa tiene una pequeña franja de ruido de mapeo en el borde derecho del circuito
  (aprox. `x∈[1.3, 1.6]`, `y∈[-2, 2]`); no interfiere con la ruta planificada.
- La "pared virtual" es una decisión de diseño para forzar la vuelta completa al circuito;
  no representa un obstáculo real del entorno.

## Autor

Shirley Andrea — Proyecto individual, algoritmo asignado: **Dijkstra**.