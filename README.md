# F1TENTH Global Path Planner (Dijkstra + PCHIP) & PID Trajectory Tracking — AutoDRIVE Simulator

Proyecto de Mapeo, Planificación Global y Control de Trayectorias, desarrollado sobre el
simulador **AutoDRIVE** (F1TENTH) usando **ROS 2 (Humble)**. El sistema:

1. Construye un mapa 2D del entorno mediante **SLAM Toolbox** (Parte A).
2. Planifica una ruta global entre un punto de inicio y una meta usando **Dijkstra**, y la
   suaviza con un **spline PCHIP** (shape-preserving) para obtener una curva continua y
   cinemáticamente viable (Parte B — paquete [`f1tenth_global_planner`](src/f1tenth_global_planner/)).
3. Sigue esa trayectoria de forma autónoma en el simulador mediante un **controlador PID**
   de seguimiento de trayectoria, con conteo de vueltas y cronómetro (Parte C — paquete
   [`f1tenth_pid_controller`](src/f1tenth_pid_controller/)).

## 🎥 Demostraciones Visuales

📺 **[Generación de mapa con SLAM](https://www.youtube.com/watch?v=9VNgWsDzoU8)**

📺 **[Movimiento del auto en AutoDRIVE sobre el mapa y trayectoria generada](https://www.youtube.com/watch?v=JdyhFhaaJMo)**

📺 **Control autónomo (PID) — video de evidencia: pendiente de enlace.**
> Debe mostrar al vehículo recorriendo la pista de forma autónoma sin chocar, la terminal
> con el contador de vueltas y la terminal con el cronómetro por vuelta.

A continuación, se observa el funcionamiento de los algoritmos:

**Generación de la trayectoria mediante Dijkstra:**
![Generación de trayectoria](docs/media/generacion_trayectoria.gif)

**Comparación visual: Ruta cruda (Dijkstra) vs. Ruta suavizada (PCHIP):**
![Comparación cruda vs suavizada](docs/media/comparacion_cruda_vs_suavizada.gif)

**Trayectoria final (mapa + waypoints crudos y suavizados superpuestos):**
![Trayectoria final](docs/media/trajectory_overlay.png)

> **Nota:** La visualización final del `Path` superpuesto sobre el vehículo en movimiento
> (tanto en teleoperación como en modo autónomo con el PID) se puede apreciar a detalle en
> los videos enlazados arriba.

## 📂 Estructura del repositorio

```text
.
├── docs/
│   ├── debug_images/            # Capturas de diagnóstico usadas durante el desarrollo
│   └── media/                   # GIFs, imagen de trayectoria final y recursos multimedia del README
├── waypoints/                    # Waypoints generados (CSV) de la ruta cruda y suavizada
│   ├── raw_path.csv
│   └── smoothed_path.csv
├── scripts/                     # Herramientas de desarrollo (no forman parte de ningún paquete ROS2)
│   ├── visualize_map.py         # Visualiza el mapa (.pgm/.yaml) con grilla de coordenadas del mundo
│   ├── pick_start_goal.py       # Elige visualmente start/goal haciendo clic sobre el mapa inflado
│   └── plot_waypoints.py        # Genera la imagen final (mapa + waypoints crudos y suavizados)
└── src/
    ├── f1tenth_global_planner/  # Paquete ROS2 (ament_python) — Parte A y B
    │   ├── f1tenth_global_planner/
    │   │   ├── global_planner_node.py     # Dijkstra (con centrado de carril) sobre el OccupancyGrid inflado
    │   │   ├── path_smoother_node.py      # Simplificación (Douglas-Peucker) + spline PCHIP
    │   │   └── sim_tf_broadcaster_node.py # Publica TF map->f1tenth_1 (ips + imu del simulador)
    │   ├── launch/
    │   │   ├── map_launch.py         # Solo mapa (map_server + lifecycle_manager)
    │   │   ├── planning_launch.py    # Mapa + Dijkstra + suavizado + RViz
    │   │   └── demo_launch.py        # Pipeline completo + TF broadcaster (para grabar con el simulador)
    │   ├── maps/
    │   │   ├── F1tenth_Map.{pgm,yaml}         # Mapa crudo generado con SLAM Toolbox
    │   │   └── F1tenth_Map_walled.{pgm,yaml}  # Copia con una "pared virtual" (ver explicación abajo)
    │   ├── config/
    │   │   └── mapper_params_online_async.yaml  # Parámetros de SLAM Toolbox usados para el mapeo
    │   ├── package.xml
    │   └── setup.py
    └── f1tenth_pid_controller/  # Paquete ROS2 (ament_python) — Parte C
        ├── f1tenth_pid_controller/
        │   └── pid_controller_node.py   # Nodo de control PID (dirección + velocidad, vueltas y cronómetro)
        ├── launch/
        │   └── pid_control_launch.py    # Lanza el nodo con los parámetros de config/pid_params.yaml
        ├── config/
        │   └── pid_params.yaml          # Ganancias PID y parámetros de ejecución
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
| `ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs` | Usados por `sim_tf_broadcaster_node` y `pid_controller_node` (Parte C) |
| `python3-numpy`, `python3-scipy` | Dijkstra, transformada de distancia, interpolación PCHIP |
| `python3-matplotlib`, `python3-pillow`, `python3-yaml` | Solo para los scripts de `scripts/` (no se instalan con los paquetes ROS2) |
| [AutoDRIVE Simulator + `autodrive_ros2`](https://github.com/Tinker-Twins/AutoDRIVE) | Simulador y bridge ROS2 (ver instrucciones oficiales del devkit) |

Instalación rápida de dependencias de sistema:

```bash
sudo apt update
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
                 ros-humble-slam-toolbox ros-humble-rviz2 \
                 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

pip3 install numpy scipy matplotlib pillow pyyaml --user
# Si tu Ubuntu da error de "externally managed environment":
# pip3 install numpy scipy matplotlib pillow pyyaml --break-system-packages
```

## 🚀 Instalación

1. Clona este repositorio dentro del `src/` de tu workspace de ROS 2 (o usa symlinks, como
   se hizo durante el desarrollo):

   ```bash
   cd ~/autodrive_ws/src
   git clone https://github.com/shirleyandrea052004-sketch/f1tenth-dijkstra-global-planner.git
   ln -s ~/f1tenth-dijkstra-global-planner/src/f1tenth_global_planner f1tenth_global_planner
   ln -s ~/f1tenth-dijkstra-global-planner/src/f1tenth_pid_controller f1tenth_pid_controller
   ```

   *(Alternativa sin symlink: copia directamente `f1tenth-dijkstra-global-planner/src/f1tenth_global_planner`
   y `f1tenth-dijkstra-global-planner/src/f1tenth_pid_controller` a `~/autodrive_ws/src/`).*

2. Compila ambos paquetes:

   ```bash
   cd ~/autodrive_ws
   colcon build --packages-select f1tenth_global_planner f1tenth_pid_controller --symlink-install
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

### Opción B — Planificación + simulador (sin control autónomo)

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
4. (Opcional) Mueve el vehículo con teleoperación para ver el `Path` alineado con su
   posición real:
   ```bash
   ros2 run autodrive_f1tenth teleop_keyboard
   ```

En el RViz que abre `demo_launch.py`, agrega el display **TF** (By display type → TF) para ver
el frame `f1tenth_1` (posición real del vehículo en el simulador) moviéndose sobre el mapa y
el `Path` planificado.

### Opción C — Demo completa con conducción autónoma (Dijkstra + PCHIP + PID)

1. Repite los pasos 1–3 de la Opción B (simulador, bridge y `demo_launch.py`). Esto genera y
   actualiza `~/f1tenth_waypoints/smoothed_path.csv`.
2. Lanza el controlador PID:
   ```bash
   ros2 launch f1tenth_pid_controller pid_control_launch.py
   ```
   En la terminal deberías ver mensajes como:
   ```
   [pid_controller_node]: Trayectoria cargada desde CSV: 252 puntos (.../smoothed_path.csv).
   [pid_controller_node]: 🏁 Vuelta 1 completada | tiempo: 42.180 s
   [pid_controller_node]: 🏁 Vuelta 2 completada | tiempo: 39.902 s
   ```
3. (Opcional) Ajusta las ganancias PID y el perfil de velocidad en
   `src/f1tenth_pid_controller/config/pid_params.yaml` (solo relanzando el nodo, sin
   recompilar) durante el *tuning* — ver [🎛️ Sintonización](#-sintonización-tuning).

> ⚠️ El vehículo debe iniciar el modo autónomo desde su pose de spawn por defecto en
> AutoDRIVE: `start_x/start_y` en `demo_launch.py` se fijaron para coincidir con esa pose
> (ver [Parte B — Planificación global](#parte-b--planificación-global-dijkstra)), de modo
> que el primer waypoint de la ruta —y por tanto la "línea de meta" que usa el contador de
> vueltas— coincide con el punto real de arranque.

## 🧠 Algoritmos y decisiones de diseño

### Parte A — Mapeo (SLAM Toolbox)

Se siguió el tutorial oficial *["Generating a 2D Map of the F1TENTH in AutoDRIVE Using SLAM
Toolbox"](#)*, moviendo manualmente el vehículo por el circuito con teleoperación y
guardando el mapa resultante (`F1tenth_Map.pgm` / `.yaml`) con `map_saver_cli`. Los
parámetros de SLAM Toolbox usados están en `config/mapper_params_online_async.yaml`.

### Parte B — Planificación global (Dijkstra)

`global_planner_node.py`:
- Se suscribe a `/map` (QoS `TRANSIENT_LOCAL`, ya que `map_server` publica en modo *latched*).
- **Infla los obstáculos** según el radio del robot (`robot_radius`, `0.20 m` en los launch
  files) usando una transformada de distancia (`scipy.ndimage.distance_transform_edt`):
  cualquier celda libre a menos de `robot_radius` metros de un obstáculo se marca como
  ocupada. El mismo mapa de distancias se reutiliza para el centrado de carril (ver abajo).
- Convierte el grid en un grafo implícito con **conectividad-8** (permite movimientos
  diagonales, costo base `√2`) y corre **Dijkstra** (con `heapq` como cola de prioridad) desde
  el `start` hasta el `goal`.
- **Centrado de carril:** al costo base de cada movimiento se le suma una penalización
  inversamente proporcional a la distancia a la pared más cercana
  (`penalty = centering_weight / (dist_a_pared_en_celdas + 0.1)`, con `centering_weight`
  por defecto `5.0`). Sin esta penalización, Dijkstra produce el camino geométricamente más
  corto, que tiende a pegarse a los bordes internos de las curvas; con ella, la ruta se aleja
  de las paredes y se mantiene más centrada en el carril, dejando margen de maniobra para el
  controlador PID (Parte C).
- El `start`/`goal` se pueden fijar por parámetro (`start_x`, `start_y`, `goal_x`, `goal_y`)
  o sobrescribir en vivo desde RViz (`2D Pose Estimate` / `2D Nav Goal`).
- Publica la ruta resultante en `/raw_path` (`nav_msgs/Path`).

**Sobre el punto de inicio/meta:** `start_x/y` y `goal_x/y` en `demo_launch.py` y
`planning_launch.py` se ajustaron para que coincidan con la **posición y orientación reales**
del vehículo al arrancar el modo autónomo en AutoDRIVE (en vez de un punto arbitrario del
circuito). Así, el primer tramo de la ruta generada por Dijkstra —y la tangente que calcula
el suavizado PCHIP en ese punto— apuntan en la misma dirección en la que el auto ya está
orientado al spawnear, evitando un error de *heading*/*cross-track* grande apenas arranca el
control PID (Parte C). Las coordenadas se eligieron con `scripts/pick_start_goal.py` sobre el
grid inflado real (ver [🛠️ Herramientas de desarrollo](#-herramientas-de-desarrollo-scripts)).

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
   en cada punto — esta orientación es la que consume el controlador PID como referencia de
   *heading* (Parte C).

### Parte C — Control PID de seguimiento de trayectoria (`f1tenth_pid_controller`)

`pid_controller_node.py` lee la trayectoria suavizada (por defecto desde
`waypoints/smoothed_path.csv`, o en vivo desde `/smoothed_path`) y publica comandos de
dirección/aceleración para que el vehículo la recorra de forma autónoma en AutoDRIVE.

1. **Pose del vehículo:** en cada ciclo de control se consulta vía `tf2` la transformada
   `map -> f1tenth_1` publicada por `sim_tf_broadcaster_node` (paquete
   `f1tenth_global_planner`, a partir de `/autodrive/f1tenth_1/ips` e `/imu`), obteniendo
   `(x, y, yaw)`. La velocidad actual se estima por diferenciación numérica de la posición
   entre ciclos, con un promedio móvil de 5 muestras y filtros de "posición repetida" y
   "teletransporte/reset" (saltos de posición o velocidades físicamente imposibles, p. ej. un
   reset manual en Unity) para no ensuciar la estimación.
2. **Punto más cercano de la ruta:** búsqueda local con ventana deslizante alrededor del
   último índice encontrado (con caída a búsqueda completa si el resultado cae en el borde de
   la ventana), para eficiencia sobre trayectorias largas.
3. **Error lateral (cross-track) firmado**, proyectando el vector auto→punto-más-cercano sobre
   la normal a la trayectoria en ese punto:
   ```
   e_y = -sin(psi_p) * (x - x_p) + cos(psi_p) * (y - y_p)
   ```
4. **Error de orientación:** `e_psi = normalize_angle(psi_p - yaw)`.
5. **Ley de dirección (estilo Stanley):** el error de entrada al PID combina el error de
   *heading* con una corrección lateral no lineal que se atenúa a alta velocidad y se hace más
   agresiva a baja velocidad:
   ```
   e_total = e_psi + atan2(k_cte * e_y, v_estimada + speed_epsilon)
   delta   = Kp_s * e_total + Ki_s * ∫e_total dt + Kd_s * d(e_total)/dt
   ```
   `delta` se satura a `± max_steering_angle_rad`, se normaliza a `[-1, 1]` (con
   `steering_sign` configurable si el auto gira al lado incorrecto), se **atenúa** por debajo
   de `steering_activation_speed` (0.6 m/s) para evitar giros bruscos con el auto casi
   detenido, y se limita su tasa de cambio (`max_steering_rate`) antes de publicarse en
   `steering_command`. Incluye anti-windup: el término integral no se actualiza mientras la
   salida está saturada empujando en la misma dirección del error.
6. **Perfil de velocidad de referencia (curvatura):** se estima la curvatura local mirando
   `lookahead_curvature_pts` puntos hacia adelante (`kappa ≈ |Δpsi| / Δs`), y la velocidad
   objetivo se reduce en curvas cerradas y aumenta en tramos rectos:
   ```
   v_target = clip( base_speed / (1 + curvature_gain * kappa),  min_speed,  base_speed )
   ```
7. **Lazo PID de velocidad:** `e_v = v_target - v_estimada`, PID saturado a
   `[min_throttle, max_throttle]` y con límite de tasa de cambio (`max_throttle_rate`),
   publicado en `throttle_command`.
8. **Protección anti-atasco:** si la velocidad estimada permanece bajo
   `stall_speed_threshold` por más de `stall_timeout` segundos, se reinician ambos PID para
   evitar que el término integral quede "congelado" empujando en una dirección inútil.
9. **Contador de vueltas y cronómetro:** se define como "línea de meta" la posición del
   primer waypoint de la trayectoria (`path_xy[0]`) — coherente con el ajuste de `start`
   descrito en la Parte B. El nodo mantiene una máquina de estados simple: cuando el vehículo
   se aleja más de `lap_trigger_radius * lap_exit_radius_factor` de la meta se marca
   "fuera de zona de meta"; cuando regresa a menos de `lap_trigger_radius` **y** ya pasó más
   de `min_lap_time` segundos desde la última vuelta (para evitar conteos falsos por ruido),
   se incrementa el contador de vueltas y se imprime el tiempo transcurrido por terminal.

### Variables/parámetros importantes

| Nodo | Parámetro | Default | Descripción |
|---|---|---|---|
| `global_planner_node` | `robot_radius` | `0.30` (nodo) / `0.20` (launch files) | Margen de seguridad (m) para inflar obstáculos antes de correr Dijkstra |
| `global_planner_node` | `centering_weight` | `5.0` | Peso de la penalización por cercanía a paredes; favorece rutas centradas en el carril |
| `global_planner_node` | `start_x/y`, `goal_x/y` | ver `demo_launch.py` | Punto de inicio/meta, alineado con la pose real de spawn del vehículo en AutoDRIVE |
| `global_planner_node` | `use_rviz_goals` | `True` | Permite sobrescribir start/goal desde RViz |
| `global_planner_node` | `waypoints_csv_path` | `~/f1tenth_waypoints/raw_path.csv` | Ruta donde se exporta el CSV de waypoints crudos en cada ejecución |
| `path_smoother_node` | `points_per_meter` | `10.0` | Densidad de muestreo de la curva final |
| `path_smoother_node` | `simplify_epsilon` | `0.15` | Tolerancia (m) de Douglas-Peucker |
| `path_smoother_node` | `safety_radius` | `0.08` | Margen de seguridad (m) usado solo para validar el suavizado |
| `path_smoother_node` | `max_local_refinements` | `25` | Máximo de iteraciones de refinamiento local antes de usar la ruta cruda como respaldo |
| `path_smoother_node` | `waypoints_csv_path` | `~/f1tenth_waypoints/smoothed_path.csv` | Ruta donde se exporta el CSV de waypoints suavizados en cada ejecución |
| `pid_controller_node` | `kp_steer` / `ki_steer` / `kd_steer` | `0.85` / `0.02` / `0.15` | Ganancias del PID de dirección |
| `pid_controller_node` | `k_cte` | `0.8` | Fuerza de la corrección lateral (ley estilo Stanley) |
| `pid_controller_node` | `speed_epsilon` | `1.2` | Suaviza la corrección lateral a baja velocidad (evita división por ~0) |
| `pid_controller_node` | `base_speed` / `min_speed` | `2.0` / `0.4` | Rango de velocidad objetivo (m/s) |
| `pid_controller_node` | `curvature_gain` | `6.0` | Cuánto se frena en curvas cerradas |
| `pid_controller_node` | `kp_speed` / `ki_speed` / `kd_speed` | `0.60` / `0.05` / `0.05` | Ganancias del PID de velocidad |
| `pid_controller_node` | `lap_trigger_radius` / `lap_exit_radius_factor` / `min_lap_time` | `0.45` / `4.0` / `8.0` | Radios (m) y tiempo mínimo (s) para el conteo de vueltas |
| `pid_controller_node` | `path_csv` / `path_source` | `~/f1tenth_waypoints/smoothed_path.csv` / `"csv"` | Fuente de la trayectoria a seguir |

> Todos los parámetros del PID están en `src/f1tenth_pid_controller/config/pid_params.yaml`
> y son ajustables sin recompilar (solo relanzando el nodo).

## 🗺️ Waypoints generados

Los waypoints de la ruta global se exportan automáticamente a CSV cada vez que corren los
nodos (`global_planner_node` y `path_smoother_node`), y se guardan también en este
repositorio como evidencia:

| Archivo | Contenido |
|---|---|
| [`waypoints/raw_path.csv`](waypoints/raw_path.csv) | Ruta cruda de Dijkstra: `index, x, y` |
| [`waypoints/smoothed_path.csv`](waypoints/smoothed_path.csv) | Ruta suavizada (PCHIP): `index, x, y, yaw_rad` — es la que consume `pid_controller_node` por defecto |

Por defecto, al correr los nodos, los CSV se regeneran en `~/f1tenth_waypoints/` (parámetro
`waypoints_csv_path` de cada nodo). Para actualizar los archivos de este repositorio con una
nueva ejecución:

```bash
cp ~/f1tenth_waypoints/raw_path.csv waypoints/
cp ~/f1tenth_waypoints/smoothed_path.csv waypoints/
```

## 🛠️ Herramientas de desarrollo (`scripts/`)

Estos scripts corren de forma independiente (sin ROS2):

- **`visualize_map.py`**: genera una imagen del mapa con grilla de coordenadas del mundo real
  superpuesta.
  ```bash
  python3 scripts/visualize_map.py src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml
  ```

- **`pick_start_goal.py`**: carga el mapa, aplica el mismo inflado de obstáculos que usa el
  nodo de Dijkstra, y permite hacer clic directamente sobre la imagen para elegir start/goal,
  validando en tiempo real si el punto es libre. Así se eligieron las coordenadas actuales de
  `start_x/y` y `goal_x/y` (ver [Parte B](#parte-b--planificación-global-dijkstra)) para que
  coincidan con la pose real de spawn del vehículo.
  ```bash
  python3 scripts/pick_start_goal.py src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml --radius 0.20
  ```

- **`plot_waypoints.py`**: genera la imagen final de la trayectoria (mapa + waypoints crudos
  y suavizados superpuestos, ver arriba en "Demostraciones Visuales").
  ```bash
  python3 scripts/plot_waypoints.py \
      src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml \
      waypoints/raw_path.csv \
      waypoints/smoothed_path.csv \
      -o docs/media/trajectory_overlay.png
  ```

## 🎛️ Sintonización (tuning) del PID

Recomendación de orden de ajuste:

1. Fija `base_speed` bajo (p. ej. 1.0–1.5 m/s) y ajusta primero `kp_steer` hasta lograr
   seguimiento estable sin oscilar (zig-zag) en rectas.
2. Añade `kd_steer` para amortiguar oscilaciones remanentes.
3. Usa `ki_steer` pequeño solo si hay un error lateral residual constante (sesgo).
4. Ajusta `k_cte` para mejorar la corrección lateral en curvas cerradas sin sobrecorregir.
5. Una vez estable, incrementa `base_speed` y ajusta el PID de velocidad (`kp_speed`,
   `kd_speed`) para minimizar el error de velocidad sin sobrepasos agresivos.
6. Ajusta `curvature_gain` para frenar lo suficiente en curvas sin perder demasiada velocidad
   en tramos rectos.

## 📌 Notas y limitaciones conocidas

- El mapa tiene una pequeña franja de ruido de mapeo en el borde derecho del circuito
  (aprox. `x∈[1.3, 1.6]`, `y∈[-2, 2]`); no interfiere con la ruta planificada.
- La "pared virtual" es una decisión de diseño para forzar la vuelta completa al circuito;
  no representa un obstáculo real del entorno.
- El controlador PID asume que `path_xy[0]` (primer waypoint de la ruta suavizada)
  corresponde a la pose real de arranque del vehículo en modo autónomo; si se cambia el
  `start` del planificador, la "línea de meta" del contador de vueltas se mueve con él.
- Si el CSV de trayectoria no contiene la columna `yaw_rad` (por ejemplo, si se usa
  `raw_path.csv` en vez de `smoothed_path.csv`), el nodo PID calcula el yaw por diferencias
  finitas entre puntos consecutivos.
- Verifica los nombres exactos de los tópicos `steering_command` / `throttle_command` (y de
  `ips`/`imu`) con `ros2 topic list` una vez que el bridge de AutoDRIVE esté corriendo, ya que
  pueden variar según la versión del devkit; ambos son configurables en `pid_params.yaml` sin
  recompilar.

## Autor

Shirley Andrea — Proyecto individual. Algoritmos implementados: **Dijkstra** (planificación
global, Parte A) y **PID** (seguimiento de trayectoria, Parte B).
