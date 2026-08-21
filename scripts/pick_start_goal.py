"""
Elige visualmente start y goal haciendo clic sobre el mapa YA INFLADO
(el mismo que usa el nodo Dijkstra). Imprime las coordenadas exactas
y valida que ambos puntos estén en zona libre.

Uso:
    python3 pick_start_goal.py maps/F1tenth_Map_walled.yaml --radius 0.20

Instrucciones en pantalla:
    1. Haz clic UNA vez para el START (se marca en verde)
    2. Haz clic UNA vez para el GOAL (se marca en rojo)
    3. Cierra la ventana - las coordenadas se imprimen en la terminal
"""
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


def load_map_as_occupancy(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = os.path.join(yaml_dir, meta['image'])

    img = Image.open(image_path).convert('L')  # escala de grises
    pgm_array = np.array(img)  # fila 0 = TOP de la imagen (convención PGM)

    # map_server voltea verticalmente al cargar, para que fila 0 del
    # OccupancyGrid = y MÍNIMA (origin). Replicamos eso aquí.
    ros_array = np.flipud(pgm_array)

    # Umbral simple: negro=ocupado(100), blanco=libre(0), gris=desconocido(-1)
    grid = np.full(ros_array.shape, -1, dtype=np.int8)
    grid[ros_array < 50] = 100
    grid[ros_array > 200] = 0

    resolution = meta['resolution']
    origin = meta['origin']
    return grid, resolution, origin


def inflate(grid, resolution, robot_radius):
    occupied = (grid == 100) | (grid == -1)
    free_mask = ~occupied
    dist = distance_transform_edt(free_mask)
    radius_cells = robot_radius / resolution
    inflated = grid.copy()
    inflated[dist <= radius_cells] = 100
    return inflated


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml_path')
    parser.add_argument('--radius', type=float, default=0.20)
    args = parser.parse_args()

    grid, resolution, origin = load_map_as_occupancy(args.yaml_path)
    inflated = inflate(grid, resolution, args.radius)

    height, width = grid.shape
    x_min, y_min = origin[0], origin[1]
    x_max = x_min + width * resolution
    y_max = y_min + height * resolution

    fig, ax = plt.subplots(figsize=(6, 16))
    # Aquí origin='lower' SÍ es correcto: ya volteamos el array arriba,
    # así que la fila 0 realmente corresponde a y_min.
    ax.imshow(inflated, cmap='gray_r', origin='lower',
              extent=[x_min, x_max, y_min, y_max], vmin=-1, vmax=100)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Clic 1 = START (verde) | Clic 2 = GOAL (rojo) | radius={args.radius}m')
    ax.grid(True, alpha=0.3)
    ax.set_xticks(np.arange(np.floor(x_min), np.ceil(x_max) + 1, 0.5))
    ax.set_yticks(np.arange(np.floor(y_min), np.ceil(y_max) + 1, 0.5))
    ax.tick_params(axis='both', labelsize=6)
    plt.tight_layout()

    print("Haz clic en la figura: primero START, luego GOAL...")
    pts = plt.ginput(2, timeout=0)

    def world_to_grid(x, y):
        gx = int((x - x_min) / resolution)
        gy = int((y - y_min) / resolution)
        return gx, gy

    def is_free(x, y):
        gx, gy = world_to_grid(x, y)
        if not (0 <= gx < width and 0 <= gy < height):
            return False, None
        return inflated[gy, gx] < 50, inflated[gy, gx]

    labels = ['START', 'GOAL']
    colors = ['green', 'red']
    print("\n--- Resultado ---")
    for (x, y), label, color in zip(pts, labels, colors):
        free, val = is_free(x, y)
        status = "LIBRE ✔️" if free else f"OCUPADO/INFLADO ❌ (valor={val})"
        print(f"{label}: x={x:.3f}, y={y:.3f}  ->  {status}")
        ax.plot(x, y, 'o', color=color, markersize=10)

    plt.savefig('pick_result.png', dpi=150)
    print("\nImagen guardada como pick_result.png con los puntos marcados.")
    plt.show()


if __name__ == '__main__':
    main()
