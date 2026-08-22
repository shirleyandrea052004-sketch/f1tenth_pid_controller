"""
Genera la imagen final de la trayectoria: mapa + waypoints crudos (Dijkstra)
+ waypoints suavizados (PCHIP), para entregar como evidencia del proyecto.

Uso:
    python3 plot_waypoints.py \
        ../src/f1tenth_global_planner/maps/F1tenth_Map_walled.yaml \
        ~/f1tenth_waypoints/raw_path.csv \
        ~/f1tenth_waypoints/smoothed_path.csv \
        -o trajectory_overlay.png
"""
import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = os.path.join(yaml_dir, meta['image'])
    img = np.array(Image.open(image_path).convert('L'))
    return img, meta['resolution'], meta['origin']


def load_waypoints(csv_path):
    xs, ys = [], []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return np.array(xs), np.array(ys)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml')
    parser.add_argument('raw_csv')
    parser.add_argument('smoothed_csv')
    parser.add_argument('-o', '--output', default='trajectory_overlay.png')
    args = parser.parse_args()

    pgm_array, resolution, origin = load_map(args.map_yaml)
    ros_array = np.flipud(pgm_array)  # fila 0 = y mínima, convención ROS

    height, width = ros_array.shape
    x_min, y_min = origin[0], origin[1]
    x_max = x_min + width * resolution
    y_max = y_min + height * resolution

    xs_raw, ys_raw = load_waypoints(args.raw_csv)
    xs_smooth, ys_smooth = load_waypoints(args.smoothed_csv)

    fig, ax = plt.subplots(figsize=(6, 16))
    ax.imshow(ros_array, cmap='gray', origin='lower',
              extent=[x_min, x_max, y_min, y_max])
    ax.plot(xs_raw, ys_raw, '-', color='red', linewidth=1.2,
            label=f'Ruta cruda - Dijkstra ({len(xs_raw)} pts)')
    ax.plot(xs_smooth, ys_smooth, '-', color='limegreen', linewidth=1.8,
            label=f'Ruta suavizada - PCHIP ({len(xs_smooth)} pts)')
    ax.plot(xs_raw[0], ys_raw[0], 'o', color='blue', markersize=8, label='Start')
    ax.plot(xs_raw[-1], ys_raw[-1], 'o', color='gold', markersize=8, label='Goal')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trayectoria global: Dijkstra + suavizado PCHIP')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f'Imagen guardada en: {args.output}')


if __name__ == '__main__':
    main()