"""
Genera una imagen del mapa F1TENTH con una grilla de coordenadas del
mundo real superpuesta, para elegir visualmente puntos de start/goal
en zona libre.

Uso:
    python3 visualize_map.py /ruta/a/F1tenth_Map.yaml
"""
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    import os
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_path = os.path.join(yaml_dir, meta['image'])

    img = Image.open(image_path)
    grid = np.array(img)

    resolution = meta['resolution']
    origin = meta['origin']  # [x, y, theta]
    negate = meta.get('negate', 0)

    return grid, resolution, origin, negate


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 visualize_map.py /ruta/a/F1tenth_Map.yaml")
        sys.exit(1)

    yaml_path = sys.argv[1]
    grid, resolution, origin, negate = load_map(yaml_path)

    height, width = grid.shape[:2]
    origin_x, origin_y = origin[0], origin[1]

    # Extensión en coordenadas del mundo real
    # OJO: en un PGM, la fila 0 es la parte SUPERIOR de la imagen,
    # pero corresponde a la Y MÁXIMA del mapa (el mapa está "volteado").
    x_min = origin_x
    x_max = origin_x + width * resolution
    y_min = origin_y
    y_max = origin_y + height * resolution

    fig, ax = plt.subplots(figsize=(8, 16))

    # extent=[x_min, x_max, y_min, y_max] y origin='lower' hace que
    # matplotlib interprete correctamente la orientación del mapa.
    cmap = 'gray' if not negate else 'gray_r'
    ax.imshow(grid, cmap=cmap, origin='lower',
              extent=[x_min, x_max, y_min, y_max])

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('F1TENTH Map - elige start/goal en zona libre (blanco)')
    ax.grid(True, color='cyan', alpha=0.4, linewidth=0.5)

    # Grilla más marcada cada 1 metro
    ax.set_xticks(np.arange(np.floor(x_min), np.ceil(x_max) + 1, 1.0))
    ax.set_yticks(np.arange(np.floor(y_min), np.ceil(y_max) + 1, 1.0))
    ax.tick_params(axis='both', labelsize=7)

    out_path = 'map_with_grid.png'
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Imagen guardada en: {out_path}")
    print(f"Rango X: [{x_min:.2f}, {x_max:.2f}]  Rango Y: [{y_min:.2f}, {y_max:.2f}]")


if __name__ == '__main__':
    main()