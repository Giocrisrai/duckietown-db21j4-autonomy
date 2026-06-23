#!/usr/bin/env python3
"""Genera un dataset sintetico estilo Duckietown para validar el pipeline YOLO26.

Las 4 clases (duckie, cone, truck, bus) se dibujan como formas de colores sobre
un fondo tipo pista. NO sustituye al dataset real de competencia; sirve para
probar y validar la cadena entrenar -> validar -> exportar de punta a punta.

Uso:
    python tools/make_synthetic_dataset.py --root datasets/duckietown --train 120 --val 30
Genera:
    datasets/duckietown/images/{train,val}/*.jpg
    datasets/duckietown/labels/{train,val}/*.txt   (formato YOLO: cls cx cy w h)
    datasets/duckietown.yaml
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

CLASSES = ["duckie", "cone", "truck", "bus"]
# Colores BGR distintivos por clase
COLORS = {
    0: (0, 215, 255),    # duckie  - amarillo
    1: (0, 140, 255),    # cone    - naranja
    2: (40, 40, 200),    # truck   - rojo
    3: (200, 90, 0),     # bus     - azul
}
IMG = 416


def _rng(seed):
    """Generador determinista (evita estado global)."""
    return np.random.default_rng(seed)


def _draw_object(img, cls, cx, cy, w, h):
    """Dibuja la forma de la clase y devuelve la caja en pixeles (x1,y1,x2,y2)."""
    x1, y1 = int(cx - w / 2), int(cy - h / 2)
    x2, y2 = int(cx + w / 2), int(cy + h / 2)
    color = COLORS[cls]
    if cls == 0:        # duckie: circulo
        cv2.circle(img, (int(cx), int(cy)), int(min(w, h) / 2), color, -1)
    elif cls == 1:      # cone: triangulo
        pts = np.array([[cx, y1], [x1, y2], [x2, y2]], dtype=np.int32)
        cv2.fillPoly(img, [pts], color)
    else:               # truck/bus: rectangulo
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
    return x1, y1, x2, y2


def _make_image(rng, n_max=3):
    """Crea una imagen con 1..n_max objetos. Devuelve (img, labels)."""
    # fondo tipo pista: gris con una franja amarilla central tenue
    img = np.full((IMG, IMG, 3), 90, dtype=np.uint8)
    cv2.line(img, (IMG // 2, 0), (IMG // 2, IMG), (60, 180, 200), 6)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    labels = []
    n = int(rng.integers(1, n_max + 1))
    for _ in range(n):
        cls = int(rng.integers(0, len(CLASSES)))
        w = float(rng.integers(40, 110))
        h = float(rng.integers(40, 110))
        cx = float(rng.integers(int(w / 2) + 2, IMG - int(w / 2) - 2))
        cy = float(rng.integers(int(h / 2) + 2, IMG - int(h / 2) - 2))
        _draw_object(img, cls, cx, cy, w, h)
        labels.append((cls, cx / IMG, cy / IMG, w / IMG, h / IMG))
    return img, labels


def _write_split(root: Path, split: str, count: int, seed: int):
    img_dir = root / "images" / split
    lbl_dir = root / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    rng = _rng(seed)
    for i in range(count):
        img, labels = _make_image(rng)
        cv2.imwrite(str(img_dir / f"{split}_{i:04d}.jpg"), img)
        lines = [f"{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}" for c, cx, cy, w, h in labels]
        (lbl_dir / f"{split}_{i:04d}.txt").write_text("\n".join(lines) + "\n")
    return count


def main():
    p = argparse.ArgumentParser(description="Dataset sintetico Duckietown para YOLO26")
    p.add_argument("--root", default="datasets/duckietown")
    p.add_argument("--train", type=int, default=120)
    p.add_argument("--val", type=int, default=30)
    p.add_argument("--seed", type=int, default=7)
    args = p.parse_args()

    root = Path(args.root)
    n_tr = _write_split(root, "train", args.train, args.seed)
    n_va = _write_split(root, "val", args.val, args.seed + 1000)

    yaml_path = root.parent / "duckietown.yaml"
    data = {
        "path": str(root.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {i: n for i, n in enumerate(CLASSES)},
    }
    yaml_path.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"OK: {n_tr} train + {n_va} val en {root}; config en {yaml_path}")


if __name__ == "__main__":
    main()
