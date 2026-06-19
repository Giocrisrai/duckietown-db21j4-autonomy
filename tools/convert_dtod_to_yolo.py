#!/usr/bin/env python3
"""Convierte el dataset OFICIAL de object detection de Duckietown a formato YOLO.

Fuente: duckietown_object_detection_dataset (frames/*.png + annotation/final_anns.json),
descargable de:
  https://duckietown-public-storage.s3.amazonaws.com/assets/mooc/2022/duckietown_object_detection_dataset.zip

El JSON mapea frame -> [{cat_name, bbox:[x,y,w,h] en pixeles top-left}].
Las clases reales son duckie/cone/duckiebot (640x480). Las mapeamos a las ranuras
del proyecto (0=duckie,1=cone,2=truck,3=bus): duckiebot ocupa la ranura "truck"
(= otro vehiculo); "bus" queda sin ejemplos (objeto solo de simulador).

Uso:
    python tools/convert_dtod_to_yolo.py --src /tmp/dtod/duckietown_object_detection_dataset \
        --dst datasets/duckietown_real --val-frac 0.15
"""
import argparse
import json
from pathlib import Path

import cv2
import yaml

# cat_name del dataset -> id de clase del proyecto (orden fijo duckie,cone,truck,bus)
NAME_TO_ID = {"duckie": 0, "cone": 1, "duckiebot": 2}
CLASSES = ["duckie", "cone", "truck", "bus"]
W, H = 640, 480


def _to_yolo(bbox):
    """bbox [x,y,w,h] top-left en pixeles -> (cx,cy,w,h) normalizado, recortado a [0,1]."""
    x, y, w, h = bbox
    cx = (x + w / 2) / W
    cy = (y + h / 2) / H
    nw = w / W
    nh = h / H
    clamp = lambda v: max(0.0, min(1.0, v))
    return clamp(cx), clamp(cy), clamp(nw), clamp(nh)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--src", required=True, help="raiz del dataset extraido")
    p.add_argument("--dst", default="datasets/duckietown_real")
    p.add_argument("--val-frac", type=float, default=0.15)
    args = p.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    anns = json.loads((src / "annotation" / "final_anns.json").read_text())
    frames = sorted(anns.keys())                      # orden determinista
    n_val = int(len(frames) * args.val_frac)
    splits = {"val": frames[:n_val], "train": frames[n_val:]}

    counts = {"train": 0, "val": 0}
    for split, names in splits.items():
        (dst / "images" / split).mkdir(parents=True, exist_ok=True)
        (dst / "labels" / split).mkdir(parents=True, exist_ok=True)
        for name in names:
            img_path = src / "frames" / name
            if not img_path.exists():
                continue
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            stem = Path(name).stem
            cv2.imwrite(str(dst / "images" / split / f"{stem}.jpg"), img)
            lines = []
            for o in anns[name]:
                cid = NAME_TO_ID.get(o.get("cat_name"))
                if cid is None:
                    continue
                cx, cy, nw, nh = _to_yolo(o["bbox"])
                if nw <= 0 or nh <= 0:
                    continue
                lines.append(f"{cid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}")
            (dst / "labels" / split / f"{stem}.txt").write_text("\n".join(lines) + "\n")
            counts[split] += 1

    yaml_path = dst.parent / "duckietown_real.yaml"
    yaml_path.write_text(yaml.safe_dump({
        "path": str(dst.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {i: n for i, n in enumerate(CLASSES)},
    }, sort_keys=False))
    print(f"OK: {counts['train']} train + {counts['val']} val en {dst}; config {yaml_path}")
    print("Clases reales presentes: duckie(0), cone(1), duckiebot->truck(2). bus(3) sin ejemplos.")


if __name__ == "__main__":
    main()
