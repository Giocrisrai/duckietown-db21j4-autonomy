#!/usr/bin/env python3
"""Dry-run del seguidor de linea: corre el lazo percepcion->duty sobre UNA imagen,
SIN mover motores. Equivale al `main.py --dry-run` del repo Pi del colega.

Sirve para: (1) validar la cadena hsv_lane -> cross_track_error -> LanePID ->
vw_to_duty sobre frames reales del robot, y (2) afinar el HSV (ver yellow_px /
bbox) antes de cablear motores.

Uso:
    python3 tools/lane_follow_dryrun.py /tmp/frame.jpg
    make dry-run                      # captura un frame del robot y lo corre
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "perception/packages/object_detection/src"))
sys.path.insert(0, str(ROOT / "control"))


def main(path):
    import cv2
    from object_detection.hsv_lane import detect_yellow_line, YELLOW_LOWER, YELLOW_UPPER
    from lane_follow_loop import LaneFollowLoop

    img = cv2.imread(path)
    if img is None:
        print(f"  no pude leer la imagen: {path}")
        return 1
    det = detect_yellow_line(img, YELLOW_LOWER, YELLOW_UPPER)
    loop = LaneFollowLoop(v_nominal=0.2)
    ul, ur = loop.step(det, dt=0.033)
    print(f"frame {img.shape[1]}x{img.shape[0]} | linea: found={det.found} "
          f"bbox=({det.x},{det.y},{det.w},{det.h}) yellow_px={det.yellow_px}")
    print(f"lazo -> duty: u_left={ul:+.3f} u_right={ur:+.3f} "
          f"({'sigue linea' if det.found else 'sin linea -> STOP (fail-safe)'})")
    print("  (dry-run: NO se movio ningun motor)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/dry.jpg"))
