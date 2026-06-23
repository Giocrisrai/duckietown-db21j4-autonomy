#!/usr/bin/env python3
"""Graba un dataset de frames de la camara del robot (para entrenar/calibrar).

Reusa la autodeteccion + pipeline de `camera_support` y el chequeo de salud de
`camera_health`. Guarda N frames + un manifest.json. Corre EN EL ROBOT.

Uso (en el robot):
    python3 tools/record_dataset.py --out /data/datasets/run1 --count 50 --interval 0.3
    python3 tools/record_dataset.py --model imx219 --count 100      # fuerza el sensor
"""
import argparse
import glob
import json
import sys
import time
from pathlib import Path

P = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(P / "perception/packages/object_detection/src/object_detection"))
import camera_support as cs                       # noqa: E402
from camera_health import check_frame             # noqa: E402
from dataset_recorder import frame_filename, capture_plan, manifest  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/dataset")
    ap.add_argument("--count", type=int, default=50)
    ap.add_argument("--interval", type=float, default=0.3, help="segundos entre frames")
    ap.add_argument("--model", default=None, help="imx219|usb (por defecto autodetecta)")
    ap.add_argument("--prefix", default="frame")
    args = ap.parse_args(argv)

    plan = capture_plan(args.count, args.interval)   # valida antes de tocar hardware
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    import cv2
    model = args.model
    if model is None:
        try:
            dmesg = Path("/var/log/dmesg").read_text()
        except Exception:
            dmesg = ""
        model = cs.detect_camera(dmesg=dmesg, devices=glob.glob("/dev/video*")) or "usb"
    print(f"Camara: {model} | plan: {plan['count']} frames cada {plan['interval_s']}s "
          f"(~{plan['duracion_estimada_s']:.1f}s) -> {out}")

    pipe = cs.gst_pipeline(model)
    cap = (cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
           if cs.camera_info(model)["tipo"] != "v4l2" else cv2.VideoCapture(0))
    frames, sanos, w, h = [], 0, 0, 0
    try:
        for i in range(plan["count"]):
            ok, frame = cap.read()
            if not ok or frame is None:
                print(f"  [{i}] sin frame, reintentando...")
                time.sleep(args.interval)
                continue
            h, w = frame.shape[:2]
            sano, _ = check_frame(frame)
            sanos += int(sano)
            name = frame_filename(args.prefix, i)
            cv2.imwrite(str(out / name), frame)
            frames.append(name)
            if args.interval:
                time.sleep(args.interval)
    finally:
        cap.release()

    man = manifest(str(out), frames, model, w, h)
    man["sanos"] = sanos
    (out / "manifest.json").write_text(json.dumps(man, indent=2))
    print(f"OK: {len(frames)} frames ({sanos} sanos) en {out} (+ manifest.json)")
    return 0 if frames else 1


if __name__ == "__main__":
    sys.exit(main())
