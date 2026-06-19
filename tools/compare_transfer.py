#!/usr/bin/env python3
"""Valida el beneficio del TRANSFER LEARNING en YOLO26.

Entrena dos modelos las mismas epocas sobre el mismo dataset:
  - preentrenado:  YOLO('yolo26n.pt')   (pesos COCO -> fine-tuning)  [transfer learning]
  - desde cero:    YOLO('yolo26n.yaml')  (pesos aleatorios)
y reporta el mAP50 de cada uno. El transfer learning deberia dar mAP mas alto
(y converger antes) con las mismas epocas.

Uso:
    python tools/compare_transfer.py --data datasets/duckietown.yaml --epochs 15 --device mps
"""
import argparse


def _train_and_eval(weights, data, epochs, imgsz, device, name):
    from ultralytics import YOLO
    model = YOLO(weights)
    model.train(data=data, epochs=epochs, imgsz=imgsz, device=device,
                name=name, verbose=False, plots=False)
    metrics = model.val(data=data, imgsz=imgsz, verbose=False)
    return float(metrics.box.map50)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True)
    p.add_argument("--epochs", type=int, default=15)
    p.add_argument("--imgsz", type=int, default=416)
    p.add_argument("--device", default="mps")
    args = p.parse_args()

    pre = _train_and_eval("yolo26n.pt", args.data, args.epochs, args.imgsz,
                          args.device, "transfer_pretrained")
    scratch = _train_and_eval("yolo26n.yaml", args.data, args.epochs, args.imgsz,
                              args.device, "transfer_scratch")
    print("\n==== TRANSFER LEARNING ====")
    print(f"  preentrenado (transfer): mAP50 = {pre:.4f}")
    print(f"  desde cero (scratch):    mAP50 = {scratch:.4f}")
    mejora = (pre - scratch)
    print(f"  mejora del transfer learning: {mejora:+.4f} "
          f"({'CONFIRMADO' if mejora > 0 else 'sin ventaja'})")


if __name__ == "__main__":
    main()
