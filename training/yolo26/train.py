"""Entrena YOLO26n con el dataset oficial de Duckietown.

Corre identico en Colab (CUDA) y en MacBook M1 (MPS):
    python -m yolo26.train --data datasets/duckietown.yaml --epochs 100

Pasos previos (documentados, no automatizados aqui):
  1. Colocar imagenes/labels YOLO en datasets/duckietown/{images,labels}/{train,val}
  2. Generar el yaml:  python -c "from yolo26.prepare_dataset import write_dataset_yaml; \
     write_dataset_yaml('datasets/duckietown.yaml')"
"""
import argparse
from .config import IMG_SIZE
from .device import select_device


def main():
    p = argparse.ArgumentParser(description="Entrena YOLO26n para Duckietown")
    p.add_argument("--data", required=True, help="ruta a duckietown.yaml")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--imgsz", type=int, default=IMG_SIZE)
    p.add_argument("--device", default="auto")
    p.add_argument("--model", default="yolo26n.pt")
    # Rendimiento: cache acelera mucho las epocas (lee del RAM/disco una vez);
    # patience habilita early-stopping; workers ajusta el dataloader.
    p.add_argument("--cache", default="ram",
                   help="cache de imagenes: 'ram' | 'disk' | 'false'")
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--patience", type=int, default=20,
                   help="early-stopping: epocas sin mejora antes de parar")
    args = p.parse_args()

    device = select_device(args.device)
    cache = args.cache if args.cache.lower() != "false" else False
    print(f"==> Entrenando en device={device} (cache={cache}, workers={args.workers})")
    from ultralytics import YOLO

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=device,
        cache=cache,
        workers=args.workers,
        patience=args.patience,
    )
    print("OK: entrenamiento terminado. Pesos en runs/detect/train/weights/best.pt")


if __name__ == "__main__":
    main()
