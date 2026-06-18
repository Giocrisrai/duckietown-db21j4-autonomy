"""CLI para crear un modelo ONNX sintetico con detecciones fijas."""
import argparse
import numpy as np
from simulation.sim_perception import make_dummy_onnx


def main():
    """Crea un modelo ONNX dummy con detecciones especificadas."""
    parser = argparse.ArgumentParser(
        description="Genera un modelo ONNX sintetico que emite detecciones fijas"
    )
    parser.add_argument(
        "output_path",
        type=str,
        help="Ruta donde guardar el modelo ONNX (ej: /tmp/dummy.onnx)"
    )
    parser.add_argument(
        "--x1",
        type=float,
        default=100.0,
        help="Coordenada x1 de la caja (default: 100.0)"
    )
    parser.add_argument(
        "--y1",
        type=float,
        default=100.0,
        help="Coordenada y1 de la caja (default: 100.0)"
    )
    parser.add_argument(
        "--x2",
        type=float,
        default=200.0,
        help="Coordenada x2 de la caja (default: 200.0)"
    )
    parser.add_argument(
        "--y2",
        type=float,
        default=200.0,
        help="Coordenada y2 de la caja (default: 200.0)"
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.9,
        help="Confianza (score) de la deteccion (default: 0.9)"
    )
    parser.add_argument(
        "--class_id",
        type=float,
        default=0.0,
        help="ID de clase (default: 0.0 para duckie)"
    )

    args = parser.parse_args()

    # Crea detecciones con un unico objeto
    dets = np.array([[[args.x1, args.y1, args.x2, args.y2, args.conf, args.class_id]]], dtype=np.float32)

    path = make_dummy_onnx(args.output_path, dets)
    print(f"Modelo ONNX creado en: {path}")


if __name__ == "__main__":
    main()
