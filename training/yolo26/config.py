"""Configuración del dataset para YOLO26."""
from pathlib import Path

# Clases en orden fijo: 0=duckie, 1=cone, 2=truck, 3=bus
CLASSES = ["duckie", "cone", "truck", "bus"]

# Tamaño de imagen: múltiplo de 32, ligero para Jetson Nano
IMG_SIZE = 416

# Directorio de dataset
DATASET_DIR = Path(__file__).resolve().parents[2] / "datasets" / "duckietown"


def dataset_yaml_dict(dataset_dir: Path = DATASET_DIR) -> dict:
    """Diccionario de dataset en formato Ultralytics.

    Args:
        dataset_dir: Ruta al directorio raíz del dataset.

    Returns:
        Dict con estructura para YOLO: path, train, val, names.
    """
    return {
        "path": str(dataset_dir),
        "train": "images/train",
        "val": "images/val",
        "names": {i: n for i, n in enumerate(CLASSES)},
    }
