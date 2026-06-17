"""Prepara el dataset oficial de Duckietown para YOLO26.

El dataset oficial (clases duckie/cone/truck/bus) se descarga desde el
ejercicio object-detection del curso (duckietown-lx, branch mooc2022) o
desde Roboflow. Este modulo solo genera el archivo de configuracion que
Ultralytics necesita; la descarga/colocacion de imagenes se documenta en
el docstring de train.py.
"""
from pathlib import Path
import yaml
from .config import dataset_yaml_dict, DATASET_DIR


def write_dataset_yaml(out_path, dataset_dir: Path = DATASET_DIR) -> Path:
    """Escribe el archivo duckietown.yaml para Ultralytics.

    Args:
        out_path: Ruta donde escribir el archivo yaml.
        dataset_dir: Directorio raíz del dataset (por defecto DATASET_DIR).

    Returns:
        Path al archivo creado.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        yaml.safe_dump(dataset_yaml_dict(Path(dataset_dir)), f, sort_keys=False)
    return out_path
