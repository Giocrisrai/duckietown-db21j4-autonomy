"""Valida la estructura de un dataset YOLO de Duckietown antes de entrenar."""
from pathlib import Path


class DatasetError(Exception):
    """Error de estructura o contenido del dataset."""


def check_dataset(root, num_classes: int = 4) -> dict:
    """Valida layout images/labels (train y val) y rango de class ids.

    Verifica que existan directorios images/<split> y labels/<split> para
    cada split (train, val). Por cada archivo .txt en labels/<split>, valida
    que cada línea no vacía tenga exactamente 5 campos (class_id cx cy w h)
    y que class_id esté en el rango [0, num_classes).

    Args:
        root: Path raíz del dataset.
        num_classes: Número de clases esperadas (default 4:
                     0=duckie, 1=cone, 2=truck, 3=bus).

    Returns:
        dict: {"train": n, "val": m} conteo de archivos .txt por split.

    Raises:
        DatasetError: Si faltan directorios, líneas están malformadas,
                      o class ids están fuera de rango.
    """
    root = Path(root)
    counts = {}
    for split in ("train", "val"):
        img_dir = root / "images" / split
        lbl_dir = root / "labels" / split

        # Verificar que los directorios existan
        if not img_dir.is_dir():
            raise DatasetError(f"falta directorio {img_dir}")
        if not lbl_dir.is_dir():
            raise DatasetError(f"falta directorio {lbl_dir}")

        # Procesar archivos de etiquetas
        labels = sorted(lbl_dir.glob("*.txt"))
        for lbl in labels:
            for i, line in enumerate(lbl.read_text().splitlines(), 1):
                line = line.strip()
                # Ignorar líneas vacías
                if not line:
                    continue
                # Validar exactamente 5 campos
                parts = line.split()
                if len(parts) != 5:
                    raise DatasetError(
                        f"{lbl}:{i} debe tener 5 campos (cls cx cy w h)"
                    )
                # Validar que class id esté en rango
                try:
                    cls = int(float(parts[0]))
                except ValueError:
                    raise DatasetError(
                        f"{lbl}:{i} class id '{parts[0]}' no es un número válido"
                    )
                if not (0 <= cls < num_classes):
                    raise DatasetError(
                        f"{lbl}:{i} class id {cls} fuera de [0,{num_classes})"
                    )

        counts[split] = len(labels)
    return counts
