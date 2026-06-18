import pytest
from yolo26.dataset_check import check_dataset, DatasetError


def _crear_dataset(root, etiqueta="0 0.5 0.5 0.2 0.2"):
    """Crea un dataset YOLO mínimo con train/val splits."""
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True)
        (root / "labels" / split).mkdir(parents=True)
        (root / "labels" / split / "img1.txt").write_text(etiqueta + "\n")


def test_dataset_valido(tmp_path):
    """Dataset con estructura correcta debe pasar validación."""
    _crear_dataset(tmp_path)
    counts = check_dataset(tmp_path)
    assert counts == {"train": 1, "val": 1}


def test_falta_directorio(tmp_path):
    """Debe lanzar DatasetError si falta un directorio requerido."""
    (tmp_path / "images" / "train").mkdir(parents=True)
    with pytest.raises(DatasetError):
        check_dataset(tmp_path)


def test_class_id_fuera_de_rango(tmp_path):
    """Debe lanzar DatasetError si class id está fuera de [0, num_classes)."""
    _crear_dataset(tmp_path, etiqueta="9 0.5 0.5 0.2 0.2")
    with pytest.raises(DatasetError):
        check_dataset(tmp_path)


def test_linea_malformada(tmp_path):
    """Debe lanzar DatasetError si una línea no tiene 5 campos."""
    _crear_dataset(tmp_path, etiqueta="0 0.5 0.5")
    with pytest.raises(DatasetError):
        check_dataset(tmp_path)
