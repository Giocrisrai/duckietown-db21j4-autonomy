from yolo26.config import CLASSES, IMG_SIZE, dataset_yaml_dict


def test_import_paquetes():
    import yolo26  # noqa: F401
    import object_detection  # noqa: F401
    import duckietown_planning  # noqa: F401


def test_clases_en_orden_fijo():
    assert CLASSES == ["duckie", "cone", "truck", "bus"]


def test_imgsz_multiplo_de_32():
    assert IMG_SIZE % 32 == 0


def test_dataset_yaml_dict():
    d = dataset_yaml_dict()
    assert d["names"] == {0: "duckie", 1: "cone", 2: "truck", 3: "bus"}
    assert d["train"] == "images/train"
    assert d["val"] == "images/val"
