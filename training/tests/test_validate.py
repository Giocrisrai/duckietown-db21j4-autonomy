"""Tests para validacion del modelo YOLO26."""
import sys
import types

from yolo26.validate import passes_threshold, MIN_MAP50, validate


def test_validate_extrae_map50(monkeypatch):
    """validate() debe correr YOLO.val(data,imgsz) y devolver box.map50."""
    calls = {}

    class FakeBox:
        map50 = 0.842

    class FakeMetrics:
        box = FakeBox()

    class FakeYOLO:
        def __init__(self, weights):
            calls["weights"] = weights

        def val(self, **kw):
            calls["kw"] = kw
            return FakeMetrics()

    fake = types.ModuleType("ultralytics")
    fake.YOLO = FakeYOLO
    monkeypatch.setitem(sys.modules, "ultralytics", fake)

    m = validate("best.pt", "duckietown.yaml", imgsz=416)
    assert m == 0.842
    assert calls["weights"] == "best.pt"
    assert calls["kw"]["data"] == "duckietown.yaml"
    assert calls["kw"]["imgsz"] == 416


def test_umbral_por_defecto():
    """Verifica que la constante MIN_MAP50 es 0.70."""
    assert MIN_MAP50 == 0.70


def test_pasa_umbral():
    """Verifica que un mAP50 de 0.85 supera el umbral."""
    assert passes_threshold(0.85) is True


def test_no_pasa_umbral():
    """Verifica que un mAP50 de 0.5 no supera el umbral."""
    assert passes_threshold(0.5) is False


def test_umbral_personalizado():
    """Verifica que se puede personalizar el umbral minimo."""
    assert passes_threshold(0.6, min_map50=0.55) is True
