import sys
import types

import pytest
from yolo26.export_onnx import build_export_kwargs, export


def test_export_invoca_yolo_con_kwargs(monkeypatch):
    """export() carga los pesos y exporta a ONNX con los kwargs correctos."""
    calls = {}

    class FakeYOLO:
        def __init__(self, weights):
            calls["weights"] = weights

        def export(self, **kw):
            calls["kw"] = kw
            return "/tmp/best.onnx"

    fake = types.ModuleType("ultralytics")
    fake.YOLO = FakeYOLO
    monkeypatch.setitem(sys.modules, "ultralytics", fake)

    out = export("best.pt", imgsz=416, opset=12)
    assert out == "/tmp/best.onnx"
    assert calls["weights"] == "best.pt"
    assert calls["kw"]["format"] == "onnx"
    assert calls["kw"]["opset"] == 12
    assert calls["kw"]["imgsz"] == 416


def test_kwargs_por_defecto():
    k = build_export_kwargs()
    assert k["format"] == "onnx"
    assert k["imgsz"] == 416
    assert k["opset"] == 12
    assert k["simplify"] is True
    assert k["dynamic"] is False


def test_imgsz_debe_ser_multiplo_de_32():
    with pytest.raises(AssertionError):
        build_export_kwargs(imgsz=400)


def test_opset_fuera_de_rango_falla():
    with pytest.raises(AssertionError):
        build_export_kwargs(opset=20)
