import pytest
from yolo26.export_onnx import build_export_kwargs


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
