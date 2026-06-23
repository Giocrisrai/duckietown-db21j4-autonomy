"""Tests de la logica pura del grabador de dataset (sin camara)."""
import pytest
from object_detection.dataset_recorder import (
    frame_filename,
    capture_plan,
    manifest,
)


def test_frame_filename_zero_padded():
    assert frame_filename("frame", 0) == "frame_00000.jpg"
    assert frame_filename("frame", 42) == "frame_00042.jpg"
    assert frame_filename("img", 7, ext="png") == "img_00007.png"


def test_capture_plan_valido():
    p = capture_plan(count=10, interval_s=0.5)
    assert p["count"] == 10
    assert p["interval_s"] == 0.5
    assert p["duracion_estimada_s"] == pytest.approx(4.5)  # (10-1)*0.5


def test_capture_plan_un_frame_sin_duracion():
    assert capture_plan(1, 1.0)["duracion_estimada_s"] == 0.0


def test_capture_plan_rechaza_invalidos():
    with pytest.raises(ValueError):
        capture_plan(0, 1.0)        # count debe ser >= 1
    with pytest.raises(ValueError):
        capture_plan(5, -0.1)       # intervalo no negativo


def test_manifest_estructura():
    m = manifest(out_dir="/data/ds", frames=["frame_00000.jpg", "frame_00001.jpg"],
                 model="imx219", width=1280, height=720)
    assert m["model"] == "imx219"
    assert m["width"] == 1280 and m["height"] == 720
    assert m["count"] == 2
    assert m["frames"] == ["frame_00000.jpg", "frame_00001.jpg"]
    assert m["out_dir"] == "/data/ds"
