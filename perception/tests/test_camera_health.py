"""Tests del chequeo de salud de frames de camara."""
import numpy as np
from object_detection.camera_health import check_frame


def test_frame_valido():
    img = np.full((480, 640, 3), 120, dtype=np.uint8)
    ok, motivo = check_frame(img)
    assert ok is True
    assert motivo == "ok"


def test_frame_none():
    ok, motivo = check_frame(None)
    assert ok is False
    assert "None" in motivo


def test_resolucion_baja():
    img = np.full((50, 50, 3), 120, dtype=np.uint8)
    ok, motivo = check_frame(img)
    assert ok is False
    assert "resolucion" in motivo


def test_frame_casi_negro():
    img = np.zeros((480, 640, 3), dtype=np.uint8)   # camara tapada
    ok, motivo = check_frame(img)
    assert ok is False
    assert "negro" in motivo
