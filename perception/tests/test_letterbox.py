"""Tests para letterbox preprocessing."""
import pytest
from object_detection.letterbox import compute_letterbox


def test_imagen_cuadrada_sin_padding():
    """Imagen cuadrada 416x416 escalada a 416 debe ser identidad."""
    r, new_unpad, (dw, dh) = compute_letterbox(416, 416, 416)
    assert r == 1.0
    assert new_unpad == (416, 416)
    assert (dw, dh) == (0.0, 0.0)


def test_imagen_horizontal_escala_por_ancho():
    """Imagen 480x640 -> escala por el lado mas grande (640) a 416."""
    r, new_unpad, (dw, dh) = compute_letterbox(480, 640, 416)
    assert r == pytest.approx(416/640)
    assert new_unpad == (416, 312)   # (w, h) escalados
    assert dh == pytest.approx((416 - 312) / 2)
    assert dw == pytest.approx(0.0)
