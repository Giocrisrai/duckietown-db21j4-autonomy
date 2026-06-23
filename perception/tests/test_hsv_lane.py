"""Tests de la deteccion de linea amarilla por HSV (con imagenes sinteticas).

Patron portado (adaptado) del repo del colega Luissalamanca23/
Duckietown_Follow_Line_Pi (robot/vision.py). Lane following clasico en CPU, sin
depender de dt-core.
"""
import numpy as np
import cv2
from object_detection.hsv_lane import detect_yellow_line, YELLOW_LOWER, YELLOW_UPPER


def _frame_con_amarillo(x, y, w, h, size=(480, 640)):
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), -1)  # amarillo BGR
    return img


def test_detecta_rectangulo_amarillo():
    img = _frame_con_amarillo(300, 300, 40, 80)        # en el tercio inferior
    det = detect_yellow_line(img, YELLOW_LOWER, YELLOW_UPPER)
    assert det.found is True
    assert det.x == 300                                 # bbox en coords del crop
    assert 35 <= det.w <= 45
    assert det.yellow_px > 0
    assert det.frame_w == 640


def test_sin_amarillo_no_encuentra():
    img = np.zeros((480, 640, 3), dtype=np.uint8)       # todo negro
    det = detect_yellow_line(img, YELLOW_LOWER, YELLOW_UPPER)
    assert det.found is False
    assert det.yellow_px == 0


def test_ignora_franja_muy_cercana_usa_segunda():
    """Si el contorno mayor esta demasiado abajo (y > y_too_close), usa el 2do."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    # crop_top = 160; franja grande MUY abajo (y_crop ~ 290 > 200) y franja lejana arriba
    cv2.rectangle(img, (100, 450), (250, 478), (0, 255, 255), -1)   # grande, muy abajo
    cv2.rectangle(img, (400, 180), (440, 230), (0, 255, 255), -1)   # mas chica, lejana
    det = detect_yellow_line(img, YELLOW_LOWER, YELLOW_UPPER, y_too_close=200)
    assert det.found is True
    assert det.x >= 390          # eligio la franja lejana (derecha), no la cercana
