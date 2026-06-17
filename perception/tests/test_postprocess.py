import numpy as np
from object_detection.postprocess import scale_boxes, filter_detections


def test_scale_boxes_revierte_letterbox():
    """Las cajas se mapean correctamente del espacio letterbox a la imagen original."""
    # box en espacio 416 con padding dh=52 (imagen 480x640, r=0.65)
    boxes = np.array([[100.0, 152.0, 200.0, 252.0]])  # x1,y1,x2,y2
    r = 416 / 640
    out = scale_boxes(boxes, r, (0.0, 52.0), orig_shape=(480, 640))
    # y se corrige restando dh y dividiendo por r
    assert out[0, 1] == np.float64((152.0 - 52.0) / r)
    assert out[0, 0] == np.float64(100.0 / r)


def test_scale_boxes_clipa_a_la_imagen():
    """Las cajas se clipan a los limites de la imagen original."""
    boxes = np.array([[-10.0, -10.0, 9999.0, 9999.0]])
    out = scale_boxes(boxes, 1.0, (0.0, 0.0), orig_shape=(480, 640))
    assert out[0, 0] == 0.0 and out[0, 1] == 0.0
    assert out[0, 2] == 640 and out[0, 3] == 480


def test_filter_detections_por_confianza():
    """Filtra detecciones por umbral de confianza."""
    dets = np.array([
        [0, 0, 10, 10, 0.9, 0],
        [0, 0, 10, 10, 0.1, 1],
    ])
    out = filter_detections(dets, conf_thres=0.25)
    assert out.shape == (1, 6)
    assert out[0, 5] == 0
