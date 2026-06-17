"""Postproceso de la salida YOLO26 (end-to-end, NMS-free).

La salida del engine TensorRT son cajas ya decodificadas en el espacio
letterbox 416: (N, 6) = [x1, y1, x2, y2, score, cls]. Aqui solo revertimos
el letterbox a coordenadas de la imagen original y filtramos por confianza.
"""
import numpy as np


def scale_boxes(boxes_xyxy, r, pad, orig_shape):
    """Mapea cajas del espacio letterbox a la imagen original.

    Revierte el preprocessing letterbox:
    - Resta el padding (dw, dh) aplicado al colocar la imagen en el espacio 416.
    - Divide por el factor de escala r para volver a coordenadas de la imagen original.
    - Clipea las coordenadas a los limites (0, w) y (0, h).

    Parametros:
        boxes_xyxy: np.ndarray de shape (N, 4+) con [x1, y1, x2, y2, ...].
        r: float, factor de escala aplicado en el letterbox.
        pad: tuple (dw, dh), padding aplicado.
        orig_shape: tuple (h, w), tamaño de la imagen original.

    Devuelve:
        np.ndarray de shape igual a boxes_xyxy, con las cajas mapeadas.
    """
    dw, dh = pad
    out = np.asarray(boxes_xyxy, dtype=np.float64).copy()
    # Restar padding a x1, x2
    out[:, [0, 2]] -= dw
    # Restar padding a y1, y2
    out[:, [1, 3]] -= dh
    # Dividir por factor de escala
    out[:, :4] /= r
    # Clipear a los limites de la imagen original
    h, w = orig_shape
    out[:, [0, 2]] = out[:, [0, 2]].clip(0, w)
    out[:, [1, 3]] = out[:, [1, 3]].clip(0, h)
    return out


def filter_detections(dets, conf_thres: float = 0.25):
    """Filtra detecciones (N,6) por score >= conf_thres.

    Parametros:
        dets: np.ndarray de shape (N, 6) con [x1, y1, x2, y2, score, cls].
        conf_thres: float, umbral minimo de confianza.

    Devuelve:
        np.ndarray con las detecciones que superan el umbral, shape (M, 6).
        Si no hay detecciones, devuelve shape (0, 6).
    """
    dets = np.asarray(dets, dtype=np.float64)
    if dets.size == 0:
        return dets.reshape(0, 6)
    return dets[dets[:, 4] >= conf_thres]
