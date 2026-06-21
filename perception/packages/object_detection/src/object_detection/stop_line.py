"""Deteccion de la LINEA ROJA de stop (paridad con dt-core stop_line_filter).

En Duckietown las intersecciones se delimitan por una franja ROJA perpendicular
en el piso (no por un obstaculo). El robot debe DETENERSE en esa linea, no cuando
el ToF dispara. Este modulo detecta la franja roja en una ROI frontal y estima a
que distancia (proxy) esta, para disparar la maniobra de interseccion.

El rojo en HSV "envuelve" el matiz (cerca de 0 y de 180), por eso se usan DOS
rangos. La distancia es un proxy por la fila del centroide: cuanto mas abajo en la
imagen aparece la franja, mas cerca esta (distance -> 0). Para distancia metrica
real haria falta la calibracion extrinseca (ground projection).
"""
import cv2
import numpy as np

# Rangos HSV del rojo (OpenCV: H in [0,179]). Dos tramos por el wrap-around.
RED_LOW_1 = (0, 70, 50)
RED_HIGH_1 = (10, 255, 255)
RED_LOW_2 = (170, 70, 50)
RED_HIGH_2 = (179, 255, 255)

MIN_FRACTION = 0.02       # fraccion minima de pixeles rojos en la ROI para "detectada"
ROI_FRAC = 0.5            # mira solo la mitad INFERIOR (lo que el robot tiene delante)
EPS_FRAC = 1e-3


def red_mask(bgr):
    """Mascara binaria (0/255) de los pixeles rojos de una imagen BGR."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array(RED_LOW_1), np.array(RED_HIGH_1))
    m2 = cv2.inRange(hsv, np.array(RED_LOW_2), np.array(RED_HIGH_2))
    return cv2.bitwise_or(m1, m2)


def detect_stop_line(bgr, min_fraction=MIN_FRACTION, roi_frac=ROI_FRAC,
                     homography=None):
    """Detecta la franja roja de stop en la ROI frontal (inferior) de la imagen.

    Devuelve un dict:
      - detected (bool): si hay suficiente rojo en la ROI.
      - red_fraction (float): fraccion de pixeles rojos en la ROI [0..1].
      - centroid_row (float|None): fila media del rojo en la imagen completa.
      - distance (float|None): proxy en [0..1]; 0 = la linea esta justo delante
        (abajo en la imagen), 1 = lejos (arriba). None si no se detecta.
      - distance_m (float|None): distancia REAL hacia adelante (metros) al
        centroide de la franja, si se pasa `homography` (calibracion extrinseca);
        None sin homografia o si el centroide cae en el horizonte.
    `roi_frac` acota la franja inferior considerada (0.5 = mitad de abajo).
    """
    h = bgr.shape[0]
    roi_frac = min(max(roi_frac, EPS_FRAC), 1.0)
    y0 = int(round(h * (1.0 - roi_frac)))     # inicio de la ROI inferior
    mask = red_mask(bgr)
    roi = mask[y0:, :]
    red_fraction = float(np.count_nonzero(roi)) / float(roi.size)
    if red_fraction < min_fraction:
        return {"detected": False, "red_fraction": red_fraction,
                "centroid_row": None, "distance": None, "distance_m": None}
    ys, xs = np.nonzero(roi)
    centroid_row = float((ys + y0).mean())     # fila media (imagen completa)
    distance = float((h - centroid_row) / h)   # abajo -> cerca (0); arriba -> lejos (1)
    distance_m = None
    if homography is not None:
        from . import ground_projection
        centroid_col = float(xs.mean())
        try:
            distance_m = float(ground_projection.forward_distance(
                homography, centroid_col, centroid_row))
        except ground_projection.GroundProjectionError:
            distance_m = None                  # centroide en el horizonte
    return {"detected": True, "red_fraction": red_fraction,
            "centroid_row": centroid_row, "distance": distance,
            "distance_m": distance_m}
