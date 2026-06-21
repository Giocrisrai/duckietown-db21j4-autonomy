"""Pose de carril por PROYECCION A TIERRA: error lateral metrico para el PID.

Paridad con el pipeline oficial line_detector -> ground_projection -> lane_filter:
detecta los pixeles de las lineas del carril (AMARILLA discontinua al centro/izq,
BLANCA solida a la derecha en la conduccion por la derecha de Duckietown), los
proyecta al plano del suelo con la homografia (calibracion extrinseca) y estima la
desviacion lateral REAL del robot respecto al centro del carril, en metros.

Esto reemplaza cualquier proxy de error de carril por una medida metrica: el PID
recibe `d` en metros y lo lleva a 0 (centrado). Convencion: d > 0 cuando el centro
del carril queda a la IZQUIERDA del robot (el robot derivo a la derecha) -> con
kp>0 el control comanda omega>0 (girar a la izquierda) para corregir.
"""
import numpy as np

from . import ground_projection

# Rangos HSV (OpenCV H in [0,179]) de las lineas del carril.
YELLOW_LOW = (20, 80, 80)
YELLOW_HIGH = (35, 255, 255)
WHITE_LOW = (0, 0, 170)
WHITE_HIGH = (179, 60, 255)

HALF_LANE = 0.11          # media calzada (m): media anchura nominal del carril DB21
ROI_FRAC = 0.5            # mira la mitad inferior (el suelo frente al robot)


def lane_offset(yellow_y, white_y, half_lane=HALF_LANE):
    """Desviacion lateral `d` (m) del centro del carril respecto al robot.

    yellow_y / white_y: posicion lateral media (m, +izquierda) de cada linea en el
    frame del robot, o None si no se detecto. Devuelve None si no hay ninguna.
      - ambas: centro = (amarilla + blanca) / 2.
      - solo amarilla: el centro esta `half_lane` a su derecha.
      - solo blanca:   el centro esta `half_lane` a su izquierda.
    """
    if yellow_y is not None and white_y is not None:
        return (yellow_y + white_y) / 2.0
    if yellow_y is not None:
        return yellow_y - half_lane
    if white_y is not None:
        return white_y + half_lane
    return None


def _line_lateral(mask, homography, y0):
    """Posicion lateral media (m) de los pixeles `mask` proyectados al suelo, o None.

    Solo cuenta puntos por delante del robot (x > 0); ignora los del horizonte.
    """
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    lat = []
    for u, v in zip(xs.astype(float), (ys + y0).astype(float)):
        try:
            gx, gy = ground_projection.pixel_to_ground(homography, u, v)
        except ground_projection.GroundProjectionError:
            continue
        if gx > 0:
            lat.append(gy)
    return float(np.mean(lat)) if lat else None


def estimate_lane_pose(bgr, homography, half_lane=HALF_LANE, roi_frac=ROI_FRAC):
    """Estima la pose de carril desde una imagen BGR y la homografia.

    Devuelve un dict:
      - detected (bool): se vio al menos una linea proyectable.
      - d (float|None): desviacion lateral metrica (m); 0 = centrado.
      - yellow_y, white_y (float|None): posicion lateral de cada linea (m).
    """
    import cv2
    h = bgr.shape[0]
    y0 = int(round(h * (1.0 - min(max(roi_frac, 1e-3), 1.0))))
    hsv = cv2.cvtColor(bgr[y0:, :], cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array(YELLOW_LOW), np.array(YELLOW_HIGH))
    white = cv2.inRange(hsv, np.array(WHITE_LOW), np.array(WHITE_HIGH))
    yellow_y = _line_lateral(yellow, homography, y0)
    white_y = _line_lateral(white, homography, y0)
    d = lane_offset(yellow_y, white_y, half_lane)
    return {"detected": d is not None, "d": d,
            "yellow_y": yellow_y, "white_y": white_y}
