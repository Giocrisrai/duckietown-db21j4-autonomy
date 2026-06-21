"""Pose de carril por PROYECCION A TIERRA: error lateral y rumbo para el PID.

Paridad con el pipeline oficial line_detector -> ground_projection -> lane_filter:
detecta los pixeles de las lineas del carril (AMARILLA discontinua al centro/izq,
BLANCA solida a la derecha en la conduccion por la derecha de Duckietown), los
proyecta al plano del suelo con la homografia (calibracion extrinseca) y estima la
pose REAL del robot respecto al carril:
  - d   : desviacion lateral respecto al centro del carril, en metros.
  - phi : rumbo (angulo) del carril respecto al eje de avance del robot, en rad.

Esto reemplaza cualquier proxy por medidas metricas. Convenciones (ambas con el
mismo signo para que el control sume): d > 0 y phi > 0 cuando el carril queda /
se va hacia la IZQUIERDA del robot -> con ganancias kp>0 el control comanda
omega>0 (girar a la izquierda) para corregir. El error de direccion combinado
es d + k*phi (cross-track + heading), como en el lane_controller oficial.
"""
import math

import numpy as np

from . import ground_projection

# Rangos HSV (OpenCV H in [0,179]) de las lineas del carril.
YELLOW_LOW = (20, 80, 80)
YELLOW_HIGH = (35, 255, 255)
WHITE_LOW = (0, 0, 170)
WHITE_HIGH = (179, 60, 255)

HALF_LANE = 0.11          # media calzada (m): media anchura nominal del carril DB21
ROI_FRAC = 0.5            # mira la mitad inferior (el suelo frente al robot)
OPEN_KSIZE = 3           # apertura morfologica: borra el speckle del ruido del sensor


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


def lane_heading(yellow_slope, white_slope):
    """Rumbo `phi` (rad) del carril respecto al eje de avance del robot.

    yellow_slope / white_slope: pendiente dy/dx de cada linea en el suelo, o None.
    Las lineas son paralelas, asi que se promedian las pendientes disponibles y se
    toma phi = atan(pendiente). phi > 0 = el carril se va a la izquierda al alejarse.
    """
    slopes = [s for s in (yellow_slope, white_slope) if s is not None]
    if not slopes:
        return None
    return math.atan(sum(slopes) / len(slopes))


def steering_error(d, phi, k_heading=0.5):
    """Error de direccion combinado: d + k_heading * phi (cross-track + heading).

    None-safe: trata las componentes ausentes como 0; devuelve None si ambas faltan.
    """
    if d is None and phi is None:
        return None
    return (d or 0.0) + k_heading * (phi or 0.0)


def _line_stats(mask, homography, y0):
    """(posicion lateral media, pendiente dy/dx) de los pixeles `mask` en el suelo.

    Solo cuenta puntos por delante del robot (x > 0); ignora los del horizonte.
    Devuelve (None, None) si no hay puntos; la pendiente es None si no hay variacion
    suficiente en x para ajustar una recta.
    """
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None, None
    gx, gy = [], []
    for u, v in zip(xs.astype(float), (ys + y0).astype(float)):
        try:
            x, y = ground_projection.pixel_to_ground(homography, u, v)
        except ground_projection.GroundProjectionError:
            continue
        if x > 0:
            gx.append(x)
            gy.append(y)
    if not gx:
        return None, None
    lateral = float(np.median(gy))              # mediana: robusta a pixeles espurios (ruido)
    slope = None
    if len(set(gx)) >= 2:                       # hace falta variacion en x para la recta
        slope = float(np.polyfit(gx, gy, 1)[0])
    return lateral, slope


def estimate_lane_pose(bgr, homography, half_lane=HALF_LANE, roi_frac=ROI_FRAC,
                       open_ksize=OPEN_KSIZE):
    """Estima la pose de carril desde una imagen BGR y la homografia.

    `open_ksize`: lado del kernel de apertura morfologica que limpia el speckle del
    ruido del sensor (0 lo desactiva). Junto con la mediana en _line_stats hace la
    estimacion robusta a ruido gaussiano y a oclusiones parciales de las lineas.

    Devuelve un dict:
      - detected (bool): se vio al menos una linea proyectable.
      - d (float|None): desviacion lateral metrica (m); 0 = centrado.
      - phi (float|None): rumbo del carril (rad); 0 = recto delante.
      - yellow_y, white_y (float|None): posicion lateral de cada linea (m).
    """
    import cv2
    h = bgr.shape[0]
    y0 = int(round(h * (1.0 - min(max(roi_frac, 1e-3), 1.0))))
    hsv = cv2.cvtColor(bgr[y0:, :], cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array(YELLOW_LOW), np.array(YELLOW_HIGH))
    white = cv2.inRange(hsv, np.array(WHITE_LOW), np.array(WHITE_HIGH))
    if open_ksize and open_ksize >= 2:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_ksize, open_ksize))
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
    yellow_y, yellow_slope = _line_stats(yellow, homography, y0)
    white_y, white_slope = _line_stats(white, homography, y0)
    d = lane_offset(yellow_y, white_y, half_lane)
    phi = lane_heading(yellow_slope, white_slope)
    return {"detected": d is not None, "d": d, "phi": phi,
            "yellow_y": yellow_y, "white_y": white_y}
