"""Deteccion de la linea central amarilla por HSV (lane following clasico, CPU).

Adaptado del repo del colega Luissalamanca23/Duckietown_Follow_Line_Pi
(robot/vision.py). Da un seguidor de linea propio sin depender del pipeline de
dt-core: recorta el tercio superior, umbraliza el amarillo en HSV y devuelve el
bounding box del contorno relevante. El control (control/lane_follow_pid.py)
calcula el error cross-track desde ese bbox.
"""
from collections import namedtuple

import cv2
import numpy as np

# Umbrales por defecto (del sistema que funcionaba en el repo del colega).
YELLOW_LOWER = (15, 27, 125)
YELLOW_UPPER = (36, 255, 255)

LaneDetection = namedtuple(
    "LaneDetection", "found x y w h yellow_px frame_w frame_h crop_top")


def detect_yellow_line(frame_bgr, lower=YELLOW_LOWER, upper=YELLOW_UPPER,
                       crop_top_fraction=1.0 / 3.0, y_too_close=200, min_area=0):
    """Detecta la linea amarilla. Devuelve LaneDetection (bbox en coords del crop).

    Toma el contorno de mayor area; si esta demasiado abajo (y > y_too_close) y hay
    mas de uno, usa el segundo (la franja cercana suele ser ruido del pie del robot).
    """
    h = frame_bgr.shape[0]
    crop_top = int(h * crop_top_fraction)
    cropped = frame_bgr[crop_top:, :]
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    fh, fw = mask.shape
    yellow_px = int(cv2.countNonZero(mask))

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if min_area > 0:
        contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return LaneDetection(False, 0, 0, 0, 0, yellow_px, fw, fh, crop_top)

    largest = max(contours, key=cv2.contourArea)
    x, y, w, hh = cv2.boundingRect(largest)
    if y > y_too_close and len(contours) > 1:
        rest = [c for c in contours if c is not largest]
        x, y, w, hh = cv2.boundingRect(max(rest, key=cv2.contourArea))
    return LaneDetection(True, x, y, w, hh, yellow_px, fw, fh, crop_top)


def center_x(det):
    """Centro horizontal del frame (setpoint base del control)."""
    return det.frame_w // 2
