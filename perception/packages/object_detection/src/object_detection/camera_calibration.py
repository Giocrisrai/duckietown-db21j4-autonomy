"""Calibracion de camara del Duckiebot: intrinsecos + correccion de distorsion.

Formato ROS CameraInfo, igual al del stack oficial
(/data/config/calibrations/camera_intrinsic/<robot>.yaml). La camara del DB21 es
gran-angular (~160deg fisheye), por lo que la correccion de distorsion es clave
para el seguir-linea y la proyeccion a suelo.
"""
from pathlib import Path
import numpy as np
import yaml


class CameraCalibrationError(Exception):
    """Calibracion de camara invalida."""


def validate_intrinsics(data: dict) -> dict:
    """Valida un dict de intrinsecos (CameraInfo). Lanza CameraCalibrationError."""
    for key in ("image_width", "image_height", "camera_matrix", "distortion_coefficients"):
        if key not in data:
            raise CameraCalibrationError(f"falta '{key}'")
    if data["image_width"] <= 0 or data["image_height"] <= 0:
        raise CameraCalibrationError("dimensiones de imagen invalidas")
    cm = data["camera_matrix"]
    if cm.get("rows") != 3 or cm.get("cols") != 3 or len(cm.get("data", [])) != 9:
        raise CameraCalibrationError("camera_matrix debe ser 3x3 (9 valores)")
    return data


def load_intrinsics(path) -> dict:
    """Carga y valida el YAML de intrinsecos de la camara."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return validate_intrinsics(data)


def camera_matrix(data: dict) -> np.ndarray:
    """Matriz intrinseca K (3x3)."""
    return np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)


def dist_coeffs(data: dict) -> np.ndarray:
    """Coeficientes de distorsion."""
    return np.array(data["distortion_coefficients"]["data"], dtype=np.float64)


def undistort(img, data: dict):
    """Corrige la distorsion de un frame BGR usando los intrinsecos (cv2)."""
    import cv2
    return cv2.undistort(img, camera_matrix(data), dist_coeffs(data))
