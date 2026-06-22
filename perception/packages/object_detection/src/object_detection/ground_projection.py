"""Proyeccion a tierra / calibracion EXTRINSECA (paridad con dt-core ground_projection).

Una homografia 3x3 `H` mapea un pixel de la imagen (u, v) a coordenadas metricas
en el plano del SUELO, en el marco del robot (x adelante, y a la izquierda):

    [X, Y, W]^T = H . [u, v, 1]^T      ->      (x, y) = (X/W, Y/W)

Esto convierte los *proxies* de cercania (fila del bbox / centroide) en distancias
REALES en metros, que es lo que necesitan el seguir-linea fino y decidir donde
parar en la franja de stop. La homografia se estima una vez con un patron de suelo
conocido (4+ correspondencias pixel<->suelo) y vive en el robot en
/data/config/calibrations/camera_extrinsic/<robot>.yaml (lista de 9 valores).
"""
from pathlib import Path
import numpy as np
import yaml

# Por debajo de esto el denominador homogeneo se considera 0 (punto en el horizonte).
_W_EPS = 1e-9
# Determinante minimo para considerar la homografia no singular (invertible).
_DET_EPS = 1e-12


class GroundProjectionError(Exception):
    """Calibracion extrinseca / proyeccion a tierra invalida."""


def validate_extrinsics(data: dict) -> dict:
    """Valida un dict de calibracion extrinseca. Lanza GroundProjectionError.

    Requiere `homography`: 9 numeros (3x3 row-major) que formen una matriz no
    singular (invertible), para poder proyectar suelo->pixel ademas de pixel->suelo.
    """
    if not data or "homography" not in data:
        raise GroundProjectionError("falta 'homography'")
    h = data["homography"]
    try:
        H = np.array(h, dtype=np.float64)
    except (TypeError, ValueError):
        raise GroundProjectionError("homography debe ser una lista de 9 numeros")
    if H.size != 9:
        raise GroundProjectionError("homography debe tener 9 valores (3x3)")
    H = H.reshape(3, 3)
    if not np.isfinite(H).all():
        raise GroundProjectionError("homography con valores no finitos")
    if abs(np.linalg.det(H)) < _DET_EPS:
        raise GroundProjectionError("homography singular (no invertible)")
    return data


def load_extrinsics(path) -> dict:
    """Carga y valida el YAML de calibracion extrinseca."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return validate_extrinsics(data)


def save_extrinsics(path, data: dict):
    """Valida y guarda el YAML de calibracion extrinseca (lista de 9 valores)."""
    full = validate_extrinsics(data)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(full, sort_keys=True))
    return path


def homography(data: dict) -> np.ndarray:
    """Matriz de homografia H (3x3) a partir del dict de extrinsecos."""
    return np.array(data["homography"], dtype=np.float64).reshape(3, 3)


def estimate_homography(image_points, ground_points) -> np.ndarray:
    """Estima H (pixel->suelo) a partir de >=4 correspondencias (u,v)<->(x,y).

    `image_points`: pixeles (u, v); `ground_points`: metros en el suelo (x, y).
    Usa cv2.findHomography (minimos cuadrados) como el procedimiento oficial.
    """
    import cv2
    src = np.array(image_points, dtype=np.float64).reshape(-1, 1, 2)
    dst = np.array(ground_points, dtype=np.float64).reshape(-1, 1, 2)
    if src.shape[0] < 4:
        raise GroundProjectionError("se requieren >=4 correspondencias")
    H, _ = cv2.findHomography(src, dst, method=0)
    if H is None or abs(np.linalg.det(H)) < _DET_EPS:
        raise GroundProjectionError("homografia no estimable (puntos colineales/degenerados)")
    return H


def pixel_to_ground(H, u, v):
    """Proyecta un pixel (u, v) al suelo: devuelve (x, y) en metros (robot frame).

    Lanza GroundProjectionError si el punto cae en el horizonte (w ~ 0).
    """
    X, Y, w = np.asarray(H, dtype=np.float64) @ np.array([u, v, 1.0])
    if abs(w) < _W_EPS:
        raise GroundProjectionError("pixel en el horizonte: sin proyeccion a tierra")
    return (X / w, Y / w)


def ground_to_pixel(H, x, y):
    """Proyecta un punto del suelo (x, y) metros a pixel (u, v) usando H^-1."""
    Hinv = np.linalg.inv(np.asarray(H, dtype=np.float64))
    U, V, w = Hinv @ np.array([x, y, 1.0])
    if abs(w) < _W_EPS:
        raise GroundProjectionError("punto del suelo no proyectable a la imagen")
    return (U / w, V / w)


def forward_distance(H, u, v):
    """Distancia hacia adelante (x, metros) del punto de suelo bajo el pixel (u, v)."""
    return pixel_to_ground(H, u, v)[0]
