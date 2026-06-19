"""Chequeos de salud de un frame de camara (detecta fallos comunes de hardware).

Util para diagnosticar compatibilidad/funcionamiento de la camara CSI del
Duckiebot: frames vacios, resolucion incorrecta o imagen casi negra (camara
tapada, cable suelto, o sin exposicion).
"""
import numpy as np


def check_frame(img, min_size=(160, 120), dark_thresh: float = 5.0):
    """Devuelve (ok: bool, motivo: str) para un frame BGR (numpy).

    Detecta: frame None (sin captura), array sin dimensiones de imagen,
    resolucion por debajo de `min_size` (w, h), o imagen casi negra
    (media de intensidad < dark_thresh).
    """
    if img is None:
        return False, "frame None (sin captura)"
    arr = np.asarray(img)
    if arr.ndim < 2:
        return False, "frame sin dimensiones de imagen"
    h, w = arr.shape[:2]
    min_w, min_h = min_size
    if w < min_w or h < min_h:
        return False, f"resolucion baja {w}x{h} (< {min_w}x{min_h})"
    if float(arr.mean()) < dark_thresh:
        return False, "frame casi negro (camara tapada o sin exposicion)"
    return True, "ok"
