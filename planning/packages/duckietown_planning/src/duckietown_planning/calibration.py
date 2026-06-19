"""Calibracion cinematica de ruedas del Duckiebot (formato Duckietown).

Maneja gain/trim/baseline/radius como el stack oficial, mas carga/guardado y
validacion del YAML de calibracion (el que vive en el robot en
/data/config/calibrations/kinematics/<robot>.yaml).
"""
from pathlib import Path
import yaml

# Valores por defecto del DB21 (coinciden con los del stack oficial).
DEFAULTS = {
    "gain": 1.0,
    "trim": 0.0,
    "baseline": 0.10,
    "radius": 0.0318,
    "k": 27.0,
    "limit": 1.0,
}


class CalibrationError(Exception):
    """Calibracion invalida."""


def apply_trim_gain(v_left, v_right, trim=0.0, gain=1.0):
    """Aplica gain (escala) y trim (asimetria) a las velocidades de rueda.

    trim>0 acelera la derecha y frena la izquierda (corrige jalar a la izq).
    Devuelve (v_left_cal, v_right_cal).
    """
    return gain * v_left * (1.0 - trim), gain * v_right * (1.0 + trim)


def validate_kinematics(cal: dict) -> dict:
    """Valida un dict de calibracion cinematica. Lanza CalibrationError si algo
    esta fuera de rango fisico. Devuelve el dict completado con DEFAULTS."""
    full = {**DEFAULTS, **(cal or {})}
    if not (0.1 <= full["gain"] <= 3.0):
        raise CalibrationError(f"gain fuera de rango: {full['gain']}")
    if not (-0.5 <= full["trim"] <= 0.5):
        raise CalibrationError(f"trim fuera de rango: {full['trim']}")
    if full["baseline"] <= 0:
        raise CalibrationError(f"baseline debe ser > 0: {full['baseline']}")
    if full["radius"] <= 0:
        raise CalibrationError(f"radius debe ser > 0: {full['radius']}")
    return full


def load_kinematics(path) -> dict:
    """Carga y valida el YAML de calibracion cinematica."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return validate_kinematics(data)


def save_kinematics(path, cal: dict):
    """Valida y guarda el YAML de calibracion cinematica (formato Duckietown)."""
    full = validate_kinematics(cal)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(full, sort_keys=True))
    return path
