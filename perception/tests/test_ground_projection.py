"""Tests de proyeccion a tierra / calibracion extrinseca (homografia)."""
import math
import numpy as np
import pytest

from object_detection.ground_projection import (
    GroundProjectionError, validate_extrinsics, load_extrinsics, save_extrinsics,
    homography, estimate_homography, pixel_to_ground, ground_to_pixel,
    forward_distance,
)

W, H = 640, 480

# Cuatro correspondencias pixel<->suelo (x adelante, y izquierda; metros).
GROUND = [(0.10, 0.10), (0.10, -0.10), (0.30, 0.10), (0.30, -0.10)]
IMAGE = [(180.0, 400.0), (460.0, 400.0), (260.0, 250.0), (380.0, 250.0)]


def _H():
    return estimate_homography(IMAGE, GROUND)


def test_estima_homografia_3x3():
    Hm = _H()
    assert Hm.shape == (3, 3)
    assert np.isfinite(Hm).all()


def test_pixel_a_suelo_recupera_correspondencias():
    Hm = _H()
    for (u, v), (x, y) in zip(IMAGE, GROUND):
        gx, gy = pixel_to_ground(Hm, u, v)
        assert abs(gx - x) < 1e-6
        assert abs(gy - y) < 1e-6


def test_round_trip_pixel_suelo_pixel():
    Hm = _H()
    u, v = 320.0, 330.0                       # punto interior arbitrario
    x, y = pixel_to_ground(Hm, u, v)
    uu, vv = ground_to_pixel(Hm, x, y)
    assert abs(uu - u) < 1e-6 and abs(vv - v) < 1e-6


def test_mas_abajo_en_imagen_es_mas_cerca():
    # Mayor v (mas abajo) -> menor distancia hacia adelante (mas cerca del robot).
    Hm = _H()
    cerca = forward_distance(Hm, 320.0, 420.0)
    lejos = forward_distance(Hm, 320.0, 260.0)
    assert cerca < lejos
    assert cerca > 0


def test_punto_en_horizonte_lanza():
    # Un pixel cuyo mapeo cae en el infinito (w ~ 0) no tiene proyeccion a tierra.
    Hm = _H()
    # Construimos H degenerada para forzar w=0 en un punto conocido.
    bad = Hm.copy()
    bad[2, :] = [0.0, 0.0, 0.0]               # tercera fila nula -> w=0 siempre
    with pytest.raises(GroundProjectionError):
        pixel_to_ground(bad, 320.0, 240.0)


def test_validate_extrinsics_ok_y_errores():
    data = {"homography": list(np.eye(3).flatten())}
    assert validate_extrinsics(data)["homography"] == data["homography"]
    with pytest.raises(GroundProjectionError):
        validate_extrinsics({})                          # falta homography
    with pytest.raises(GroundProjectionError):
        validate_extrinsics({"homography": [1, 2, 3]})   # no son 9 valores
    with pytest.raises(GroundProjectionError):
        validate_extrinsics({"homography": [0] * 9})     # singular (no invertible)


def test_save_load_round_trip(tmp_path):
    Hm = _H()
    data = {"homography": [float(x) for x in Hm.flatten()]}
    p = save_extrinsics(tmp_path / "sub" / "robot.yaml", data)
    assert p.exists()
    back = load_extrinsics(p)
    assert np.allclose(homography(back), Hm)
