"""Tests del lazo de marcha integrado: bbox de linea -> duty de ruedas.

Une hsv_lane (deteccion) -> cross_track_error -> LanePID -> vw_to_duty ->
motor_safety, todo el camino percepcion->control->duty, sin hardware.
"""
import sys
from pathlib import Path

import pytest

# La deteccion vive en el paquete de percepcion.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "perception/packages/object_detection/src"))
from object_detection.hsv_lane import LaneDetection      # noqa: E402
from lane_follow_loop import LaneFollowLoop               # noqa: E402


def _det(x, found=True, frame_w=640):
    # y/h fijos; lo que mueve el error es x (posicion horizontal de la linea).
    return LaneDetection(found, x, 0, 40, 20, 999, frame_w, 160, 160)


def test_sin_linea_se_detiene():
    loop = LaneFollowLoop()
    assert loop.step(_det(0, found=False), dt=0.1) == (0.0, 0.0)


def test_linea_centrada_va_casi_recto():
    loop = LaneFollowLoop(v_nominal=0.2)
    # linea cerca del centro -> ruedas casi iguales y positivas (avanza).
    ul, ur = loop.step(_det(300), dt=0.1)        # centro ~ 320
    assert ul > 0 and ur > 0
    assert ul == pytest.approx(ur, abs=0.05)


def test_linea_a_la_izquierda_gira_a_la_izquierda():
    loop = LaneFollowLoop(v_nominal=0.2)
    ul, ur = loop.step(_det(50), dt=0.1)         # linea bien a la izquierda
    assert ur > ul                                # rueda derecha mas rapida = gira izq


def test_duty_acotado_a_max_duty():
    loop = LaneFollowLoop(v_nominal=5.0, max_duty=0.5)   # v absurda -> satura
    ul, ur = loop.step(_det(320), dt=0.1)
    assert abs(ul) <= 0.5 and abs(ur) <= 0.5


def test_reset_propaga_al_pid():
    loop = LaneFollowLoop()
    loop.step(_det(50), dt=0.1)
    loop.reset()
    assert loop.pid.integral == 0.0
