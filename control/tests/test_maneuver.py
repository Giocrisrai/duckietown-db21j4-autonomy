"""Tests de la maniobra por demostracion (replay a distancia por rueda).

Patron portado (adaptado) del repo del colega Luissalamanca23/
Duckietown_Follow_Line_Pi (robot/maneuver.py). Grabas cuanto avanzo cada rueda en
el cruce y reproduces en lazo cerrado preservando la RAZON -> reproduce la forma
del arco, no solo el punto final. Util para giros de interseccion en competencia.
"""
import pytest
from maneuver import replay_step, reached, MIN_DUTY


def test_reached_direccional():
    assert reached(0.5, 0.50, tol=0.005) is True       # llego
    assert reached(0.5, 0.40, tol=0.005) is False
    # objetivo negativo (retroceso): llega cuando measured <= target+tol
    assert reached(-0.5, -0.50, tol=0.005) is True
    assert reached(-0.5, -0.40, tol=0.005) is False


def test_ambas_llegaron_done():
    uL, uR, done = replay_step(0.5, 0.38, 0.5, 0.38, max_duty=0.5)
    assert done is True and (uL, uR) == (0.0, 0.0)


def test_recto_ruedas_iguales():
    uL, uR, done = replay_step(0.5, 0.5, 0.0, 0.0, max_duty=0.5)
    assert done is False
    assert uL == pytest.approx(uR)        # mismas distancias -> recto


def test_giro_preserva_la_razon():
    """tL=0.5, tR=0.38 desde cero: la rueda lider satura, la otra escala por la razon."""
    uL, uR, done = replay_step(0.5, 0.38, 0.0, 0.0, max_duty=0.5)
    assert done is False
    assert uL == pytest.approx(0.5)                       # lider al tope
    assert uR == pytest.approx(0.5 * 0.38 / 0.5)          # razon preservada
    assert uL / uR == pytest.approx(0.5 / 0.38)


def test_rueda_que_llega_se_detiene_la_otra_sigue():
    # L ya llego (0.5), R a medias (0.2 de 0.38) -> uL=0, uR>0.
    uL, uR, done = replay_step(0.5, 0.38, 0.5, 0.2, max_duty=0.5)
    assert done is False
    assert uL == 0.0 and uR > 0.0


def test_piso_min_duty_cerca_del_objetivo():
    """Error chico -> base no baja del MIN_DUTY (vence el deadband)."""
    uL, uR, done = replay_step(0.30, 0.30, 0.29, 0.29, max_duty=0.5)
    assert done is False
    assert abs(uL) == pytest.approx(MIN_DUTY)
