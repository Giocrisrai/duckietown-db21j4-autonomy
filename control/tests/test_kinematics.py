"""Tests de la cinematica inversa (v, omega) -> duty de ruedas.

Patron portado (adaptado) del repo del colega Luissalamanca23/
Duckietown_Follow_Line_Pi (robot/kinematics.py), que a su vez porta la formula
del kinematics_node oficial de Duckietown.
"""
import pytest
from kinematics import vw_to_duty, DEFAULTS


def test_quieto():
    assert vw_to_duty(0.0, 0.0) == (0.0, 0.0)


def test_recto_simetrico_sin_trim():
    ul, ur = vw_to_duty(0.2, 0.0, dict(DEFAULTS, trim=0.0))
    assert ul == pytest.approx(ur)       # sin trim, recto -> ruedas iguales
    assert ul > 0


def test_giro_izquierda_rueda_derecha_mas_rapida():
    ul, ur = vw_to_duty(0.2, 2.0)        # omega>0 = giro a la izquierda
    assert ur > ul


def test_trim_acelera_la_derecha():
    base_l, base_r = vw_to_duty(0.2, 0.0, dict(DEFAULTS, trim=0.0))
    tl, tr = vw_to_duty(0.2, 0.0, dict(DEFAULTS, trim=0.1))
    assert tr > base_r and tl < base_l   # trim>0 sube derecha, baja izquierda


def test_clamp_a_limite():
    ul, ur = vw_to_duty(100.0, 0.0, dict(DEFAULTS, limit=0.5))
    assert ul == 0.5 and ur == 0.5       # saturado al limite
    ul, ur = vw_to_duty(-100.0, 0.0, dict(DEFAULTS, limit=0.5))
    assert ul == -0.5 and ur == -0.5
