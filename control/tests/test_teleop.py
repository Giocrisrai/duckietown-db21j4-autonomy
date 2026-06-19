"""Tests de teleoperacion (joystick -> ruedas)."""
from teleop import joystick_to_wheels


def test_avanza_recto():
    vl, vr = joystick_to_wheels(1.0, 0.0)
    assert vl == vr and vl > 0


def test_giro_derecha():
    vl, vr = joystick_to_wheels(0.5, 1.0)   # steer derecha
    assert vl > vr                          # rueda izquierda mas rapida


def test_clamp_a_max():
    vl, vr = joystick_to_wheels(1.0, 1.0, max_speed=0.5)
    assert -0.5 <= vl <= 0.5 and -0.5 <= vr <= 0.5


def test_detenido():
    assert joystick_to_wheels(0.0, 0.0) == (0.0, 0.0)
