"""Tests de calibracion de ruedas (trim/gain)."""
from duckietown_planning.calibration import apply_trim_gain


def test_sin_trim_solo_gain():
    l, r = apply_trim_gain(1.0, 1.0, trim=0.0, gain=0.5)
    assert l == 0.5 and r == 0.5


def test_trim_acelera_derecha():
    l, r = apply_trim_gain(1.0, 1.0, trim=0.1, gain=1.0)
    assert r > l                                # corrige jalar a la izquierda
    assert abs(l - 0.9) < 1e-9 and abs(r - 1.1) < 1e-9
