"""Tests del controlador PID."""
from pid import PID


def test_proporcional():
    pid = PID(0.5, 0.0, 0.0)
    assert pid.step(2.0, 0.1) == 1.0           # kp*error


def test_integral_acumula():
    pid = PID(0.0, 1.0, 0.0)
    assert pid.step(1.0, 1.0) == 1.0           # integral=1
    assert pid.step(1.0, 1.0) == 2.0           # integral=2


def test_derivativo_cero_en_primer_paso():
    pid = PID(0.0, 0.0, 1.0)
    assert pid.step(5.0, 0.1) == 0.0           # sin error previo


def test_limite_de_salida():
    pid = PID(10.0, 0.0, 0.0, out_limits=(-1.0, 1.0))
    assert pid.step(100.0, 0.1) == 1.0         # clamp


def test_reset_limpia_integral():
    pid = PID(0.0, 1.0, 0.0)
    pid.step(1.0, 1.0)
    pid.reset()
    assert pid.step(1.0, 1.0) == 1.0           # integral reiniciado
