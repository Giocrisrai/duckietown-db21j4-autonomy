"""Tests del driver de motor TB6612 manejado NATIVO por la Jetson (GPIO + PWM).

Igual que el colega usa la GPIO de la Raspberry para la dirección, aquí usamos la
GPIO/PWM de la Jetson directo sobre un TB6612 — sin PCA9685. Lógica pura (los pines
GPIO/PWM se inyectan) -> testeable sin hardware.
"""
import pytest
from tb6612 import tb6612_inputs, TB6612Motor, HIGH, LOW


def test_adelante():
    in1, in2, pwm = tb6612_inputs(0.6)
    assert (in1, in2) == (HIGH, LOW)
    assert pwm == pytest.approx(0.6)


def test_atras():
    in1, in2, pwm = tb6612_inputs(-0.4)
    assert (in1, in2) == (LOW, HIGH)
    assert pwm == pytest.approx(0.4)


def test_detenido_coast():
    assert tb6612_inputs(0.0) == (LOW, LOW, 0.0)


def test_freno_activo():
    in1, in2, pwm = tb6612_inputs(0.0, brake=True)
    assert (in1, in2) == (HIGH, HIGH)   # ambos altos = freno
    assert pwm == 0.0


def test_clamp():
    assert tb6612_inputs(5.0)[2] == pytest.approx(1.0)
    assert tb6612_inputs(-5.0)[2] == pytest.approx(1.0)


class _FakeIO:
    """IO falso: registra niveles GPIO y duty PWM por pin."""
    def __init__(self):
        self.gpio = {}
        self.pwm = {}

    def set_gpio(self, pin, level):
        self.gpio[pin] = level

    def set_pwm(self, pin, duty):
        self.pwm[pin] = duty


def test_motor_aplica_a_los_pines_correctos():
    io = _FakeIO()
    m = TB6612Motor(io, in1=31, in2=33, pwm=32)   # pines del header Jetson
    m.run(0.5)
    assert io.gpio[31] == HIGH and io.gpio[33] == LOW
    assert io.pwm[32] == pytest.approx(0.5)
    m.run(-0.5)
    assert io.gpio[31] == LOW and io.gpio[33] == HIGH
    m.stop()
    assert io.pwm[32] == 0.0
