"""Tests del IO real de Jetson (Jetson.GPIO) para el driver TB6612.

El backend Jetson.GPIO se inyecta como un FAKE -> testeable sin la placa.
"""
import pytest
from jetson_io import JetsonIO, duty_to_percent, DEFAULT_PINS
from tb6612 import TB6612Motor, HIGH, LOW


def test_duty_to_percent():
    assert duty_to_percent(0.5) == 50.0
    assert duty_to_percent(0.0) == 0.0
    assert duty_to_percent(2.0) == 100.0      # clamp arriba
    assert duty_to_percent(-1.0) == 0.0       # clamp abajo


class _FakePWM:
    def __init__(self, pin, hz):
        self.pin, self.hz, self.duty, self.running = pin, hz, None, False

    def start(self, d):
        self.running, self.duty = True, d

    def ChangeDutyCycle(self, d):
        self.duty = d

    def stop(self):
        self.running = False


class _FakeGPIO:
    BOARD, OUT = "BOARD", "OUT"

    def __init__(self):
        self.mode = None
        self.setup_pins = []
        self.outputs = {}
        self.pwms = {}

    def setmode(self, m):
        self.mode = m

    def setup(self, pin, mode):
        self.setup_pins.append((pin, mode))

    def output(self, pin, level):
        self.outputs[pin] = level

    def PWM(self, pin, hz):
        p = _FakePWM(pin, hz)
        self.pwms[pin] = p
        return p

    def cleanup(self):
        pass


def test_setmode_board():
    g = _FakeGPIO()
    JetsonIO(gpio=g)
    assert g.mode == "BOARD"


def test_set_gpio_configura_y_escribe():
    g = _FakeGPIO()
    io = JetsonIO(gpio=g)
    io.set_gpio(29, HIGH)
    assert (29, "OUT") in g.setup_pins
    assert g.outputs[29] == HIGH


def test_set_pwm_crea_y_ajusta():
    g = _FakeGPIO()
    io = JetsonIO(gpio=g)
    io.set_pwm(32, 0.5)
    assert g.pwms[32].running is True
    assert g.pwms[32].duty == 50.0
    io.set_pwm(32, 0.25)
    assert g.pwms[32].duty == 25.0           # reusa el mismo PWM


def test_integracion_tb6612motor_sobre_jetson_io():
    """El motor TB6612 manejado por el IO de Jetson aplica a los pines reales."""
    g = _FakeGPIO()
    io = JetsonIO(gpio=g)
    p = DEFAULT_PINS["left"]
    m = TB6612Motor(io, in1=p["in1"], in2=p["in2"], pwm=p["pwm"])
    m.run(0.4)
    assert g.outputs[p["in1"]] == HIGH and g.outputs[p["in2"]] == LOW
    assert g.pwms[p["pwm"]].duty == pytest.approx(40.0)
    m.stop()
    assert g.pwms[p["pwm"]].duty == 0.0
