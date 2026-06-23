"""IO real de la Jetson para `tb6612.py`: implementa set_gpio/set_pwm con Jetson.GPIO.

El backend `gpio` se INYECTA (Jetson.GPIO en el robot; un fake en tests). En el
robot:
    from jetson_io import JetsonIO, DEFAULT_PINS
    from tb6612 import TB6612Motor
    io = JetsonIO()                                  # usa Jetson.GPIO real
    L = TB6612Motor(io, **DEFAULT_PINS["left"])
    R = TB6612Motor(io, **DEFAULT_PINS["right"])
    L.run(0.3); R.run(0.3)                            # ¡ruedas en el aire!

Los pines PWM (32/33) son los de **PWM por hardware** del Jetson Nano; hay que
habilitarlos UNA vez con `sudo /opt/nvidia/jetson-io/jetson-io.py` (o el overlay).
"""

DEFAULT_PWM_HZ = 1000

# Mapa de pines del header de 40 pines (numeracion BOARD). Evita los que usa el
# HATv3 para no chocar si la HUT sigue puesta; PWM por hardware en 32 y 33.
DEFAULT_PINS = {
    "left":  {"in1": 29, "in2": 31, "pwm": 32},   # PWM0 (pin 32)
    "right": {"in1": 35, "in2": 37, "pwm": 33},   # PWM2 (pin 33)
}


def duty_to_percent(duty):
    """duty [0..1] -> ciclo de trabajo [0..100] (Jetson.GPIO PWM usa porcentaje)."""
    return max(0.0, min(1.0, duty)) * 100.0


class JetsonIO:
    """set_gpio(pin, level) y set_pwm(pin, duty) sobre Jetson.GPIO (BOARD)."""

    def __init__(self, gpio=None, pwm_hz=DEFAULT_PWM_HZ):
        if gpio is None:                       # robot-side
            import Jetson.GPIO as gpio
        self.gpio = gpio
        self.pwm_hz = pwm_hz
        self.gpio.setmode(self.gpio.BOARD)
        self._out = set()
        self._pwm = {}

    def _ensure_out(self, pin):
        if pin not in self._out:
            self.gpio.setup(pin, self.gpio.OUT)
            self._out.add(pin)

    def set_gpio(self, pin, level):
        self._ensure_out(pin)
        self.gpio.output(pin, level)

    def set_pwm(self, pin, duty):
        if pin not in self._pwm:
            self._ensure_out(pin)
            p = self.gpio.PWM(pin, self.pwm_hz)
            p.start(0.0)
            self._pwm[pin] = p
        self._pwm[pin].ChangeDutyCycle(duty_to_percent(duty))

    def cleanup(self):
        for p in self._pwm.values():
            try:
                p.stop()
            except Exception:
                pass
        try:
            self.gpio.cleanup()
        except Exception:
            pass
