"""Driver de motor TB6612 manejado NATIVO por la Jetson (GPIO + PWM).

Mueve un motor DC con un puente H TB6612 usando los periféricos PROPIOS de la
Jetson: pines GPIO para la dirección (IN1/IN2) y un pin PWM para la velocidad —
sin ningún PCA9685. Es el análogo Jetson de lo que el repo Pi del colega hace con
la GPIO de la Raspberry (`gpio_drive` + pinctrl). Así "hacemos lo propio con
nuestro dispositivo".

La capa IO se INYECTA (cualquier objeto con set_gpio(pin,level) y set_pwm(pin,duty),
p.ej. un wrapper de Jetson.GPIO) -> la lógica es pura y testeable sin hardware.
Para correr en el robot: pasar un IO que use Jetson.GPIO (PWM por hardware en los
pines 32/33, o PWM por software para la dirección-velocidad en GPIO libres).
"""

HIGH = 1
LOW = 0


def tb6612_inputs(duty, brake=False):
    """duty [-1,1] -> (in1, in2, pwm) para un canal del TB6612.

    duty>0 adelante (IN1=H,IN2=L); duty<0 atrás (IN1=L,IN2=H); duty=0 coast
    (ambos L) o freno (ambos H si brake=True). pwm = |duty| acotado a [0,1].
    """
    d = max(-1.0, min(1.0, duty))
    mag = abs(d)
    if d > 0:
        return HIGH, LOW, mag
    if d < 0:
        return LOW, HIGH, mag
    return (HIGH, HIGH, 0.0) if brake else (LOW, LOW, 0.0)


class TB6612Motor:
    """Un motor del TB6612 sobre pines de la Jetson (IO inyectado)."""

    def __init__(self, io, in1, in2, pwm, brake=False):
        self.io, self.in1, self.in2, self.pwm = io, in1, in2, pwm
        self.brake = brake

    def run(self, duty):
        a, b, p = tb6612_inputs(duty, brake=self.brake)
        self.io.set_gpio(self.in1, a)
        self.io.set_gpio(self.in2, b)
        self.io.set_pwm(self.pwm, p)

    def stop(self):
        self.io.set_pwm(self.pwm, 0.0)
        self.io.set_gpio(self.in1, LOW)
        self.io.set_gpio(self.in2, LOW)
