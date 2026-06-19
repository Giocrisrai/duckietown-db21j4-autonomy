"""Controlador PID para el control de movimiento del Duckiebot (seguir linea).

Version propia, testeable y simulable, para sintonizar y optimizar las ganancias
antes de llevarlas al lane_controller del robot. Incluye anti-windup (clamp del
termino integral) y limites de salida.
"""


class PID:
    def __init__(self, kp, ki, kd, out_limits=None, i_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limits = out_limits          # (min, max) o None
        self.i_limits = i_limits              # (min, max) anti-windup del integral
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def step(self, error, dt):
        """Devuelve la salida de control para el error actual y paso dt (s)."""
        # Proporcional
        p = self.kp * error
        # Integral con anti-windup
        self._integral += error * dt
        if self.i_limits is not None:
            lo, hi = self.i_limits
            self._integral = max(lo, min(hi, self._integral))
        i = self.ki * self._integral
        # Derivativo (0 en el primer paso)
        if self._prev_error is None or dt <= 0:
            d = 0.0
        else:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        out = p + i + d
        if self.out_limits is not None:
            lo, hi = self.out_limits
            out = max(lo, min(hi, out))
        return out
