"""PID de seguir-linea: error cross-track desde el bbox + control con anti-windup.

Adaptado del repo del colega Luissalamanca23/Duckietown_Follow_Line_Pi
(robot/controller.py). El error se calcula con un punto de lookahead sobre el
bbox de la linea; el PID devuelve omega (rad/s). El ANTI-WINDUP (clamp del
integral a omega_max/|ki|) es clave: sin el, con el robot quieto (banco/sim) o en
curva larga el integral se dispara y "clava" el volante en ±omega_max.
"""

# Constantes por defecto del sistema que funcionaba (repo del colega).
KP, KI, KD = 1.0, 0.1, 0.05
SIZE_RATIO = 0.8       # lookahead: target_x = x + size_ratio*(y + h/2)
PIXEL_SCALE = 0.02     # escala pixel -> error normalizado
OMEGA_MAX = 8.0        # clamp de seguridad de omega (rad/s)


def cross_track_error(x, y, h, center_x, size_ratio=SIZE_RATIO,
                      pixel_scale=PIXEL_SCALE, lane_offset_px=0):
    """Error cross-track normalizado desde el bbox de la linea.

    target_x = x + size_ratio*(y + h/2)   # punto de lookahead ('American driver')
    El setpoint se corre lane_offset_px a la izquierda para circular a la derecha
    de la linea (carril derecho). diff>0 = linea a la derecha del objetivo.
    """
    target_x = x + size_ratio * (y + h / 2.0)
    goal = center_x - lane_offset_px
    return (target_x - goal) * pixel_scale


class LanePID:
    """PID con anti-windup. update(diff, dt) -> omega (rad/s)."""

    def __init__(self, kp=KP, ki=KI, kd=KD, omega_max=OMEGA_MAX):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.omega_max = omega_max
        self.reset()

    def reset(self):
        """Limpia integral y prev_diff (llamar en cada transicion de estado)."""
        self.integral = 0.0
        self.prev_diff = 0.0

    def update(self, diff, dt):
        p = self.kp * diff
        self.integral += diff * dt
        if self.ki != 0.0:                       # anti-windup: clamp del integral
            i_clamp = self.omega_max / abs(self.ki)
            self.integral = max(-i_clamp, min(i_clamp, self.integral))
        i = self.ki * self.integral
        d = self.kd * (diff - self.prev_diff) / dt if dt > 0 else 0.0
        omega = -(p + i + d)
        omega = max(-self.omega_max, min(self.omega_max, omega))
        self.prev_diff = diff
        return omega
