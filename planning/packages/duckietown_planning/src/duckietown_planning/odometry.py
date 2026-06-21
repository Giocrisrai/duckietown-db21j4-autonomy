"""Odometria / deadreckoning del Duckiebot a partir de encoders de rueda.

Reproduce (y testea) la funcionalidad de deadreckoning del stack oficial:
integra los ticks de los encoders a una pose (x, y, theta) con el modelo
diferencial. DB21: 135 ticks/rev, radio de rueda ~31.8 mm.
"""
import math

TICKS_PER_REV = 135
WHEEL_RADIUS = 0.0318      # m
BASELINE = 0.10            # m (separacion entre ruedas)


def ticks_to_distance(ticks, resolution=TICKS_PER_REV, wheel_radius=WHEEL_RADIUS):
    """Distancia lineal recorrida por una rueda (m) dados sus ticks."""
    revoluciones = ticks / resolution
    return 2.0 * math.pi * wheel_radius * revoluciones


def update_pose(x, y, theta, d_left, d_right, baseline=BASELINE):
    """Integra un paso de deadreckoning diferencial (arco de curvatura constante).

    Forma cerrada EXACTA: en el paso, v y omega se asumen constantes, asi que el
    robot describe un arco de circulo. Por eso el resultado es invariante al
    tamano del paso (dt variable): integrar un arco en 1 paso o en N da la misma
    pose. Un punto medio (1er orden) sesgaba la pose con dt grande (p. ej. un hipo
    del loop en el Jetson mientras gira).

    d_left/d_right: distancia (m) recorrida por cada rueda en el paso.
    Devuelve (x, y, theta) actualizados.
    """
    d = (d_left + d_right) / 2.0
    dtheta = (d_right - d_left) / baseline
    if abs(dtheta) < 1e-9:
        # Limite recto: arco de curvatura ~0 (evita 1/dtheta y la cancelacion).
        return (x + d * math.cos(theta),
                y + d * math.sin(theta),
                theta + dtheta)
    radius = d / dtheta                          # radio del arco (d = radius * dtheta)
    return (x + radius * (math.sin(theta + dtheta) - math.sin(theta)),
            y - radius * (math.cos(theta + dtheta) - math.cos(theta)),
            theta + dtheta)
