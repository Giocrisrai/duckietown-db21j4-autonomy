"""Lógica de freno de emergencia basada en la distancia del ToF VL53L0X."""
import math


def should_brake(distance_m, threshold_m: float = 0.15) -> bool:
    """True si hay un obstáculo dentro del umbral (metros).

    Una lectura None o <= 0 se considera inválida (fuera de rango / error del
    sensor) y NO dispara el freno, para evitar frenazos por ruido.

    Args:
        distance_m: Distancia medida en metros.
        threshold_m: Umbral de distancia en metros para disparar el freno.

    Returns:
        True si distance_m es válida y <= threshold_m, False en caso contrario.
    """
    if distance_m is None or distance_m <= 0:
        return False
    return distance_m <= threshold_m


def should_brake_range(range_m, min_range, max_range, threshold_m: float = 0.15) -> bool:
    """Decide frenar a partir de una lectura sensor_msgs/Range (fail-safe).

    - NaN -> True (lectura ambigua: frenar por seguridad).
    - range_m < min_range (incluye -inf): objeto demasiado cerca -> True.
    - range_m > max_range (incluye +inf): sin objeto en rango -> False.
    - en rango valido -> aplica el umbral via should_brake.
    """
    if math.isnan(range_m):
        return True
    if range_m < min_range:
        return True
    if range_m > max_range:
        return False
    return should_brake(range_m, threshold_m)
