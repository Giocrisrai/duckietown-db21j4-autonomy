"""Lógica de freno de emergencia basada en la distancia del ToF VL53L0X."""


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
