"""Política reactiva: elige un giro válido en una intersección."""

TURNS = ("left", "straight", "right")


def choose_turn(available, planned=None,
                default_priority=("straight", "right", "left")) -> str:
    """Devuelve un giro dentro de `available`.

    - Si hay `planned` y es válido, se respeta (lo usa el planeador global).
    - Si no, se elige por `default_priority`.

    Args:
        available: Lista de giros disponibles ("left", "straight", "right").
        planned: Giro sugerido por el planeador global o None.
        default_priority: Orden de preferencia por defecto.

    Returns:
        str: Giro elegido.

    Raises:
        ValueError: Si no hay salidas disponibles.
    """
    if not available:
        raise ValueError("no hay salidas disponibles en la intersección")
    if planned is not None and planned in available:
        return planned
    for t in default_priority:
        if t in available:
            return t
    return available[0]
