"""Mapea IDs de AprilTag de Duckietown a su significado semantico y accion.

El mapa es ajustable a la pista; los ids de ejemplo siguen convenciones comunes
de Duckietown (señales de stop / interseccion / direccion).
"""

TAG_MEANINGS = {
    1: ("stop", "stop"),
    2: ("t_intersection", "yield"),
    10: ("turn_left", "left"),
    11: ("turn_right", "right"),
    12: ("go_straight", "straight"),
}


def tag_meaning(tag_id, table=TAG_MEANINGS):
    """Devuelve (tipo, accion) para un tag id; ('unknown', None) si no se conoce."""
    return table.get(tag_id, ("unknown", None))


def select_action(tag_ids, table=TAG_MEANINGS, priority=("stop", "yield")):
    """Accion mas prioritaria entre los tags detectados.

    'stop' y 'yield' (por `priority`) mandan; si no hay, la primera accion
    conocida; None si ningun tag es conocido.
    """
    meanings = [tag_meaning(t, table) for t in tag_ids]
    for tipo in priority:
        for m_tipo, m_accion in meanings:
            if m_tipo == tipo:
                return m_accion
    for _m_tipo, m_accion in meanings:
        if m_accion is not None:
            return m_accion
    return None
