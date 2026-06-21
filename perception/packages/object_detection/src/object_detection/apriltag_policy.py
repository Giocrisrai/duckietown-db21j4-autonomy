"""Mapea IDs de AprilTag de Duckietown a su tipo de senal y accion sugerida.

Usa la tabla OFICIAL de Duckietown (`data/apriltagsDB.yaml`, del repo
duckietown/signs-and-tags): 552 tags, familia tag36h11, 6.5 cm. Cada tag de tipo
TrafficSign tiene un `traffic_sign_type` (stop, yield, 4-way-intersect, etc.).
Aqui mapeamos cada tipo a una accion para el control del robot.
"""
from pathlib import Path
import yaml

_DB_PATH = Path(__file__).resolve().parent / "data" / "apriltagsDB.yaml"

# Accion del robot por tipo de senal (segun semantica Duckietown).
SIGN_ACTIONS = {
    "stop": "stop",
    "do-not-enter": "stop",
    "yield": "yield",
    "pedestrian": "yield",         # ceder ante peatones
    "duck-crossing": "yield",      # ceder ante patos
    "4-way-intersect": "intersection",   # 4-way stop: turna por prioridad (cede al de la derecha)
    "T-intersection": "yield-main",      # via secundaria: cede a TODA la via principal
    "left-T-intersect": "yield-main",
    "right-T-intersect": "yield-main",
}

_db_cache = None


def load_apriltag_db(path=_DB_PATH) -> dict:
    """Carga apriltagsDB.yaml -> dict {tag_id: traffic_sign_type} (solo TrafficSign)."""
    data = yaml.safe_load(Path(path).read_text())
    return {
        d["tag_id"]: d["traffic_sign_type"]
        for d in data
        if d.get("tag_type") == "TrafficSign" and d.get("traffic_sign_type")
    }


def _db() -> dict:
    """Tabla oficial cacheada (carga perezosa)."""
    global _db_cache
    if _db_cache is None:
        _db_cache = load_apriltag_db()
    return _db_cache


def tag_meaning(tag_id, db=None):
    """Devuelve (tipo_de_senal, accion) para un tag id.

    ('unknown', None) si el id no esta en la tabla. Si el tipo no tiene accion
    asociada, la accion es None (la senal se detecta pero no fuerza maniobra).
    """
    table = db if db is not None else _db()
    tipo = table.get(tag_id)
    if tipo is None:
        return ("unknown", None)
    return (tipo, SIGN_ACTIONS.get(tipo))


def select_action(tag_ids, db=None, priority=("stop", "yield")):
    """Accion mas prioritaria entre los tags detectados.

    Las acciones en `priority` mandan (stop sobre yield sobre el resto); si
    ninguna aplica, devuelve la primera accion no nula; None si nada aplica.
    """
    meanings = [tag_meaning(t, db) for t in tag_ids]
    for accion_prioritaria in priority:
        for _tipo, accion in meanings:
            if accion == accion_prioritaria:
                return accion
    for _tipo, accion in meanings:
        if accion is not None:
            return accion
    return None
