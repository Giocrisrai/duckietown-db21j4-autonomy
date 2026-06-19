"""Tests para la política de AprilTags (tabla oficial Duckietown apriltagsDB.yaml)."""
from object_detection.apriltag_policy import (
    tag_meaning,
    select_action,
    load_apriltag_db,
    SIGN_ACTIONS,
)


def test_db_carga_y_tiene_tipos():
    db = load_apriltag_db()
    assert db[1] == "stop"          # id 1 = stop (oficial)
    assert db[2] == "yield"
    assert db[8] == "4-way-intersect"
    assert db[12] == "pedestrian"


def test_tag_stop():
    assert tag_meaning(1) == ("stop", "stop")


def test_tag_yield():
    assert tag_meaning(2) == ("yield", "yield")


def test_tag_interseccion():
    assert tag_meaning(8) == ("4-way-intersect", "intersection")


def test_tag_peaton_cede():
    assert tag_meaning(12) == ("pedestrian", "yield")


def test_tag_desconocido():
    assert tag_meaning(999999) == ("unknown", None)


def test_select_prioriza_stop():
    # id 8 (interseccion) + id 1 (stop) -> stop manda
    assert select_action([8, 1]) == "stop"


def test_select_prioriza_yield_sobre_interseccion():
    # id 8 (interseccion) + id 2 (yield) -> yield manda sobre interseccion
    assert select_action([8, 2]) == "yield"


def test_select_fallback_interseccion():
    assert select_action([8]) == "intersection"


def test_select_sin_conocidos():
    assert select_action([999999]) is None
