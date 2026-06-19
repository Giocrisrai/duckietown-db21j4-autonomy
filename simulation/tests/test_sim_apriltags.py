"""Tests de deteccion/simulacion de AprilTags ("senaleticas QR") y enlace con planificacion."""
import pytest

# Libs nativas opcionales (requirements-apriltag.txt): si no estan, se saltan.
pytest.importorskip("pupil_apriltags")
pytest.importorskip("moms_apriltag")

from sim_apriltags import make_tag_image, detect_tag_ids, detect_and_act, decide_with_signs


def test_genera_y_detecta_stop():
    ids = detect_tag_ids(make_tag_image(1))   # id 1 = stop (oficial)
    assert 1 in ids


def test_stop_da_accion_stop():
    ids, accion = detect_and_act(make_tag_image(1))
    assert 1 in ids
    assert accion == "stop"


def test_interseccion():
    ids, accion = detect_and_act(make_tag_image(8))   # 4-way-intersect
    assert 8 in ids
    assert accion == "intersection"


def test_peaton_cede():
    ids, accion = detect_and_act(make_tag_image(12))  # pedestrian -> yield
    assert 12 in ids
    assert accion == "yield"


def test_decide_stop_manda_sobre_plan():
    assert decide_with_signs([1], planned_turn="left") == "stop"


def test_decide_interseccion_sigue_plan():
    # una senal de interseccion no es de seguridad: se sigue el giro planeado
    assert decide_with_signs([8], planned_turn="left") == "left"


def test_decide_sin_senales_sigue_plan():
    assert decide_with_signs([], planned_turn="right") == "right"
