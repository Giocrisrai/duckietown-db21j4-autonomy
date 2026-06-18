"""Tests para la política de AprilTags."""
from object_detection.apriltag_policy import tag_meaning, select_action, TAG_MEANINGS


def test_tag_conocido():
    assert tag_meaning(1) == ("stop", "stop")


def test_tag_desconocido():
    assert tag_meaning(9999) == ("unknown", None)


def test_select_prioriza_stop():
    assert select_action([10, 1]) == "stop"   # stop manda sobre turn_left


def test_select_fallback_accion_conocida():
    assert select_action([10]) == "left"


def test_select_sin_conocidos():
    assert select_action([9999]) is None
