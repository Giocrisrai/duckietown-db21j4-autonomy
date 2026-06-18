"""Tests para la localizacion por conteo de intersecciones."""
import pytest
from duckietown_planning.map_graph import MapGraph
from duckietown_planning.localization import IntersectionLocalizer


def _grafo():
    return MapGraph(["A", "B", "C"],
                    [{"from": "A", "to": "B", "turn": "straight"},
                     {"from": "B", "to": "C", "turn": "right"}])


def test_avanza_con_giro_valido():
    loc = IntersectionLocalizer(_grafo(), "A")
    assert loc.update("straight") == "B"
    assert loc.current == "B"


def test_giro_invalido_lanza_error():
    loc = IntersectionLocalizer(_grafo(), "A")
    with pytest.raises(ValueError):
        loc.update("right")
