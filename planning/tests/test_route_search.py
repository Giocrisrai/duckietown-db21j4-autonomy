from duckietown_planning.map_graph import MapGraph
from duckietown_planning.route_search import plan_route


def _grafo():
    return MapGraph(
        ["A", "B", "C", "D"],
        [{"from": "A", "to": "B", "turn": "straight"},
         {"from": "B", "to": "C", "turn": "right"},
         {"from": "C", "to": "D", "turn": "right"},
         {"from": "A", "to": "C", "turn": "right"}],
    )


def test_ruta_directa_mas_corta():
    # A->C directo (right) es mas corto que A->B->C
    assert plan_route(_grafo(), "A", "C") == ["right"]


def test_ruta_multietapa():
    assert plan_route(_grafo(), "A", "D") == ["right", "right"]  # A->C->D


def test_mismo_nodo():
    assert plan_route(_grafo(), "A", "A") == []


def test_sin_ruta():
    assert plan_route(_grafo(), "D", "A") is None
