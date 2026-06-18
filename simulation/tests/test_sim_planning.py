"""Tests para la simulación de lazo cerrado del cerebro de planificación."""
from duckietown_planning.map_graph import MapGraph
from sim_planning import simulate_route


def _grafo():
    """Grafo de prueba: A->B->C->D, con atajo A->C."""
    return MapGraph(
        ["A", "B", "C", "D"],
        [{"from": "A", "to": "B", "turn": "straight"},
         {"from": "B", "to": "C", "turn": "right"},
         {"from": "C", "to": "D", "turn": "right"},
         {"from": "A", "to": "C", "turn": "right"}],
    )


def test_simulacion_llega_al_destino():
    """Simula una ruta desde A a D y valida que llegue al destino."""
    final, giros = simulate_route(_grafo(), "A", "D")
    assert final == "D"
    assert giros == ["right", "right"]   # A->C->D, ruta mas corta


def test_simulacion_mismo_nodo():
    """Simula cuando inicio y destino son el mismo nodo."""
    final, giros = simulate_route(_grafo(), "A", "A")
    assert final == "A"
    assert giros == []
