from pathlib import Path
from duckietown_planning.map_graph import MapGraph, load_map


def test_neighbors_y_turns():
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "straight"}])
    assert g.neighbors("A") == [("straight", "B")]
    assert g.turns_from("A") == ["straight"]
    assert g.neighbors("B") == []


def test_load_map_del_yaml():
    g = load_map(Path("planning/maps/example_loop.yaml"))
    assert set(g.nodes) == {"A", "B", "C", "D"}
    assert ("straight", "B") in g.neighbors("A")
    assert ("right", "C") in g.neighbors("A")
