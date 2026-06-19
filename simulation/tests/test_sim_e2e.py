"""Test END-TO-END: integra percepcion+planificacion+control+seguridad+calibracion."""
from duckietown_planning.map_graph import MapGraph
from sim_e2e import run_mission


def _grafo():
    return MapGraph(
        ["A", "B", "C", "D"],
        [{"from": "A", "to": "B", "turn": "straight"},
         {"from": "B", "to": "C", "turn": "right"},
         {"from": "C", "to": "D", "turn": "right"},
         {"from": "A", "to": "C", "turn": "right"}],
    )


def test_mision_completa_llega_al_destino():
    r = run_mission(_grafo(), "A", "D")
    assert r["reached"] is True
    assert r["final_node"] == "D"


def test_control_mantiene_el_carril():
    r = run_mission(_grafo(), "A", "D")
    assert r["max_offset"] < 0.25          # el PID mantiene el offset acotado


def test_senal_de_stop_se_respeta():
    r = run_mission(_grafo(), "A", "D", events={"A": {"signs": [1]}})  # id 1 = stop
    assert r["stops"] >= 1
    assert r["reached"] is True


def test_obstaculo_frena_y_aborta():
    # obstaculo a 0.10 m en C -> freno de emergencia, no llega
    r = run_mission(_grafo(), "A", "D", events={"C": {"obstacle_m": 0.10}})
    assert r["reached"] is False
    assert any(a == "frenar" for _, a in r["log"])
