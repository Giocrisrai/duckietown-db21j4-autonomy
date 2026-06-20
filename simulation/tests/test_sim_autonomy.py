"""Tests de la simulacion autonoma end-to-end de lazo cerrado."""
import numpy as np

from duckietown_planning.map_graph import MapGraph
from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.fsm import APPROACHING_INTERSECTION, AT_INTERSECTION, TURNING, STOPPED
from object_detection.anti_instagram import correct
from object_detection.stop_line import detect_stop_line
from sim_autonomy import run_autonomy


def _graph_lineal():
    # A --right--> B --straight--> C
    nodes = ["A", "B", "C"]
    edges = [
        {"from": "A", "to": "B", "turn": "right"},
        {"from": "B", "to": "C", "turn": "straight"},
    ]
    return MapGraph(nodes, edges)


def test_mision_simple_llega_al_destino():
    g = _graph_lineal()
    r = run_autonomy(g, "A", "C")
    assert r["reached"] is True
    assert r["final_node"] == "C"


def test_sigue_la_ruta_global():
    g = _graph_lineal()
    r = run_autonomy(g, "A", "C")
    assert r["turns_taken"] == ["right", "straight"]


def test_pasa_por_los_estados_de_interseccion():
    g = _graph_lineal()
    r = run_autonomy(g, "A", "C")
    assert APPROACHING_INTERSECTION in r["states"]
    assert AT_INTERSECTION in r["states"]
    assert TURNING in r["states"]


def test_se_mantiene_en_el_carril():
    g = _graph_lineal()
    r = run_autonomy(g, "A", "C")
    assert r["max_offset"] < 0.5            # el PID mantiene el error lateral acotado


def test_frena_ante_obstaculo_y_aun_asi_llega():
    g = _graph_lineal()
    r = run_autonomy(g, "A", "C", obstacle_ticks=range(5, 20))
    assert r["n_stops"] >= 1                 # se detuvo por el pato
    assert STOPPED in r["states"]
    assert r["reached"] is True             # y luego completo la mision


def test_start_igual_goal_llega_de_inmediato():
    g = _graph_lineal()
    r = run_autonomy(g, "C", "C", max_ticks=5)
    assert r["reached"] is True
    assert r["turns_taken"] == []


def test_pipeline_imagen_a_comportamiento():
    # Cadena REAL: imagen con franja roja -> anti-Instagram -> stop_line -> controlador.
    img = np.full((480, 640, 3), 40, dtype=np.uint8)     # escena oscura/sub-expuesta
    img[360:470, :] = (0, 0, 200)                        # franja roja de stop al frente
    corregida = correct(img)                             # normaliza iluminacion
    det = detect_stop_line(corregida)
    assert det["detected"] is True                       # la ve tras corregir

    ctrl = AutonomousController()                        # sin grafo: solo carril+eventos
    cmd = ctrl.step({"lane_error": 0.0, "stop_line": det["detected"],
                     "img_w": 640, "img_h": 480})
    assert cmd["state"] == APPROACHING_INTERSECTION      # la imagen disparo el comportamiento
