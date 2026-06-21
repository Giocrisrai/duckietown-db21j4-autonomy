"""Tests de la simulacion autonoma end-to-end de lazo cerrado."""
import numpy as np

from duckietown_planning.map_graph import MapGraph
from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.fsm import APPROACHING_INTERSECTION, AT_INTERSECTION, TURNING, STOPPED
from object_detection.anti_instagram import correct
from object_detection.stop_line import detect_stop_line
from object_detection.ground_projection import (
    estimate_homography, ground_to_pixel, save_extrinsics,
)
from sim_autonomy import run_autonomy

_HMAT = estimate_homography(
    [(180.0, 400.0), (460.0, 400.0), (260.0, 250.0), (380.0, 250.0)],
    [(0.10, 0.10), (0.10, -0.10), (0.30, 0.10), (0.30, -0.10)],
)


def _pato_a(dist_m):
    """Deteccion de pato cuyo borde inferior (centrado) proyecta a dist_m metros."""
    u, v = ground_to_pixel(_HMAT, dist_m, 0.0)
    return {"class_name": "duckie", "bbox": (u - 40, v - 80, u + 40, v), "confidence": 0.9}


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


def test_callejon_sin_salida_no_crashea():
    # Destino inalcanzable: B es un callejon sin salida (sin aristas salientes).
    # El cerebro NO debe inventar un giro fantasma ni lanzar excepcion: se detiene.
    g = MapGraph(["A", "B", "X"], [{"from": "A", "to": "B", "turn": "straight"}])
    r = run_autonomy(g, "A", "X", max_ticks=2000)   # no debe lanzar ValueError
    assert r["reached"] is False
    assert r["final_node"] in ("A", "B")            # se quedo en el grafo, no descarrilo


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


def test_cerebro_usa_distancia_metrica_de_obstaculo():
    # Con homografia, el cerebro decide por distancia REAL: un pato a 0.15 m frena.
    ctrl = AutonomousController(homography=_HMAT)
    cmd = ctrl.step({"lane_error": 0.0, "detections": [_pato_a(0.15)],
                     "img_w": 640, "img_h": 480})
    assert cmd["state"] == STOPPED
    assert cmd["decision"]["action"] == "STOP"
    assert abs(cmd["decision"]["distance_m"] - 0.15) < 1e-3
    # El mismo pato a 0.60 m no frena.
    ctrl2 = AutonomousController(homography=_HMAT)
    cmd2 = ctrl2.step({"lane_error": 0.0, "detections": [_pato_a(0.60)],
                       "img_w": 640, "img_h": 480})
    assert cmd2["decision"]["action"] == "GO"


def test_cerebro_carga_homografia_desde_yaml(tmp_path):
    # El cerebro carga la calibracion extrinseca desde un YAML (ruta) y la usa.
    path = save_extrinsics(tmp_path / "camera_extrinsic" / "robot.yaml",
                           {"homography": [float(x) for x in _HMAT.flatten()]})
    ctrl = AutonomousController(homography=str(path))
    assert ctrl.homography is not None
    cmd = ctrl.step({"lane_error": 0.0, "detections": [_pato_a(0.15)],
                     "img_w": 640, "img_h": 480})
    assert cmd["decision"]["action"] == "STOP"


def test_cerebro_para_en_franja_por_distancia_metrica():
    # Con homografia y distancia metrica a la franja, para a la distancia fijada.
    ctrl = AutonomousController(homography=_HMAT, stop_line_stop_m=0.10)
    s1 = ctrl.step({"lane_error": 0.0, "stop_line": True,
                    "stop_line_distance_m": 0.40, "img_w": 640, "img_h": 480})
    assert s1["state"] == APPROACHING_INTERSECTION       # ve la franja, aun lejos
    s2 = ctrl.step({"lane_error": 0.0, "stop_line": True,
                    "stop_line_distance_m": 0.05, "img_w": 640, "img_h": 480})
    assert s2["state"] == AT_INTERSECTION                # a 0.05 m (<0.10) se detiene
