"""Tests de la simulacion autonoma end-to-end de lazo cerrado."""
import numpy as np

from duckietown_planning.map_graph import MapGraph
from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.fsm import (
    LANE_FOLLOWING, APPROACHING_INTERSECTION, AT_INTERSECTION, TURNING, STOPPED,
)
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


def test_obstaculo_en_la_interseccion_no_pierde_el_giro():
    # Pato que cruza JUSTO cuando el robot esta parado en la interseccion: tras
    # despejarse, debe ejecutar el giro planeado y no saltarse la interseccion.
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "left"}])
    ctrl = AutonomousController(g, "A", "B")
    duck = {"class_name": "duckie", "bbox": (280, 300, 360, 460), "confidence": 0.9}

    def step(obs):
        obs.update({"img_w": 640, "img_h": 480, "lane_error": 0.0})
        return ctrl.step(obs)["state"]

    step({"stop_line": True})                                  # APPROACHING
    assert step({"stop_line": True, "at_stop_line": True}) == AT_INTERSECTION
    assert ctrl._turn == "left"
    step({"stop_line": True, "at_stop_line": True, "detections": [duck], "tof": 0.10})  # STOPPED
    # El pato se va; la franja ya quedo atras (stop_line=False):
    assert step({}) == TURNING                                # reanuda en la interseccion y gira
    assert step({"turn_done": True}) == LANE_FOLLOWING        # completa el giro pendiente
    assert ctrl.loc.current == "B" and ctrl.reached()         # y avanzo en la ruta (no se la salto)


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


def test_senal_de_stop_impone_parada_antes_de_girar():
    # Un AprilTag de stop en la interseccion obliga a una parada (dwell) antes de
    # girar; sin la senal el robot gira en cuanto llega a la linea.
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "right"}])

    def ticks_hasta_girar(sign):
        ctrl = AutonomousController(g, "A", "B", stop_sign_dwell=4)
        ctrl.step({"lane_error": 0.0, "stop_line": True})     # APPROACHING
        n_at = 0
        for _ in range(12):                                   # llega a la linea (con/ sin senal)
            obs = {"lane_error": 0.0, "stop_line": True, "at_stop_line": True}
            if sign:
                obs["sign_action"] = sign
            st = ctrl.step(obs)["state"]
            if st == TURNING:
                return n_at
            if st == AT_INTERSECTION:
                n_at += 1
        return n_at

    assert ticks_hasta_girar("stop") >= 4                     # parada obligatoria por la senal
    assert ticks_hasta_girar(None) <= 1                       # sin senal gira de inmediato


def test_yield_cede_solo_si_hay_obstaculo():
    # Una senal de YIELD cede (espera) solo si hay trafico cruzado (obstaculo);
    # si la interseccion esta despejada, pasa sin detenerse (a diferencia del stop).
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "right"}])
    # Pato a media distancia -> fusion da SLOW (obstaculo presente, no bloquea el frente).
    duck_slow = {"class_name": "duckie", "bbox": (280, 200, 360, 300), "confidence": 0.9}

    # Yield + obstaculo: cede (se queda en la interseccion sin girar)...
    c1 = AutonomousController(g, "A", "B")
    c1.step({"lane_error": 0.0, "stop_line": True})
    for _ in range(5):
        st = c1.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                      "sign_action": "yield", "detections": [duck_slow]})["state"]
    assert st == AT_INTERSECTION                              # cedio el paso, no giro
    # ...y al despejarse, procede.
    estados = [c1.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                        "sign_action": "yield", "detections": []})["state"] for _ in range(3)]
    assert TURNING in estados

    # Yield con interseccion despejada desde el inicio: pasa de inmediato (sin dwell).
    c2 = AutonomousController(g, "A", "B")
    c2.step({"lane_error": 0.0, "stop_line": True})
    libres = [c2.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                       "sign_action": "yield"})["state"] for _ in range(3)]
    assert TURNING in libres


def test_senal_de_cruce_cede_a_pato_al_costado():
    # Una senal de peaton / duck-crossing (accion 'yield') hace ceder a un pato que
    # cruza por el lateral (fuera del corredor central) -> el robot para; sin la
    # senal, ese pato lateral no bloquea y sigue. Al despejarse, reanuda.
    pato_lateral = {"class_name": "duckie", "bbox": (500, 300, 580, 440), "confidence": 0.9}

    # Sin senal de cruce: el pato lateral no esta en el camino -> sigue (GO).
    c0 = AutonomousController()
    cmd0 = c0.step({"lane_error": 0.0, "detections": [pato_lateral],
                    "img_w": 640, "img_h": 480})
    assert cmd0["decision"]["action"] == "GO"

    # Con senal de cruce: cede aunque el pato venga por el lateral.
    c1 = AutonomousController()
    cmd1 = c1.step({"lane_error": 0.0, "detections": [pato_lateral],
                    "sign_action": "yield", "img_w": 640, "img_h": 480})
    assert cmd1["state"] == STOPPED
    assert cmd1["decision"]["reason"] == "cruce"
    # El pato termina de cruzar -> reanuda.
    cmd2 = c1.step({"lane_error": 0.0, "detections": [], "sign_action": "yield",
                    "img_w": 640, "img_h": 480})
    assert cmd2["state"] == LANE_FOLLOWING


def test_4way_cede_al_vehiculo_de_la_derecha():
    # En un 4-way (accion 'intersection') se turna por prioridad: se cede al vehiculo
    # de la DERECHA y se procede cuando no hay nadie con prioridad.
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "right"}])
    bot_derecha = {"class_name": "duckiebot", "bbox": (500, 320, 600, 440), "confidence": 0.9}
    bot_izquierda = {"class_name": "duckiebot", "bbox": (40, 320, 140, 440), "confidence": 0.9}

    def en_interseccion(ctrl):
        ctrl.step({"lane_error": 0.0, "stop_line": True})
        ctrl.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                   "sign_action": "intersection", "detections": [bot_derecha]})

    # Vehiculo a la derecha: cede (espera su turno).
    c1 = AutonomousController(g, "A", "B")
    en_interseccion(c1)
    for _ in range(4):
        st = c1.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                      "sign_action": "intersection", "detections": [bot_derecha]})["state"]
    assert st == AT_INTERSECTION                          # cedio al de la derecha
    # El de la derecha ya paso -> es su turno, procede.
    estados = [c1.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                        "sign_action": "intersection", "detections": []})["state"]
               for _ in range(3)]
    assert TURNING in estados

    # Vehiculo a la IZQUIERDA (sin prioridad): procede de inmediato.
    c2 = AutonomousController(g, "A", "B")
    c2.step({"lane_error": 0.0, "stop_line": True})
    libres = [c2.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                       "sign_action": "intersection", "detections": [bot_izquierda]})["state"]
              for _ in range(3)]
    assert TURNING in libres


def test_t_interseccion_cede_a_la_via_principal():
    # En una T (accion 'yield-main') la via secundaria cede a TODA la via principal:
    # a un vehiculo por la izquierda O por la derecha (no solo a la derecha como el 4-way).
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "right"}])
    bot_izq = {"class_name": "duckiebot", "bbox": (40, 320, 140, 440), "confidence": 0.9}

    def proceder_con(sign, det):
        ctrl = AutonomousController(g, "A", "B")
        ctrl.step({"lane_error": 0.0, "stop_line": True})
        # 6 ticks en la linea con la senal y la deteccion dada
        estados = [ctrl.step({"lane_error": 0.0, "stop_line": True, "at_stop_line": True,
                              "sign_action": sign, "detections": det})["state"]
                   for _ in range(6)]
        return TURNING in estados

    # T + vehiculo a la izquierda: NO procede (cede a la via principal).
    assert proceder_con("yield-main", [bot_izq]) is False
    # Pero un 4-way ante el mismo vehiculo a la izquierda SI procede (solo cede a la derecha).
    assert proceder_con("intersection", [bot_izq]) is True
    # T con la via principal despejada: procede.
    assert proceder_con("yield-main", []) is True


def test_do_not_enter_prohibe_ese_giro():
    # Una senal do-not-enter prohibe entrar a esa via: el robot NO toma ese giro y
    # elige una alternativa valida. Mapa con dos salidas hacia B (recto y derecha).
    g = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "straight"},
                              {"from": "A", "to": "B", "turn": "right"}])

    def giro_elegido(forbidden):
        ctrl = AutonomousController(g, "A", "B")
        ctrl.step({"lane_error": 0.0, "stop_line": True})     # APPROACHING
        obs = {"lane_error": 0.0, "stop_line": True, "at_stop_line": True}
        if forbidden:
            obs["forbidden_turn"] = forbidden
        ctrl.step(obs)                                        # AT_INTERSECTION -> elige giro
        return ctrl._turn

    # Sin prohibicion: toma el giro planeado (recto, primero del BFS).
    assert giro_elegido(None) == "straight"
    # Con do-not-enter sobre 'straight': evita ese giro y toma la alternativa.
    assert giro_elegido("straight") == "right"


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


from object_detection.lane_pose import HALF_LANE


def _img_carril(centro_y, pendiente=0.0):
    """Imagen sintetica de carril (amarilla izq, blanca der) proyectada con _HMAT."""
    img = np.full((480, 640, 3), (60, 60, 60), np.uint8)
    for x in (0.14, 0.18, 0.22, 0.26, 0.30):
        cy = centro_y + pendiente * x
        for y, color in ((cy + HALF_LANE, (0, 200, 200)), (cy - HALF_LANE, (240, 240, 240))):
            u, v = ground_to_pixel(_HMAT, x, y)
            uu, vv = int(round(u)), int(round(v))
            img[max(0, vv - 8):vv + 8, max(0, uu - 8):uu + 8] = color
    return img


def test_cerebro_calcula_lane_error_desde_imagen():
    # Con homografia e imagen, el cerebro estima (d, phi) y corrige solo (sin que
    # nadie le pase lane_error). Robot derivado a la derecha -> omega>0 (izquierda).
    ctrl = AutonomousController(homography=_HMAT)
    cmd = ctrl.step({"image": _img_carril(0.05), "img_w": 640, "img_h": 480})
    assert cmd["omega"] > 0.05                            # corrige hacia la izquierda
    assert cmd["lane_pose"]["d"] > 0.02                   # expone la pose estimada

    # Carril centrado -> sin correccion apreciable.
    ctrl2 = AutonomousController(homography=_HMAT)
    cmd2 = ctrl2.step({"image": _img_carril(0.0), "img_w": 640, "img_h": 480})
    assert abs(cmd2["omega"]) < 0.05


def test_cerebro_imagen_sin_homografia_usa_lane_error():
    # Sin homografia, la imagen se ignora y manda el lane_error escalar (compat).
    ctrl = AutonomousController()                         # sin homografia
    cmd = ctrl.step({"image": _img_carril(0.05), "lane_error": 0.0,
                     "img_w": 640, "img_h": 480})
    assert abs(cmd["omega"]) < 1e-9                       # uso el escalar (0.0)
    assert cmd.get("lane_pose") is None
