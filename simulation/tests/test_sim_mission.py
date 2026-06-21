"""Test de la mision COMPLETA multi-interseccion en el lazo imagen->imagen."""
import pytest

from duckietown_planning.map_graph import MapGraph
from sim_mission import run_mission_image_loop


def _mapa_3_intersecciones():
    # A --right--> B --straight--> C --left--> D
    return MapGraph(
        ["A", "B", "C", "D"],
        [
            {"from": "A", "to": "B", "turn": "right"},
            {"from": "B", "to": "C", "turn": "straight"},
            {"from": "C", "to": "D", "turn": "left"},
        ],
    )


def test_mision_completa_llega_al_destino():
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D")
    assert r["reached"] is True
    assert r["final_node"] == "D"


def test_ejecuta_los_giros_de_la_ruta_por_vision():
    # El cerebro decide los giros solo con lo que ve (franjas) y respeta la ruta.
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D")
    assert r["turns_taken"] == ["right", "straight", "left"]
    assert r["n_intersections"] == 3


def test_se_mantiene_en_el_carril_toda_la_mision():
    # Pasa por tramos rectos y curvos (curvatura por tramo) sin salirse del carril.
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D")
    assert r["max_off"] < 0.08                       # media calzada ~0.11 m


def test_otra_ruta_mas_corta():
    g = _mapa_3_intersecciones()
    r = run_mission_image_loop(g, "A", "C")          # dos intersecciones
    assert r["reached"] is True
    assert r["turns_taken"] == ["right", "straight"]


def test_mision_completa_con_ruido():
    # Mision entera con ruido fuerte del sensor: la apertura morfologica (carril y
    # franja) lo absorbe -> llega ejecutando la ruta sin falsas franjas ni perder el carril.
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D",
                               noise_sigma=25, seed=1)
    assert r["reached"] is True
    assert r["turns_taken"] == ["right", "straight", "left"]
    assert r["max_off"] < 0.08


def test_mision_completa_con_brillo_y_ruido():
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D",
                               noise_sigma=20, brightness=60, seed=3)
    assert r["reached"] is True
    assert r["turns_taken"] == ["right", "straight", "left"]


def test_obstaculo_en_recta_frena_y_completa():
    # Pato en el primer tramo recto: el robot frena (fusion vision+ToF) y, al
    # despejarse, reanuda y completa la mision con la ruta correcta.
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D",
                               obstacle_ticks=range(20, 40))
    assert r["reached"] is True
    assert r["n_stops"] >= 1
    assert r["turns_taken"] == ["right", "straight", "left"]


def test_obstaculo_en_curva_se_mantiene_en_carril():
    # Pato en el tramo curvo: frena y, al reanudar, vuelve a engancharse a la curva.
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D",
                               obstacle_ticks=range(90, 120))
    assert r["reached"] is True
    assert r["n_stops"] >= 1
    assert r["max_off"] < 0.08


def test_obstaculo_y_ruido_juntos():
    r = run_mission_image_loop(_mapa_3_intersecciones(), "A", "D",
                               obstacle_ticks=range(20, 40), noise_sigma=25, seed=1)
    assert r["reached"] is True
    assert r["turns_taken"] == ["right", "straight", "left"]


def test_robusto_a_la_temporizacion_del_obstaculo():
    # Un pato de 12 ticks empezando en distintos momentos (incl. giros e
    # intersecciones) -> la mision siempre se completa (reanudacion robusta).
    g = _mapa_3_intersecciones()
    for t0 in (10, 55, 90, 130, 170):
        r = run_mission_image_loop(g, "A", "D", obstacle_ticks=range(t0, t0 + 12))
        assert r["reached"] is True, f"fallo con obstaculo en t0={t0}"
        assert r["turns_taken"] == ["right", "straight", "left"]


def _id_de_tipo(tipo):
    from object_detection.apriltag_policy import _db
    return next(i for i, t in _db().items() if t == tipo)


def test_apriltag_stop_para_siempre_yield_solo_si_obstaculo():
    # AprilTags REALES pintados en las senales y detectados con pupil-apriltags SOBRE
    # el frame renderizado. STOP impone parada obligatoria (mas ticks); YIELD con la
    # interseccion despejada NO para (procede como sin senal).
    pytest.importorskip("pupil_apriltags")
    pytest.importorskip("moms_apriltag")
    g = _mapa_3_intersecciones()
    base = run_mission_image_loop(g, "A", "D")
    stop = run_mission_image_loop(g, "A", "D", signs={0: _id_de_tipo("stop")},
                                  stop_sign_dwell=8)
    yld = run_mission_image_loop(g, "A", "D", signs={0: _id_de_tipo("yield")})

    assert stop["reached"] and yld["reached"]
    assert stop["turns_taken"] == ["right", "straight", "left"]
    assert stop["signs_detected"] == ["stop"]             # detectada por vision en el lazo
    assert yld["signs_detected"] == ["yield"]
    assert stop["ticks"] > base["ticks"]                  # stop SIEMPRE para -> mas ticks
    assert yld["ticks"] <= base["ticks"] + 2              # yield despejado NO para


def test_duck_crossing_cede_a_pato_lateral_en_el_lazo():
    # Senal de duck-crossing REAL (detectada en el lazo) + un pato cruzando por el
    # lateral: el robot cede (para). Sin la senal, ese pato lateral no lo detiene.
    pytest.importorskip("pupil_apriltags")
    pytest.importorskip("moms_apriltag")
    g = _mapa_3_intersecciones()
    sin = run_mission_image_loop(g, "A", "D", crossing_duck_ticks=range(0, 70))
    con = run_mission_image_loop(g, "A", "D", signs={0: _id_de_tipo("duck-crossing")},
                                 crossing_duck_ticks=range(0, 70))
    assert sin["reached"] and con["reached"]
    assert sin["n_stops"] == 0                            # sin senal: el lateral no detiene
    assert con["signs_detected"] == ["yield"]            # duck-crossing -> accion yield
    assert con["n_stops"] >= 1                            # con la senal: cede al cruce
