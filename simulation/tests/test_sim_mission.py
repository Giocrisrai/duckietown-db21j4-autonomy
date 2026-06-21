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


def test_apriltag_de_stop_detectado_en_el_lazo_impone_parada():
    # AprilTags REALES pintados en las senales y detectados con pupil-apriltags
    # SOBRE el frame renderizado: el cerebro ve 'stop'/'yield' y hace la parada
    # obligatoria, alargando la mision frente a la version sin senales.
    pytest.importorskip("pupil_apriltags")
    pytest.importorskip("moms_apriltag")
    g = _mapa_3_intersecciones()
    base = run_mission_image_loop(g, "A", "D")
    con = run_mission_image_loop(g, "A", "D",
                                 signs={0: _id_de_tipo("stop"), 2: _id_de_tipo("yield")},
                                 stop_sign_dwell=8)
    assert con["reached"] is True
    assert con["turns_taken"] == ["right", "straight", "left"]
    assert con["signs_detected"] == ["stop", "yield"]     # detectadas por vision en el lazo
    assert con["ticks"] > base["ticks"]                   # las paradas obligatorias alargan la mision
