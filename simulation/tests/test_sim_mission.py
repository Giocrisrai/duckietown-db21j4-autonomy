"""Test de la mision COMPLETA multi-interseccion en el lazo imagen->imagen."""
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
