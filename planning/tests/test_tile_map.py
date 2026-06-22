"""Tests de navegacion tile-to-tile sobre el mapa de tiles oficial de Duckietown."""
from pathlib import Path

import pytest

from duckietown_planning.tile_map import (
    parse_tile, tile_exits, TileMap, load_tile_map, tile_route,
    route_headings, route_turns,
    intersection_cells, intersection_graph, relative_turn,
)

_MAPS = Path(__file__).resolve().parents[1] / "maps" / "reference"


def test_parse_tile():
    assert parse_tile("straight/N") == ("straight", "N")
    assert parse_tile("4way") == ("4way", None)
    assert parse_tile(" curve_left/W ") == ("curve_left", "W")


def test_exits_recta_y_curva():
    assert tile_exits("straight", "N") == {"N", "S"}
    assert tile_exits("straight", "E") == {"E", "W"}
    assert tile_exits("curve_left", "W") == {"E", "S"}      # esquina sup-izq del small_loop
    assert tile_exits("curve_left", "E") == {"N", "W"}      # esquina inf-der
    assert tile_exits("4way") == {"N", "S", "E", "W"}
    assert tile_exits("3way_left", "W") == {"S", "E", "W"}  # T en el borde superior (sin N)
    assert tile_exits("asphalt", None) == set()             # no transitable


def test_curve_right_todas_las_orientaciones():
    # curve_right NO tiene ejemplo en los mapas de referencia salvo /N (loop_obstacles).
    # Las 4 orientaciones deben ser el espejo E<->W de curve_left (rotacion 90 CW
    # anclada en /N={E,S}). Sin esto, /E y /W quedaban mal y rompian loops CW.
    assert tile_exits("curve_right", "N") == {"E", "S"}   # validado por loop_obstacles (4,4)
    assert tile_exits("curve_right", "E") == {"S", "W"}
    assert tile_exits("curve_right", "S") == {"N", "W"}
    assert tile_exits("curve_right", "W") == {"E", "N"}
    # Consistencia rotacional: cada orientacion = la anterior rotada 90 CW.
    rot = {"N": "E", "E": "S", "S": "W", "W": "N"}
    for o in "NESW":
        rotada = {rot[s] for s in tile_exits("curve_right", o)}
        assert rotada == tile_exits("curve_right", rot[o])


def test_loop_clockwise_con_curve_right_cierra():
    # Un loop 3x3 en sentido HORARIO usa curve_right en las 4 esquinas; debe cerrar
    # como ciclo (cada celda de carretera con exactamente 2 vecinos).
    tm = TileMap([
        ["curve_right/N", "straight/W", "curve_right/E"],
        ["straight/S", "asphalt", "straight/N"],
        ["curve_right/W", "straight/E", "curve_right/S"],
    ])
    assert len(tm.drivable_cells()) == 8
    for cell in tm.drivable_cells():
        assert len(tm.neighbors(cell)) == 2


def test_small_loop_es_un_ciclo():
    tm = load_tile_map(_MAPS / "small_loop.yaml")
    drivable = tm.drivable_cells()
    assert len(drivable) == 8                               # 8 tiles de carretera (centro=asphalt)
    # Es un unico loop: cada tile de carretera conecta con exactamente 2 vecinos.
    for cell in drivable:
        assert len(tm.neighbors(cell)) == 2


def test_ruta_tile_a_tile_en_el_loop():
    tm = load_tile_map(_MAPS / "small_loop.yaml")
    ruta = tile_route(tm, (0, 0), (2, 2))                   # esquina a esquina opuesta
    assert ruta is not None
    assert ruta[0] == (0, 0) and ruta[-1] == (2, 2)
    # Celdas consecutivas son adyacentes en la grilla y estan conectadas.
    for a, b in zip(ruta, ruta[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
        assert b in tm.neighbors(a)


def test_ruta_a_celda_no_transitable_es_none():
    tm = load_tile_map(_MAPS / "small_loop.yaml")
    assert tile_route(tm, (0, 0), (1, 1)) is None           # (1,1) es asphalt


def test_giros_de_la_ruta_de_tiles():
    # Recorrido CCW completo del small_loop: las 4 esquinas (curve_left) son giros a
    # la IZQUIERDA y los tramos rectos 'straight'. (El sentido importa: una curva
    # recorrida al reves seria un giro a la derecha.)
    ccw = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2), (0, 1), (0, 0)]
    assert route_headings(ccw) == ["S", "S", "E", "E", "N", "N", "W", "W"]
    turns = route_turns(ccw)
    assert turns == ["straight", "left", "straight", "left", "straight", "left", "straight"]
    # El mismo loop al reves (CW) -> giros a la derecha.
    assert "right" in route_turns(ccw[::-1]) and "left" not in route_turns(ccw[::-1])


def test_ruta_bfs_da_giros_validos():
    # Cualquier ruta BFS produce giros validos (sin 'u-turn') celda a celda.
    tm = load_tile_map(_MAPS / "small_loop.yaml")
    ruta = tile_route(tm, (0, 0), (2, 2))
    turns = route_turns(ruta)
    assert all(t in ("straight", "left", "right") for t in turns)
    assert len(turns) == len(ruta) - 2


def test_4way_carga_y_es_navegable():
    tm = load_tile_map(_MAPS / "4way.yaml")
    # La interseccion 4way del centro conecta sus 4 vecinos.
    assert len(tm.neighbors((2, 2))) == 4
    # Hay ruta entre esquinas opuestas atravesando intersecciones.
    ruta = tile_route(tm, (0, 0), (4, 4))
    assert ruta is not None and ruta[0] == (0, 0) and ruta[-1] == (4, 4)


def test_giro_relativo():
    assert relative_turn("N", "N") == "straight"
    assert relative_turn("E", "N") == "left"               # E rota CCW -> N
    assert relative_turn("E", "S") == "right"              # E rota CW -> S


def test_grafo_de_intersecciones_desde_tiles():
    tm = load_tile_map(_MAPS / "4way.yaml")
    nodos = intersection_cells(tm)
    assert set(nodos) == {(0, 2), (2, 0), (2, 2), (2, 4), (4, 2)}   # 1 cuatro-vias + 4 T
    g = intersection_graph(tm)
    # El 4way del centro enlaza con las 4 intersecciones T, siguiendo cada carretera.
    destinos_4way = {to for _exit, to, _arr in g[(2, 2)]}
    assert destinos_4way == {(0, 2), (2, 0), (2, 4), (4, 2)}
    # Todas las aristas terminan en una interseccion (no en una celda cualquiera).
    for nodo, aristas in g.items():
        for _exit, to, _arr in aristas:
            assert to in set(nodos)
    # Una arista que sale por una curva llega con un rumbo distinto al de salida:
    # de (0,2) saliendo al ESTE se llega a (2,4) llegando hacia el SUR (la via curva).
    este = [(to, arr) for ex, to, arr in g[(0, 2)] if ex == "E"]
    assert este == [((2, 4), "S")]


def test_obstaculos_del_mapa_sobre_la_pista():
    from duckietown_planning.tile_map import load_map_objects, track_obstacles
    objs = load_map_objects(_MAPS / "loop_obstacles.yaml")
    assert {o["kind"] for o in objs} & {"duckie", "cone"}   # el mapa trae obstaculos
    tm = load_tile_map(_MAPS / "loop_obstacles.yaml")
    obstaculos = track_obstacles(objs, tm)
    # Hay varios obstaculos SOBRE la pista (patos/conos), y todos en tiles de pista.
    assert len(obstaculos) >= 3
    kinds = {o["kind"] for o, _ in obstaculos}
    assert "duckie" in kinds and "cone" in kinds
    for _o, cell in obstaculos:
        assert cell in tm.drivable_cells()
    # Las barreras de borde (fuera de pista) quedan excluidas.
    assert all(o["kind"] != "barrier" for o, _ in obstaculos)
