"""Tests de navegacion tile-to-tile sobre el mapa de tiles oficial de Duckietown."""
from pathlib import Path

import pytest

from duckietown_planning.tile_map import (
    parse_tile, tile_exits, TileMap, load_tile_map, tile_route,
    route_headings, route_turns,
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
