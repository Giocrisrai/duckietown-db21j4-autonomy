"""Navegacion TILE-TO-TILE sobre el mapa de tiles oficial de Duckietown.

El mapa oficial es una grilla 2D de tiles (`tiles:` en el YAML), cada uno un tipo de
carretera con orientacion: `straight/N`, `curve_left/W`, `3way_left/S`, `4way`,
ademas de `asphalt`/`floor`/`grass` (no transitables). Este modulo:

  1) parsea la grilla,
  2) calcula las SALIDAS de cada tile (a que lados N/S/E/W abre la carretera segun
     su tipo y orientacion),
  3) conecta dos tiles vecinos solo si AMBOS abren hacia el otro (conexion mutua),
  4) planifica una ruta tile-a-tile (BFS) sobre toda la red de carretera.

Asi el robot navega el mapa COMPLETO, no un grafo abstracto de intersecciones. Las
salidas por tipo/orientacion se derivaron de los mapas de referencia (small_loop,
4way) y de la semantica de tiles de Duckietown.
"""
from pathlib import Path
from collections import deque

import yaml

# Desplazamiento de fila/columna por direccion de la brujula y su opuesta.
_DELTA = {"N": (-1, 0), "S": (1, 0), "E": (0, 1), "W": (0, -1)}
_OPP = {"N": "S", "S": "N", "E": "W", "W": "E"}

# Curva-izquierda: conecta dos lados perpendiculares (derivado del small_loop CCW).
_CURVE_LEFT = {"W": {"E", "S"}, "N": {"S", "W"}, "S": {"N", "E"}, "E": {"N", "W"}}
# Curva-derecha: espejo E<->W de la curva-izquierda.
_CURVE_RIGHT = {"W": {"W", "S"}, "N": {"S", "E"}, "S": {"N", "W"}, "E": {"N", "E"}}
# T (3-way): conecta 3 lados; falta el indicado por la orientacion (derivado del 4way).
_3WAY_MISSING = {"W": "N", "N": "E", "E": "S", "S": "W"}

_NON_DRIVABLE = {"asphalt", "floor", "grass", "empty"}


def parse_tile(s):
    """'straight/N' -> ('straight', 'N'); '4way' -> ('4way', None)."""
    s = s.strip()
    if "/" in s:
        ttype, orient = s.split("/", 1)
        return ttype.strip(), orient.strip()
    return s, None


def tile_exits(ttype, orient=None):
    """Conjunto de lados {N,S,E,W} a los que abre la carretera de un tile.

    Vacio para tiles no transitables (asphalt/floor/grass) o tipos desconocidos.
    """
    if ttype == "straight":
        return {"N", "S"} if orient in ("N", "S") else {"E", "W"}
    if ttype == "curve_left":
        return set(_CURVE_LEFT[orient])
    if ttype == "curve_right":
        return set(_CURVE_RIGHT[orient])
    if ttype in ("3way_left", "3way_right"):
        return {"N", "S", "E", "W"} - {_3WAY_MISSING[orient]}
    if ttype == "4way":
        return {"N", "S", "E", "W"}
    return set()


class TileMap:
    """Grilla de tiles con conectividad tile-a-tile calculada."""

    def __init__(self, grid):
        """grid: lista de filas; cada celda es un string de tile ('straight/N')."""
        self.grid = [list(row) for row in grid]
        self.rows = len(self.grid)
        self.cols = max((len(r) for r in self.grid), default=0)
        self._exits = {}                                   # (r,c) -> set de salidas
        for r in range(self.rows):
            for c in range(len(self.grid[r])):
                ttype, orient = parse_tile(self.grid[r][c])
                ex = tile_exits(ttype, orient)
                if ex:
                    self._exits[(r, c)] = ex

    def drivable_cells(self):
        """Celdas (r,c) de carretera (con al menos una salida)."""
        return set(self._exits)

    def neighbors(self, cell):
        """Vecinos conectados: comparten una salida mutua (ambos abren al otro)."""
        r, c = cell
        out = []
        for d in self._exits.get(cell, ()):
            dr, dc = _DELTA[d]
            nb = (r + dr, c + dc)
            if _OPP[d] in self._exits.get(nb, ()):         # el vecino tambien abre hacia aca
                out.append(nb)
        return out


def load_tile_map(path) -> TileMap:
    """Carga un mapa de tiles oficial (clave `tiles:`) desde un YAML."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return TileMap(data.get("tiles", []))


def tile_route(tmap, start, goal):
    """Ruta tile-a-tile (lista de celdas) de `start` a `goal` por BFS, o None.

    Devuelve None si start/goal no son transitables o no hay camino.
    """
    drivable = tmap.drivable_cells()
    if start not in drivable or goal not in drivable:
        return None
    if start == goal:
        return [start]
    prev = {start: None}
    q = deque([start])
    while q:
        cell = q.popleft()
        if cell == goal:
            break
        for nb in tmap.neighbors(cell):
            if nb not in prev:
                prev[nb] = cell
                q.append(nb)
    if goal not in prev:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]


# Rotacion del rumbo: izquierda (CCW) y derecha (CW).
_LEFT = {"E": "N", "N": "W", "W": "S", "S": "E"}
_RIGHT = {"E": "S", "S": "W", "W": "N", "N": "E"}


def _heading(a, b):
    """Direccion de la brujula al pasar de la celda a a la b (vecinas)."""
    dr, dc = b[0] - a[0], b[1] - a[1]
    for d, (ddr, ddc) in _DELTA.items():
        if (dr, dc) == (ddr, ddc):
            return d
    raise ValueError(f"celdas no adyacentes: {a} -> {b}")


def route_headings(route):
    """Rumbos (N/S/E/W) de cada tramo de la ruta de tiles."""
    return [_heading(a, b) for a, b in zip(route, route[1:])]


def route_turns(route):
    """Giros (straight/left/right) a ejecutar al recorrer la ruta de tiles.

    Un giro por cada cambio de rumbo entre tramos consecutivos; el vocabulario es
    el que consume el controlador (choose_turn / la FSM de giro)."""
    h = route_headings(route)
    turns = []
    for h1, h2 in zip(h, h[1:]):
        if h1 == h2:
            turns.append("straight")
        elif _LEFT[h1] == h2:
            turns.append("left")
        elif _RIGHT[h1] == h2:
            turns.append("right")
        else:
            turns.append("u-turn")             # 180 grados (no deberia darse en carretera)
    return turns


def _demo():
    """Navega un mapa de tiles de referencia completo y narra la ruta."""
    ref = Path(__file__).resolve().parents[4] / "maps" / "reference" / "4way.yaml"
    tm = load_tile_map(ref)
    start, goal = (0, 0), (4, 4)
    print(f"== Navegacion tile-to-tile sobre el mapa COMPLETO ({ref.name}) ==")
    print(f"  tiles de carretera : {len(tm.drivable_cells())}")
    route = tile_route(tm, start, goal)
    print(f"  ruta {start} -> {goal} : {len(route)} tiles")
    print(f"  celdas             : {route}")
    print(f"  giros a ejecutar   : {route_turns(route)}")
    print("  OK: planifico una ruta atravesando el mapa de tiles, con sus giros."
          if route and route[-1] == goal else "  FALLO")


if __name__ == "__main__":
    _demo()
