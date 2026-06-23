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
# Curva-derecha: espejo E<->W de la curva-izquierda (= rotacion 90 CW anclada en
# /N={E,S}, validado por loop_obstacles). curve_left/X y curve_right/X NO abren los
# mismos lados: la orientacion /X significa cosas distintas para izquierda y derecha.
_CURVE_RIGHT = {"N": {"E", "S"}, "E": {"S", "W"}, "S": {"N", "W"}, "W": {"E", "N"}}
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
        """Vecinos conectados: comparten una salida mutua (ambos abren al otro).

        Recorre las direcciones en orden fijo (N,S,E,W) para que la navegacion sea
        DETERMINISTA: las salidas son un set de strings y su iteracion variaria entre
        procesos (hash randomization), dando rutas BFS distintas en cada ejecucion."""
        r, c = cell
        out = []
        for d in ("N", "S", "E", "W"):
            if d not in self._exits.get(cell, ()):
                continue
            dr, dc = _DELTA[d]
            nb = (r + dr, c + dc)
            if _OPP[d] in self._exits.get(nb, ()):         # el vecino tambien abre hacia aca
                out.append(nb)
        return out


def load_tile_map(path) -> TileMap:
    """Carga un mapa de tiles oficial (clave `tiles:`) desde un YAML."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return TileMap(data.get("tiles", []))


def load_map_objects(path):
    """Lista de objetos del mapa (clave `objects:`): patos, conos, etc. sobre la pista."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return list(data.get("objects") or [])


def object_cell(obj):
    """Celda (fila, columna) del tile donde esta un objeto. `pos` viene en unidades
    de tile [columna, fila]; la celda es la parte entera."""
    col, row = obj["pos"][0], obj["pos"][1]
    return (int(row), int(col))


def track_obstacles(objects, tmap, kinds=("duckie", "cone", "duckiebot")):
    """Obstaculos que estan SOBRE la pista (en un tile transitable), a esquivar.

    Filtra los objetos del mapa por tipo (pato/cono/duckiebot por defecto) y descarta
    los que caen fuera de la carretera (barreras de borde, decoracion). Devuelve una
    lista de (objeto, celda)."""
    out = []
    for o in objects:
        if o.get("kind") in kinds:
            cell = object_cell(o)
            if cell in tmap.drivable_cells():
                out.append((o, cell))
    return out


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


_INTERSECTION_TYPES = ("3way_left", "3way_right", "4way")


def relative_turn(entry_heading, exit_dir):
    """Giro relativo (straight/left/right/u-turn) al entrar con `entry_heading` y
    salir hacia `exit_dir` (ambos rumbos de la brujula)."""
    if entry_heading == exit_dir:
        return "straight"
    if _LEFT[entry_heading] == exit_dir:
        return "left"
    if _RIGHT[entry_heading] == exit_dir:
        return "right"
    return "u-turn"


def intersection_cells(tmap):
    """Celdas (r,c) que son intersecciones (3way / 4way)."""
    out = []
    for (r, c) in tmap.drivable_cells():
        ttype, _ = parse_tile(tmap.grid[r][c])
        if ttype in _INTERSECTION_TYPES:
            out.append((r, c))
    return out


def _follow_to_intersection(tmap, intersection, direction, intersections):
    """Sigue la carretera desde `intersection` saliendo hacia `direction` por los
    tramos (rectas/curvas) hasta la siguiente interseccion. Devuelve (celda_destino,
    rumbo_de_llegada) o None si la salida no lleva a ninguna interseccion."""
    dr, dc = _DELTA[direction]
    cur = (intersection[0] + dr, intersection[1] + dc)
    came_from = _OPP[direction]
    heading = direction
    seen = {intersection}
    while True:
        ex = tmap._exits.get(cur)
        if not ex or came_from not in ex:
            return None                                # se salio de la carretera
        if cur in intersections:
            return (cur, heading)
        cont = [d for d in ex if d != came_from]       # un tramo simple tiene 1 continuacion
        if len(cont) != 1 or cur in seen:
            return None
        seen.add(cur)
        heading = cont[0]
        dr, dc = _DELTA[heading]
        cur = (cur[0] + dr, cur[1] + dc)
        came_from = _OPP[heading]


def intersection_graph(tmap):
    """Deriva el grafo de intersecciones desde la grilla de tiles.

    Devuelve {celda_interseccion: [(salida, celda_destino, rumbo_de_llegada), ...]}.
    Las aristas son DIRECCIONALES con el rumbo de salida y el de llegada porque el
    giro (left/right/straight) en una interseccion depende de por donde se entra
    (relative_turn(rumbo_de_llegada_previo, salida))."""
    intersections = set(intersection_cells(tmap))
    graph = {}
    for cell in intersections:
        edges = []
        for d in sorted(tmap._exits.get(cell, ())):
            res = _follow_to_intersection(tmap, cell, d, intersections)
            if res is not None:
                edges.append((d, res[0], res[1]))
        graph[cell] = edges
    return graph


def loop_cells(tmap, start=None):
    """Orden de las celdas de un mapa-LOOP (sin intersecciones): sigue los vecinos
    desde `start` hasta cerrar el ciclo. Devuelve la lista de celdas en orden, o None
    si el mapa no es un loop simple (alguna celda no tiene exactamente 2 vecinos)."""
    drivable = tmap.drivable_cells()
    if not drivable or any(len(tmap.neighbors(c)) != 2 for c in drivable):
        return None
    start = start if start is not None else min(drivable)
    order, prev, cur = [start], None, start
    while True:
        nxt = next((n for n in tmap.neighbors(cur) if n != prev), None)
        if nxt is None or nxt == start:
            break
        order.append(nxt)
        prev, cur = cur, nxt
    return order if len(order) == len(drivable) else None


def _path_intersection_turns(tmap, path):
    """Giros en las intersecciones al recorrer una ruta de celdas (rectas/curvas se
    siguen solas; aqui solo las decisiones de interseccion)."""
    if path is None or len(path) < 3:
        return []
    inters = set(intersection_cells(tmap))
    headings = route_headings(path)
    return [relative_turn(headings[i - 1], headings[i])
            for i in range(1, len(path) - 1) if path[i] in inters]


def tile_route_turns(tmap, start, goal):
    """Giros (straight/left/right) a ejecutar EN LAS INTERSECCIONES al ir de la celda
    `start` a `goal` por la ruta de tiles."""
    return _path_intersection_turns(tmap, tile_route(tmap, start, goal))


def tile_tour_path(tmap, waypoints):
    """Concatena las rutas de tiles entre `waypoints` en un solo recorrido de celdas
    (sin duplicar las celdas de empalme). None si algun tramo no tiene ruta."""
    full = []
    for a, b in zip(waypoints, waypoints[1:]):
        leg = tile_route(tmap, a, b)
        if leg is None:
            return None
        full += leg if not full else leg[1:]
    return full


def tile_tour_graph(tmap, waypoints):
    """Construye un MapGraph lineal de un TOUR que pasa por `waypoints` (p. ej. todas
    las intersecciones). Devuelve (graph, inicio, fin, intersecciones_visitadas)."""
    from .map_graph import MapGraph
    path = tile_tour_path(tmap, waypoints)
    if path is None:
        return None
    turns = _path_intersection_turns(tmap, path)
    nodes = [f"n{i}" for i in range(len(turns) + 1)]
    edges = [{"from": nodes[i], "to": nodes[i + 1], "turn": t} for i, t in enumerate(turns)]
    inters = set(intersection_cells(tmap))
    visited = [c for c in path if c in inters]
    return MapGraph(nodes, edges), nodes[0], nodes[-1], visited


def tile_mission_graph(tmap, start, goal):
    """Construye un MapGraph lineal de la ruta de tiles `start`->`goal` para que el
    cerebro la conduzca: nodos = intersecciones de la ruta en orden, aristas = el
    giro en cada una. Devuelve (graph, nodo_inicial, nodo_destino)."""
    from .map_graph import MapGraph
    turns = tile_route_turns(tmap, start, goal)
    nodes = [f"n{i}" for i in range(len(turns) + 1)]
    edges = [{"from": nodes[i], "to": nodes[i + 1], "turn": t} for i, t in enumerate(turns)]
    return MapGraph(nodes, edges), nodes[0], nodes[-1]


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

    g = intersection_graph(tm)
    print(f"  intersecciones     : {len(g)}  (3way/4way derivadas de los tiles)")
    aristas = g[(2, 2)]
    print("  desde el 4way (2,2): " + ", ".join(f"{ex}->{to}" for ex, to, _ in aristas))
    # Ejemplo de giro: llegando al 4way hacia el SUR y saliendo al ESTE -> izquierda.
    print(f"  giro ejemplo       : llego al 4way hacia S, salgo al E -> "
          f"{relative_turn('S', 'E')}")
    print("  OK: planifico ruta de tiles (con giros) y derivo el grafo de intersecciones."
          if route and route[-1] == goal else "  FALLO")


if __name__ == "__main__":
    _demo()
