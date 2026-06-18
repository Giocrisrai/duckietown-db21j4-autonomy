"""Busqueda BFS del camino mas corto -> secuencia de giros."""
from collections import deque


def plan_route(graph, start, goal):
    """
    Encuentra el camino más corto entre dos nodos usando BFS.

    Devuelve la secuencia de giros necesarios para ir de start a goal.
    Si start == goal, devuelve lista vacía.
    Si no existe ruta, devuelve None.

    Args:
        graph: MapGraph con el mapa de intersecciones
        start: nombre del nodo inicial
        goal: nombre del nodo destino

    Returns:
        list[str] | None: secuencia de giros, o None si no hay ruta
    """
    if start == goal:
        return []

    # prev mapea nodo -> (nodo_previo, giro)
    prev = {start: None}
    q = deque([start])

    while q:
        cur = q.popleft()
        for turn, nxt in graph.neighbors(cur):
            if nxt not in prev:
                prev[nxt] = (cur, turn)
                if nxt == goal:
                    return _reconstruct(prev, goal)
                q.append(nxt)

    return None


def _reconstruct(prev, goal):
    """
    Reconstruye la secuencia de giros desde goal hacia atrás.

    Args:
        prev: mapeo de nodo -> (nodo_previo, giro)
        goal: nodo destino

    Returns:
        list[str]: secuencia de giros en orden correcto
    """
    turns = []
    node = goal
    while prev[node] is not None:
        p, turn = prev[node]
        turns.append(turn)
        node = p
    turns.reverse()
    return turns
