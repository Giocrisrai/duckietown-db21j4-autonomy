"""Grafo dirigido de intersecciones para el planeador global."""
from pathlib import Path
import yaml


class MapGraph:
    """Grafo de intersecciones representado como lista de adyacencia.

    Nodos representan intersecciones, aristas representan giros posibles
    entre intersecciones.

    Invariante del formato de mapa: los giros (turn) que salen de un mismo
    nodo deben ser UNICOS. Si un nodo tuviera dos aristas con el mismo turn,
    la localizacion y el seguimiento de ruta tomarian la primera coincidencia
    y podrian divergir del plan.
    """

    def __init__(self, nodes, edges):
        """Inicializa el grafo con nodos y aristas.

        Args:
            nodes: Lista de identificadores de nodos (intersecciones).
            edges: Lista de diccionarios con {from, to, turn}.
        """
        self.nodes = list(nodes)
        self.adj = {n: [] for n in self.nodes}
        for e in edges:
            self.adj[e["from"]].append((e["turn"], e["to"]))

    def neighbors(self, node):
        """Retorna los vecinos de un nodo como lista de (turn, to_node).

        Args:
            node: Identificador del nodo.

        Returns:
            Lista de tuplas (turn, to_node), o [] si el nodo no existe.
        """
        return self.adj.get(node, [])

    def turns_from(self, node):
        """Retorna los giros posibles desde un nodo.

        Args:
            node: Identificador del nodo.

        Returns:
            Lista de strings de giros (straight, left, right), o [] si no existe.
        """
        return [turn for turn, _ in self.neighbors(node)]


def load_map(path) -> MapGraph:
    """Carga un grafo de intersecciones desde un archivo YAML.

    Args:
        path: Ruta (str o Path) al archivo YAML con esquema:
              nodes: [...]
              edges: [{from, to, turn}, ...]

    Returns:
        MapGraph construido a partir del archivo.
    """
    data = yaml.safe_load(Path(path).read_text())
    return MapGraph(data["nodes"], data["edges"])
