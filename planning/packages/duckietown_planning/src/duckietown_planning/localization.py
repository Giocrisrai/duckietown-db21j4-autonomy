"""Localizacion simple: rastrea el nodo actual contando giros ejecutados."""


class IntersectionLocalizer:
    """Localizador que rastrea la interseccion actual contando giros.

    Mantiene un registro del nodo (interseccion) actual del robot y permite
    actualizar la posicion basandose en los giros ejecutados.
    """

    def __init__(self, graph, start):
        """Inicializa el localizador con un grafo y nodo inicial.

        Args:
            graph: Instancia de MapGraph que representa las intersecciones.
            start: Identificador del nodo inicial.
        """
        self.graph = graph
        self.current = start

    def update(self, turn_taken):
        """Actualiza la posicion actual basada en el giro ejecutado.

        Busca en los vecinos del nodo actual un giro que coincida con
        el parametro turn_taken. Si lo encuentra, actualiza self.current
        al nodo destino y lo retorna. Si no encuentra coincidencia, lanza
        ValueError.

        Args:
            turn_taken: String que representa el giro ejecutado
                       (e.g., 'straight', 'left', 'right').

        Returns:
            El identificador del nodo actual despues del giro.

        Raises:
            ValueError: Si el giro no es valido desde el nodo actual.
        """
        for turn, nxt in self.graph.neighbors(self.current):
            if turn == turn_taken:
                self.current = nxt
                return self.current
        raise ValueError(f"giro '{turn_taken}' no valido desde '{self.current}'")
