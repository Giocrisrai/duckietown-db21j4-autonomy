"""Simulacion de lazo cerrado del cerebro de planificacion (sin ROS).

Recorre intersecciones: en cada una el planeador global sugiere un giro y el
navegador reactivo lo elige (o decide solo si no hay plan). Valida la autonomia
de planificacion sin robot ni simulador pesado.
"""
from duckietown_planning.route_search import plan_route
from duckietown_planning.localization import IntersectionLocalizer
from duckietown_planning.intersection_policy import choose_turn


def simulate_route(graph, start, goal, max_steps: int = 100):
    """Simula el lazo planeador+navegador. Devuelve (nodo_final, giros_ejecutados).

    Parámetros:
        graph: Grafo de intersecciones (MapGraph).
        start: Nodo inicial.
        goal: Nodo destino.
        max_steps: Número máximo de iteraciones (default 100).

    Retorna:
        (nodo_final, lista_giros): Nodo alcanzado y lista de giros ejecutados.
    """
    # Planifica la ruta global desde start hasta goal
    route = plan_route(graph, start, goal)
    if route is None:
        return start, []                       # sin ruta posible

    # Inicializa localizador en el nodo de inicio
    loc = IntersectionLocalizer(graph, start)
    ejecutados = []

    # Itera hasta max_steps o hasta alcanzar goal
    for _ in range(max_steps):
        if loc.current == goal:
            break

        # Obtiene giros disponibles desde la intersección actual
        disponibles = graph.turns_from(loc.current)
        if not disponibles:
            break                              # sin salida (callejon)

        # Planeador sugiere el próximo giro de la ruta
        planeado = route[0] if route else None

        # Navegador elige giro (considera sugerencia del planeador)
        giro = choose_turn(disponibles, planned=planeado)

        # Ejecuta el giro
        ejecutados.append(giro)

        # Si se siguió el plan, avanza en la ruta
        if route and giro == route[0]:
            route.pop(0)

        # Actualiza localización
        loc.update(giro)

    return loc.current, ejecutados
