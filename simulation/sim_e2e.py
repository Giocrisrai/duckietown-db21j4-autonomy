"""Smoke-test END-TO-END que compone los modulos reales del stack (sin robot).

Encadena, en una mision completa, los modulos del proyecto:
  percepcion de senales (AprilTags) -> planificacion (ruta global + reactiva)
  -> seguridad (freno ToF) -> control (PID seguir-linea) -> calibracion + odometria.

Alcance: es un test de INTEGRACION de las piezas (que encajan en firma y cooperan
en una mision), con un modelo de mundo de juguete (cada segmento re-arranca el
seguir-linea desde un d0 fijo; no propaga la pose entre intersecciones). Para el
lazo de percepcion->control honesto (imagen->imagen) ver `sim_closed_loop` y
`sim_mission`.
"""
from duckietown_planning.route_search import plan_route
from duckietown_planning.intersection_policy import choose_turn
from duckietown_planning.localization import IntersectionLocalizer
from duckietown_planning.safety import should_brake_range
from duckietown_planning.odometry import update_pose
from duckietown_planning.calibration import apply_trim_gain
from object_detection.apriltag_policy import select_action
from pid import PID
from lane_sim import simulate_lane


def run_mission(graph, start, goal, events=None, max_intersecciones=50):
    """Ejecuta una mision end-to-end y devuelve un trace con el resultado.

    events: dict {nodo: {"signs":[tag_ids], "obstacle_m":float, "lane_d0":float}}
    Devuelve dict: reached, final_node, log, max_offset, stops, pose.
    """
    events = events or {}
    route = plan_route(graph, start, goal)
    if route is None:
        return {"reached": False, "reason": "sin ruta", "log": [], "max_offset": 0.0}

    loc = IntersectionLocalizer(graph, start)
    log, pose, max_offset, stops = [], (0.0, 0.0, 0.0), 0.0, 0

    for _ in range(max_intersecciones):
        if loc.current == goal:
            break
        ev = events.get(loc.current, {})

        # 1) CONTROL: seguir-linea hasta la interseccion (PID + dinamica lateral).
        pid = PID(6.0, 0.0, 4.0, out_limits=(-5.0, 5.0))
        _, traj = simulate_lane(pid, d0=ev.get("lane_d0", 0.15))
        max_offset = max(max_offset, max(abs(x) for x in traj))

        # 2) SEGURIDAD: obstaculo cercano (ToF) -> frenar y abortar el avance.
        obst = ev.get("obstacle_m")
        if obst is not None and should_brake_range(obst, 0.03, 1.2, 0.15):
            log.append((loc.current, "frenar"))
            return {"reached": False, "reason": f"obstaculo en {loc.current}",
                    "log": log, "max_offset": max_offset, "stops": stops, "pose": pose}

        # 3) PERCEPCION DE SENALES: stop/yield mandan (se registra la detencion).
        if select_action(ev.get("signs", [])) == "stop":
            stops += 1
            log.append((loc.current, "stop_senal"))

        # 4) PLANIFICACION: elegir giro (global con respaldo reactivo).
        disponibles = graph.turns_from(loc.current)
        if not disponibles:
            break
        planeado = route[0] if route else None
        giro = choose_turn(disponibles, planned=planeado)
        log.append((loc.current, f"giro:{giro}"))
        if route and giro == route[0]:
            route.pop(0)

        # 5) ACTUACION: calibracion de ruedas + odometria.
        vl, vr = apply_trim_gain(0.3, 0.3, trim=0.0, gain=1.0)
        pose = update_pose(*pose, d_left=vl * 0.05, d_right=vr * 0.05)
        loc.update(giro)

    return {"reached": loc.current == goal, "final_node": loc.current,
            "log": log, "max_offset": max_offset, "stops": stops, "pose": pose}
