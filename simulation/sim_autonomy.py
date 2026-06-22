"""Simulacion AUTONOMA de lazo cerrado de PLANIFICACION + FSM + control (sin robot).

Maneja una mision completa a traves del cerebro integrado
(`duckietown_planning.autonomy.AutonomousController`): seguir carril con PID,
detectar la linea de stop, detenerse en la interseccion, elegir el giro segun la
ruta global, ejecutar el giro, reanudar, y frenar ante obstaculos (pato/coche)
por la fusion vision+ToF.

Alcance: aqui el error de carril se le ENTREGA al cerebro como escalar (`lane_error`,
del estado interno del modelo), no se estima de una imagen; valida la cooperacion
de planificacion + FSM + control, no la percepcion de carril. El lazo HONESTO de
percepcion->control (imagen->imagen, sin escalares privilegiados) esta en
`sim_closed_loop` y `sim_mission`.

Modelo de mundo: integra un modelo lateral (d, phi) empujado por el `omega` del
controlador, avanza distancia por segmento, y dispara los eventos (stop_line,
at_stop_line, turn_done, obstaculo) segun el progreso.
"""
import math

from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.fsm import (
    LANE_FOLLOWING, APPROACHING_INTERSECTION, TURNING, STOPPED,
)

IMG_W, IMG_H = 640, 480
# Pato en el corredor central y bajo en la imagen -> fusion lo marca STOP.
DUCK = {"class_name": "duckie", "bbox": (280, 300, 360, 460), "confidence": 0.9}


def run_autonomy(graph, start, goal, obstacle_ticks=(), segment_len=1.0,
                 approach_frac=0.3, turn_ticks=3, d0=0.12, dt=0.05, max_ticks=6000,
                 homography=None):
    """Corre una mision autonoma de lazo cerrado y devuelve un trace.

    obstacle_ticks: iterable de indices de tick en los que hay un pato delante.
    homography: calibracion extrinseca opcional -> el cerebro decide los obstaculos
    por distancia metrica real al suelo en vez del proxy del bbox.
    Devuelve dict: reached, ticks, states (set), n_stops, turns_taken,
    max_offset, final_node.
    """
    obstacle_ticks = set(obstacle_ticks)
    ctrl = AutonomousController(graph, start, goal, img_size=(IMG_W, IMG_H),
                               homography=homography)

    d, phi, dist = d0, 0.0, 0.0
    turn_counter, pending_turn_done, last_turn = 0, False, None
    prev_state, n_stops = None, 0
    states_seen, turns_taken = set(), []
    max_offset = abs(d0)
    reached = False
    ticks = 0

    for t in range(max_ticks):
        ticks = t + 1
        obstacle = t in obstacle_ticks
        obs = {
            "lane_error": -d,                 # convencion: omega>0 corrige d<0 (lazo estable)
            "detections": [DUCK] if obstacle else [],
            "tof": 0.10 if obstacle else None,
            "stop_line": dist >= segment_len * (1.0 - approach_frac),
            "at_stop_line": dist >= segment_len,
            "turn_done": pending_turn_done,
            "img_w": IMG_W, "img_h": IMG_H,
        }
        cmd = ctrl.step(obs, dt)
        st = cmd["state"]
        states_seen.add(st)

        if st == STOPPED and prev_state != STOPPED:
            n_stops += 1
        # Giro consumado (TURNING -> LANE_FOLLOWING): registrar y reiniciar el segmento.
        if prev_state == TURNING and st == LANE_FOLLOWING:
            turns_taken.append(last_turn)
            dist, turn_counter, pending_turn_done = 0.0, 0, False

        # Modelo lateral: el omega del controlador corrige el error de carril.
        phi = max(-1.2, min(1.2, phi + cmd["omega"] * dt))
        d += cmd["v"] * math.sin(phi) * dt
        max_offset = max(max_offset, abs(d))

        # Avance de distancia solo mientras conduce hacia la interseccion.
        if st in (LANE_FOLLOWING, APPROACHING_INTERSECTION):
            dist += cmd["v"] * dt
        # Progreso de la maniobra de giro.
        if st == TURNING:
            turn_counter += 1
            last_turn = cmd["turn"]
            pending_turn_done = turn_counter >= turn_ticks

        prev_state = st
        if cmd["reached"]:
            reached = True
            break

    return {
        "reached": reached, "ticks": ticks, "states": states_seen,
        "n_stops": n_stops, "turns_taken": turns_taken,
        "max_offset": max_offset,
        "final_node": ctrl.loc.current if ctrl.loc else None,
    }


def _demo():
    """Corre una mision de ejemplo con un obstaculo y la narra por consola."""
    from duckietown_planning.map_graph import MapGraph
    g = MapGraph(
        ["A", "B", "C", "D"],
        [
            {"from": "A", "to": "B", "turn": "right"},
            {"from": "B", "to": "C", "turn": "straight"},
            {"from": "C", "to": "D", "turn": "left"},
        ],
    )
    print("== Mision autonoma end-to-end: A -> D (con un pato en el camino) ==")
    r = run_autonomy(g, "A", "D", obstacle_ticks=range(8, 24))
    print(f"  llego al destino : {r['reached']}  (nodo final: {r['final_node']})")
    print(f"  giros ejecutados : {r['turns_taken']}")
    print(f"  paradas seguridad: {r['n_stops']}  (freno ante el pato)")
    print(f"  error lateral max: {r['max_offset']:.3f} m  (PID mantiene el carril)")
    print(f"  ticks            : {r['ticks']}")
    print("  OK: el cerebro siguio carril, paro por el obstaculo, respeto la ruta "
          "en las intersecciones y llego." if r["reached"] else "  FALLO")

    # Misma mision, ahora con calibracion extrinseca: el cerebro decide por
    # distancia METRICA real al suelo (no por el proxy del bbox).
    from object_detection.ground_projection import estimate_homography
    H = estimate_homography(
        [(180.0, 400.0), (460.0, 400.0), (260.0, 250.0), (380.0, 250.0)],
        [(0.10, 0.10), (0.10, -0.10), (0.30, 0.10), (0.30, -0.10)],
    )
    print("\n== Misma mision con homografia (distancia metrica) ==")
    rm = run_autonomy(g, "A", "D", obstacle_ticks=range(8, 24), homography=H)
    print(f"  llego: {rm['reached']}  paradas: {rm['n_stops']}  "
          f"(el cerebro freno por distancia metrica real al pato, no por el proxy)")


if __name__ == "__main__":
    _demo()
