"""Mision COMPLETA multi-interseccion en el lazo IMAGEN->IMAGEN.

Une el seguir-carril por vision (sim_closed_loop) con la navegacion por la ruta
global (sim_autonomy): el mundo renderiza la camara de cada tramo (carril recto o
curvo + la franja ROJA de stop al final), y el cerebro conduce TODA la mision solo
con pixeles: sigue el carril, ve y mide la franja de stop, se detiene en la
interseccion, elige el giro segun la ruta, lo ejecuta y encadena el siguiente
tramo, hasta llegar al destino.

Es la validacion end-to-end mas completa: percepcion (lineas + franja + homografia)
+ planificacion (ruta + FSM) + control (PID), todo cerrado sobre imagenes reales
renderizadas, sin escalares privilegiados de pose.
"""
import numpy as np

from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.route_search import plan_route
from duckietown_planning.fsm import (
    LANE_FOLLOWING, APPROACHING_INTERSECTION, TURNING, STOPPED,
)
from object_detection.stop_line import detect_stop_line
from sim_closed_loop import render, _paint, _perturb, _H, IMG_W, IMG_H
from pid import PID
import numpy as _np

# Distancia (m) a la que se empieza a pintar la franja (entra en rango detectable).
STOP_LINE_PAINT_M = 0.30
SEG_CURVATURES = (0.0, 1.2, -1.2, 0.8)   # curvatura por tramo (cicla): recto/curvos
# Pato en el corredor central y bajo en la imagen -> fusion lo situa cerca (STOP).
DUCK = {"class_name": "duckie", "bbox": (280, 300, 360, 460), "confidence": 0.9}


def paint_stop_line(img, dist, half_w=0.13):
    """Pinta la franja roja perpendicular en el suelo a distancia `dist` (m)."""
    for y in np.linspace(-half_w, half_w, 40):
        _paint(img, dist, y, (0, 0, 255), blob=9)


def run_mission_image_loop(graph, start, goal, seg_len=1.0, off0=0.03,
                           turn_ticks=3, dt=0.05, base_speed=0.3, max_ticks=4000,
                           noise_sigma=0.0, brightness=0, occlusion=0.0, seed=0,
                           obstacle_ticks=()):
    """Corre la mision completa en lazo imagen->imagen y devuelve un trace.

    noise_sigma/brightness/occlusion: ruido realista de camara aplicado a CADA
    frame antes de la percepcion (gaussiano, sesgo de brillo, banda ocluida).
    obstacle_ticks: indices de tick con un pato delante (la fusion vision+ToF lo
    detecta y el robot frena; al despejarse reanuda y completa la mision).

    Devuelve dict: reached, final_node, turns_taken, n_intersections, max_off,
    n_stops, ticks.
    """
    obstacle_ticks = set(obstacle_ticks)
    rng = _np.random.default_rng(seed)
    route = plan_route(graph, start, goal)
    n_seg = len(route) if route else 0
    # Control PROPORCIONAL puro sobre d + k_heading*phi (el rumbo aporta el
    # amortiguamiento), igual que el lazo de seguir-carril validado: el PID por
    # defecto (kd alto) oscilaria con el ruido de phi y perderia el carril en curva.
    ctrl = AutonomousController(graph, start, goal, homography=_H,
                               img_size=(IMG_W, IMG_H), base_speed=base_speed,
                               pid=PID(3.0, 0.0, 0.0, out_limits=(-5.0, 5.0)),
                               k_heading=0.5)

    seg_idx = 0
    s, off, dpsi = 0.0, float(off0), 0.0
    turn_count, pending_turn_done = 0, False
    prev_state, turns_taken, max_off = None, [], abs(off0)
    n_stops = 0
    reached, ticks = False, 0

    for t in range(max_ticks):
        ticks = t + 1
        curvature = SEG_CURVATURES[seg_idx % len(SEG_CURVATURES)] if n_seg else 0.0

        # 1) MUNDO: render del tramo + franja de stop si esta cerca el final.
        img = render(off, dpsi, curvature)
        dist_to_line = seg_len - s
        if 0.0 < dist_to_line <= STOP_LINE_PAINT_M:
            paint_stop_line(img, dist_to_line)
        if noise_sigma or brightness or occlusion:
            img = _perturb(img, rng, noise_sigma, brightness, occlusion)

        # 2) PERCEPCION de la franja sobre la imagen real (con homografia -> metros).
        sl = detect_stop_line(img, homography=_H)

        # 3) CEREBRO: conduce con la imagen (lane_error interno) + eventos de franja
        #    + obstaculo (fusion vision+ToF; distancia metrica con la homografia).
        obstacle = t in obstacle_ticks
        cmd = ctrl.step({
            "image": img, "img_w": IMG_W, "img_h": IMG_H,
            "stop_line": sl["detected"], "stop_line_distance_m": sl["distance_m"],
            "turn_done": pending_turn_done,
            "detections": [DUCK] if obstacle else [],
            "tof": 0.10 if obstacle else None,
        }, dt)
        st = cmd["state"]
        if st == STOPPED and prev_state != STOPPED:
            n_stops += 1

        # 4) Giro consumado: pasar al siguiente tramo (nuevo carril, pose reiniciada).
        if prev_state == TURNING and st == LANE_FOLLOWING:
            turns_taken.append(ctrl_last_turn)
            seg_idx += 1
            s, off, dpsi = 0.0, float(off0), 0.0
            turn_count, pending_turn_done = 0, False

        # 5) MODELO cinematico (marco de Frenet) solo mientras conduce hacia la linea.
        if st in (LANE_FOLLOWING, APPROACHING_INTERSECTION):
            v, omega = cmd["v"], cmd["omega"]
            denom = max(1.0 - off * curvature, 1e-3)
            sdot = v * np.cos(dpsi) / denom
            s += sdot * dt
            off += v * np.sin(dpsi) * dt
            dpsi += (omega - curvature * sdot) * dt
            max_off = max(max_off, abs(off))
        elif st == TURNING:
            turn_count += 1
            ctrl_last_turn = cmd["turn"]
            pending_turn_done = turn_count >= turn_ticks

        prev_state = st
        if cmd["reached"]:
            reached = True
            break

    return {"reached": reached, "final_node": ctrl.loc.current if ctrl.loc else None,
            "turns_taken": turns_taken, "n_intersections": n_seg,
            "max_off": max_off, "n_stops": n_stops, "ticks": ticks}


def _demo():
    from duckietown_planning.map_graph import MapGraph
    g = MapGraph(
        ["A", "B", "C", "D"],
        [
            {"from": "A", "to": "B", "turn": "right"},
            {"from": "B", "to": "C", "turn": "straight"},
            {"from": "C", "to": "D", "turn": "left"},
        ],
    )
    print("== Mision COMPLETA multi-interseccion en lazo IMAGEN->IMAGEN: A -> D ==")
    r = run_mission_image_loop(g, "A", "D")
    print(f"  intersecciones    : {r['n_intersections']}  (tramos con franja de stop)")
    print(f"  giros ejecutados  : {r['turns_taken']}")
    print(f"  error lateral max : {r['max_off']:.3f} m  (siguio el carril por vision)")
    print(f"  llego al destino  : {r['reached']}  (nodo final: {r['final_node']})  en {r['ticks']} ticks")
    rn = run_mission_image_loop(g, "A", "D", noise_sigma=25, brightness=50, seed=1)
    print(f"  con ruido (s=25)  : llego={rn['reached']} giros={rn['turns_taken']} "
          f"(percepcion robusta)")
    ro = run_mission_image_loop(g, "A", "D", obstacle_ticks=range(20, 40), noise_sigma=25, seed=1)
    print(f"  pato + ruido      : llego={ro['reached']} paradas={ro['n_stops']} "
          f"(freno por fusion vision+ToF, luego reanudo)")
    print("  OK: el cerebro condujo TODA la mision solo con pixeles "
          "(carril + franja + giros + ruta), con ruido y frenando ante el pato."
          if r["reached"] and rn["reached"] and ro["reached"] else "  FALLO")


if __name__ == "__main__":
    _demo()
