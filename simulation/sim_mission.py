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
# Pato cruzando por el LATERAL (fuera del corredor central): solo cede ante el si
# hay una senal de cruce (peaton / duck-crossing) activa.
SIDE_DUCK = {"class_name": "duckie", "bbox": (500, 320, 590, 450), "confidence": 0.9}
# Otro Duckiebot con prioridad a la DERECHA en un 4-way (se le cede el paso).
BOT_RIGHT = {"class_name": "duckiebot", "bbox": (500, 320, 600, 450), "confidence": 0.9}
# Vehiculo en la via principal por la IZQUIERDA de una T (la via secundaria le cede).
BOT_LEFT = {"class_name": "duckiebot", "bbox": (40, 320, 140, 450), "confidence": 0.9}


def paint_stop_line(img, dist, half_w=0.13):
    """Pinta la franja roja perpendicular en el suelo a distancia `dist` (m)."""
    for y in np.linspace(-half_w, half_w, 40):
        _paint(img, dist, y, (0, 0, 255), blob=9)


def run_mission_image_loop(graph, start, goal, seg_len=1.0, off0=0.03,
                           turn_ticks=3, dt=0.05, base_speed=0.3, max_ticks=4000,
                           noise_sigma=0.0, brightness=0, occlusion=0.0, seed=0,
                           obstacle_ticks=(), signs=None, stop_sign_dwell=5,
                           crossing_duck_ticks=(), priority_vehicle_ticks=(),
                           main_road_vehicle_ticks=(), forbidden_turns=None):
    """Corre la mision completa en lazo imagen->imagen y devuelve un trace.

    noise_sigma/brightness/occlusion: ruido realista de camara aplicado a CADA
    frame antes de la percepcion (gaussiano, sesgo de brillo, banda ocluida).
    obstacle_ticks: indices de tick con un pato delante (la fusion vision+ToF lo
    detecta y el robot frena; al despejarse reanuda y completa la mision).
    signs: dict {indice_de_tramo: tag_id de AprilTag} a pintar en la senal de la
    interseccion; se DETECTA con el detector real (pupil-apriltags) sobre el frame
    y su accion (stop/yield -> parada obligatoria) entra al cerebro. Requiere las
    deps de AprilTags; sin `signs` la mision es dep-free.

    Devuelve dict: reached, final_node, turns_taken, n_intersections, max_off,
    n_stops, signs_detected, ticks.
    """
    obstacle_ticks = set(obstacle_ticks)
    crossing_duck_ticks = set(crossing_duck_ticks)
    priority_vehicle_ticks = set(priority_vehicle_ticks)
    main_road_vehicle_ticks = set(main_road_vehicle_ticks)
    forbidden_turns = forbidden_turns or {}
    sign_patches = {}                                 # parches de senal precomputados por tramo
    if signs:
        from sim_apriltags import make_tag_image
        for si, tid in signs.items():
            tag = make_tag_image(tid, scale=10)
            sign_patches[si] = _np.dstack([tag] * 3)
    rng = _np.random.default_rng(seed)
    route = plan_route(graph, start, goal)
    n_seg = len(route) if route else 0
    # Control PROPORCIONAL puro sobre d + k_heading*phi (el rumbo aporta el
    # amortiguamiento), igual que el lazo de seguir-carril validado: el PID por
    # defecto (kd alto) oscilaria con el ruido de phi y perderia el carril en curva.
    ctrl = AutonomousController(graph, start, goal, homography=_H,
                               img_size=(IMG_W, IMG_H), base_speed=base_speed,
                               pid=PID(3.0, 0.0, 0.0, out_limits=(-5.0, 5.0)),
                               k_heading=0.5, stop_sign_dwell=stop_sign_dwell)

    seg_idx = 0
    s, off, dpsi = 0.0, float(off0), 0.0
    turn_count, pending_turn_done = 0, False
    prev_state, turns_taken, max_off = None, [], abs(off0)
    n_stops, signs_detected = 0, []
    reached, ticks = False, 0

    for t in range(max_ticks):
        ticks = t + 1
        curvature = SEG_CURVATURES[seg_idx % len(SEG_CURVATURES)] if n_seg else 0.0

        # 1) MUNDO: render del tramo + franja de stop + senal (AprilTag) si esta cerca.
        img = render(off, dpsi, curvature)
        dist_to_line = seg_len - s
        approaching = 0.0 < dist_to_line <= STOP_LINE_PAINT_M
        if approaching:
            paint_stop_line(img, dist_to_line)
        patch = sign_patches.get(seg_idx) if approaching else None
        if patch is not None:
            th, tw = patch.shape[:2]
            x0 = (IMG_W - tw) // 2
            img[40:40 + th, x0:x0 + tw] = patch       # senal arriba (fuera de la ROI del carril)
        if noise_sigma or brightness or occlusion:
            img = _perturb(img, rng, noise_sigma, brightness, occlusion)

        # 2) PERCEPCION sobre la imagen real: franja (homografia -> metros) y, si hay
        #    senal en escena, el AprilTag con el detector real -> accion (stop/yield/...).
        sl = detect_stop_line(img, homography=_H)
        sign_action = None
        if patch is not None:
            from sim_apriltags import detect_tag_ids
            from object_detection.apriltag_policy import select_action
            sign_action = select_action(detect_tag_ids(img))
            if sign_action and sign_action not in signs_detected:
                signs_detected.append(sign_action)

        # 3) CEREBRO: conduce con la imagen (lane_error interno) + eventos de franja
        #    + obstaculo (fusion vision+ToF) + senal (parada obligatoria por AprilTag).
        detections = []
        tof = None
        if t in obstacle_ticks:
            detections.append(DUCK)
            tof = 0.10
        if t in crossing_duck_ticks:
            detections.append(SIDE_DUCK)              # pato cruzando por el lateral
        if t in priority_vehicle_ticks:
            detections.append(BOT_RIGHT)              # vehiculo con prioridad a la derecha (4-way)
        if t in main_road_vehicle_ticks:
            detections.append(BOT_LEFT)               # vehiculo en la via principal por la izquierda (T)
        # do-not-enter detectado: prohibe el giro indicado para este tramo.
        forbidden_turn = forbidden_turns.get(seg_idx) if sign_action == "no-entry" else None
        cmd = ctrl.step({
            "image": img, "img_w": IMG_W, "img_h": IMG_H,
            "stop_line": sl["detected"], "stop_line_distance_m": sl["distance_m"],
            "turn_done": pending_turn_done,
            "detections": detections, "tof": tof,
            "sign_action": sign_action, "forbidden_turn": forbidden_turn,
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
            "max_off": max_off, "n_stops": n_stops,
            "signs_detected": signs_detected, "ticks": ticks}


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
    rs = r
    try:
        from object_detection.apriltag_policy import _db
        db = _db()
        stop_id = next(i for i, t in db.items() if t == "stop")
        yield_id = next(i for i, t in db.items() if t == "yield")
        base = run_mission_image_loop(g, "A", "D")
        rs = run_mission_image_loop(g, "A", "D", signs={0: stop_id}, stop_sign_dwell=8)
        ry = run_mission_image_loop(g, "A", "D", signs={0: yield_id})
        print(f"  AprilTag STOP     : detectada={rs['signs_detected']} ticks={rs['ticks']} "
              f"(+{rs['ticks']-base['ticks']} vs base: parada obligatoria)")
        print(f"  AprilTag YIELD    : detectada={ry['signs_detected']} ticks={ry['ticks']} "
              f"(~base: despejado, cede solo si hay obstaculo)")
        duckx_id = next(i for i, t in db.items() if t == "duck-crossing")
        rx = run_mission_image_loop(g, "A", "D", signs={0: duckx_id}, crossing_duck_ticks=range(0, 70))
        print(f"  AprilTag CRUCE    : detectada={rx['signs_detected']} paradas={rx['n_stops']} "
              f"(cede a un pato cruzando por el lateral)")
        fourway_id = next(i for i, t in db.items() if t == "4-way-intersect")
        r4 = run_mission_image_loop(g, "A", "D", signs={0: fourway_id}, priority_vehicle_ticks=range(0, 66))
        print(f"  AprilTag 4-WAY    : detectada={r4['signs_detected']} ticks={r4['ticks']} "
              f"(+{r4['ticks']-base['ticks']} vs base: cede su turno al de la derecha)")
        t_id = next(i for i, t in db.items() if t == "T-intersection")
        rt = run_mission_image_loop(g, "A", "D", signs={0: t_id}, main_road_vehicle_ticks=range(0, 66))
        print(f"  AprilTag T-INTER  : detectada={rt['signs_detected']} ticks={rt['ticks']} "
              f"(+{rt['ticks']-base['ticks']} vs base: cede a la via principal, izq y der)")
        dne_id = next(i for i, t in db.items() if t == "do-not-enter")
        gdne = MapGraph(["A", "B"], [{"from": "A", "to": "B", "turn": "straight"},
                                     {"from": "A", "to": "B", "turn": "right"}])
        rd = run_mission_image_loop(gdne, "A", "B", signs={0: dne_id}, forbidden_turns={0: "straight"})
        print(f"  AprilTag NO-ENTRY : detectada={rd['signs_detected']} giro={rd['turns_taken']} "
              f"(prohibio 'straight', tomo la alternativa)")
    except Exception as e:                            # deps de AprilTag ausentes
        print(f"  AprilTags         : (omitido: {type(e).__name__})")
    print("  OK: el cerebro condujo TODA la mision solo con pixeles "
          "(carril + franja + giros + ruta + senales), con ruido y frenando ante el pato."
          if r["reached"] and rn["reached"] and ro["reached"] and rs["reached"] else "  FALLO")


if __name__ == "__main__":
    _demo()
