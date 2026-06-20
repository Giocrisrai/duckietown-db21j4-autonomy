"""Cerebro AUTONOMO integrado: une todas las piezas en un controlador unico.

Compone, tick a tick, el stack completo de conduccion:
  fusion de obstaculos (vision YOLO26 + ToF)  ->  decision de frenado
  FSM de conduccion (carril/interseccion/giro/parada)  ->  modo
  PID de carril  ->  omega; velocidad base modulada por el frenado
  planeador global (ruta BFS) + politica reactiva  ->  giro en interseccion
  localizacion por conteo  ->  saber en que interseccion estamos / si llegamos

Entrada por `step(obs)` (lo que produce la percepcion + sensores):
  lane_error  : error lateral al centro del carril (m), para el PID
  detections  : lista de detecciones YOLO26 (class_name, bbox, confidence)
  tof         : distancia del ToF (m) o None
  stop_line   : bool, se ve la franja roja de stop
  at_stop_line: bool, estamos sobre la linea (distancia/proxy bajo umbral)
  turn_done   : bool, la maniobra de giro termino
  img_w, img_h: tamano de imagen (para el proxy de cercania de la fusion)

Salida: dict con la orden de marcha (v, omega), el estado de la FSM, el giro
elegido, la decision de obstaculo y si ya se llego al destino.
"""
from .fsm import (
    DrivingFSM, LANE_FOLLOWING, APPROACHING_INTERSECTION, AT_INTERSECTION,
    TURNING, STOPPED,
)
from . import obstacle_fusion
from .intersection_policy import choose_turn
from .route_search import plan_route
from .localization import IntersectionLocalizer

# Velocidad angular de la maniobra de giro en interseccion (rad/s aprox).
_TURN_OMEGA = {"left": 1.5, "right": -1.5, "straight": 0.0}


class AutonomousController:
    def __init__(self, graph=None, start=None, goal=None, base_speed=0.3,
                 pid=None, img_size=(640, 480)):
        self.graph = graph
        self.goal = goal
        self.base_speed = base_speed
        self.img_w, self.img_h = img_size
        self.fsm = DrivingFSM()
        if pid is None:
            from pid import PID                      # import diferido (modulo de control)
            pid = PID(6.0, 0.0, 4.0, out_limits=(-5.0, 5.0))
        self.pid = pid
        self.route = plan_route(graph, start, goal) if graph is not None else []
        self.loc = IntersectionLocalizer(graph, start) if graph is not None else None
        self._turn = None                            # giro elegido en la interseccion actual

    def reached(self):
        """True si la localizacion indica que llegamos al destino."""
        return self.loc is not None and self.loc.current == self.goal

    def _available_turns(self):
        # Con grafo: los giros REALES del nodo actual ([] si es callejon sin salida).
        # Sin grafo (conduccion libre): "straight" por defecto.
        if self.graph is not None and self.loc is not None:
            return self.graph.turns_from(self.loc.current)
        return ["straight"]

    def step(self, obs, dt=0.05):
        """Avanza un tick. Devuelve la orden de marcha y la telemetria."""
        # 1) Fusion de obstaculos (vision + ToF) -> GO/SLOW/STOP.
        decision = obstacle_fusion.fuse(
            obs.get("detections", []), obs.get("tof"), self.img_w, self.img_h)

        # 2) FSM: la seguridad manda; el giro queda "listo" si ya lo elegimos.
        prev = self.fsm.state
        state = self.fsm.tick(
            obstacle=decision["action"],
            stop_line=obs.get("stop_line", False),
            at_stop_line=obs.get("at_stop_line", False),
            turn_ready=self._turn is not None,
            turn_done=obs.get("turn_done", False),
        )

        # 3) Giro completado (TURNING -> LANE_FOLLOWING): avanzar localizacion/ruta.
        if prev == TURNING and state == LANE_FOLLOWING:
            if self.loc is not None:
                self.loc.update(self._turn)
            if self.route and self._turn == self.route[0]:
                self.route.pop(0)
            self.pid.reset()
            self._turn = None

        # 4) Al llegar a la interseccion, elegir el giro (global con respaldo reactivo).
        #    Callejon sin salida (sin giros validos): no inventar giro fantasma; el
        #    robot se queda detenido en la interseccion (seguro) en vez de descarrilar.
        if state == AT_INTERSECTION and self._turn is None:
            available = self._available_turns()
            if available:
                planned = self.route[0] if self.route else None
                self._turn = choose_turn(available, planned=planned)

        # 5) Orden de marcha segun el estado.
        v, omega = 0.0, 0.0
        if state in (LANE_FOLLOWING, APPROACHING_INTERSECTION):
            omega = self.pid.step(obs.get("lane_error", 0.0), dt)
            v = self.base_speed * (1.0 - decision["brake"])      # SLOW reduce; GO pleno
        elif state == TURNING:
            v = self.base_speed * 0.5
            omega = _TURN_OMEGA.get(self._turn, 0.0)
        # AT_INTERSECTION y STOPPED -> v=omega=0 (detenido)

        return {
            "v": v, "omega": omega, "state": state, "turn": self._turn,
            "decision": decision, "reached": self.reached(),
        }
