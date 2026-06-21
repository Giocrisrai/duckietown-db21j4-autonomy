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
                 pid=None, img_size=(640, 480), homography=None,
                 stop_line_stop_m=0.10, k_heading=0.5, stop_sign_dwell=5):
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
        # Calibracion extrinseca: matriz 3x3, o ruta a un YAML que se carga, o None.
        # Con ella, la fusion de obstaculos y la parada en la franja usan distancia
        # METRICA real al suelo en vez de proxies.
        self.homography = self._load_homography(homography)
        self.stop_line_stop_m = stop_line_stop_m     # m: distancia a la que se para en la franja
        self.k_heading = k_heading                   # peso del termino de rumbo en el error de carril
        self.stop_sign_dwell = stop_sign_dwell       # ticks de parada obligatoria ante senal stop/yield
        self._sign_dwell = 0                          # contador de la parada por senal en curso
        self._last_pose = None                       # ultima pose de carril estimada (telemetria)

    @staticmethod
    def _load_homography(h):
        if h is None or not isinstance(h, str):
            return h                                 # ya es una matriz (o None)
        from object_detection.ground_projection import (  # import diferido (paquete percepcion)
            load_extrinsics, homography as _to_matrix)
        return _to_matrix(load_extrinsics(h))

    def reached(self):
        """True si la localizacion indica que llegamos al destino."""
        return self.loc is not None and self.loc.current == self.goal

    def _lane_error(self, obs):
        """Error de carril para el PID.

        Si la observacion trae `image` y hay homografia, lo estima de la camara:
        proyecta las lineas al suelo -> (d, phi) -> error = d + k_heading*phi
        (cross-track + heading). Si no hay imagen / linea, usa el escalar
        `lane_error` (compatibilidad con quien ya lo entrega calculado).
        """
        img = obs.get("image")
        if img is not None and self.homography is not None:
            from object_detection.lane_pose import (  # import diferido (paquete percepcion)
                estimate_lane_pose, steering_error)
            pose = estimate_lane_pose(img, self.homography)
            err = steering_error(pose["d"], pose["phi"], self.k_heading)
            if err is not None:
                self._last_pose = pose
                return err
        return obs.get("lane_error", 0.0)

    def _crossing_present(self, obs, classes=("duckie", "pedestrian"),
                          band=1.0, min_prox=0.4, min_conf=0.25):
        """True si hay un pato/peaton cruzando en la zona ANCHA (todo el ancho),
        aunque no este en el corredor central. Lo usa el cede-cruce de las senales
        de peaton / duck-crossing para ceder a quien cruza por el lateral."""
        for d in obs.get("detections", []):
            if d.get("class_name") not in classes:
                continue
            if d.get("confidence", 1.0) < min_conf:
                continue
            bbox = d.get("bbox")
            if bbox is None:
                continue
            if (obstacle_fusion.in_path(bbox, self.img_w, band=band)
                    and obstacle_fusion.bbox_proximity(bbox, self.img_h) >= min_prox):
                return True
        return False

    def _priority_vehicle_right(self, obs, classes=("duckiebot",),
                               right_frac=0.55, min_prox=0.4, min_conf=0.25):
        """True si hay un vehiculo (duckiebot) con PRIORIDAD a la derecha en un
        4-way: su centro cae en la zona derecha de la imagen y esta cerca. Regla de
        4-way stop: se cede al de la derecha."""
        for d in obs.get("detections", []):
            if d.get("class_name") not in classes:
                continue
            if d.get("confidence", 1.0) < min_conf:
                continue
            bbox = d.get("bbox")
            if bbox is None:
                continue
            x1, _, x2, _ = bbox
            cx = (x1 + x2) / 2.0
            if (cx > right_frac * self.img_w
                    and obstacle_fusion.bbox_proximity(bbox, self.img_h) >= min_prox):
                return True
        return False

    def _main_road_vehicle_present(self, obs, classes=("duckiebot",),
                                  central_band=0.3, min_prox=0.4, min_conf=0.25):
        """True si hay un vehiculo en la VIA PRINCIPAL de una T: a un lado (izquierda
        o derecha, fuera del carril propio central) y cercano. La via secundaria cede
        a toda la via principal, no solo al de la derecha."""
        for d in obs.get("detections", []):
            if d.get("class_name") not in classes:
                continue
            if d.get("confidence", 1.0) < min_conf:
                continue
            bbox = d.get("bbox")
            if bbox is None:
                continue
            if (not obstacle_fusion.in_path(bbox, self.img_w, band=central_band)
                    and obstacle_fusion.bbox_proximity(bbox, self.img_h) >= min_prox):
                return True
        return False

    def _available_turns(self):
        # Con grafo: los giros REALES del nodo actual ([] si es callejon sin salida).
        # Sin grafo (conduccion libre): "straight" por defecto.
        if self.graph is not None and self.loc is not None:
            return self.graph.turns_from(self.loc.current)
        return ["straight"]

    def step(self, obs, dt=0.05):
        """Avanza un tick. Devuelve la orden de marcha y la telemetria."""
        self._last_pose = None                       # pose de carril de ESTE tick (si la hay)
        # 1) Fusion de obstaculos (vision + ToF) -> GO/SLOW/STOP.
        #    Con homografia decide por distancia metrica real (no por el proxy del bbox).
        decision = obstacle_fusion.fuse(
            obs.get("detections", []), obs.get("tof"), self.img_w, self.img_h,
            homography=self.homography)
        # Cede-cruce: una senal de peaton / duck-crossing (accion 'yield') hace ceder
        # a quien cruza por el LATERAL (que la fusion central no marcaria) -> parar.
        if (decision["action"] == "GO" and obs.get("sign_action") == "yield"
                and self._crossing_present(obs)):
            decision = {"action": "STOP", "brake": 1.0, "reason": "cruce",
                        "tof_stop": False, "max_proximity": 0.0, "distance_m": None}

        # 2) FSM: la seguridad manda; el giro queda "listo" si ya lo elegimos.
        #    Con homografia y distancia metrica a la franja, paramos a una distancia fija.
        at_stop_line = obs.get("at_stop_line", False)
        if self.homography is not None:
            d_sl = obs.get("stop_line_distance_m")
            if d_sl is not None and d_sl <= self.stop_line_stop_m:
                at_stop_line = True
        prev = self.fsm.state
        state = self.fsm.tick(
            obstacle=decision["action"],
            stop_line=obs.get("stop_line", False),
            at_stop_line=at_stop_line,
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
            self._sign_dwell = 0                      # reiniciar la parada por senal

        # 4) Al llegar a la interseccion, elegir el giro (global con respaldo reactivo).
        #    Callejon sin salida (sin giros validos): no inventar giro fantasma; el
        #    robot se queda detenido en la interseccion (seguro) en vez de descarrilar.
        #    Senal de AprilTag: STOP impone una parada obligatoria (dwell fijo) antes
        #    de girar; YIELD cede SOLO si hay obstaculo/trafico cruzado (la fusion no
        #    da GO) y procede en cuanto se despeja; sin senal, gira al llegar.
        if state == AT_INTERSECTION and self._turn is None:
            sign = obs.get("sign_action")
            if sign == "stop" and self._sign_dwell < self.stop_sign_dwell:
                self._sign_dwell += 1                 # stop: parada obligatoria (v=0)
                must_wait = True
            elif sign == "yield":
                must_wait = decision["action"] != "GO"   # yield: cede solo si hay obstaculo
            elif sign == "intersection":
                must_wait = self._priority_vehicle_right(obs)   # 4-way: cede al de la derecha
            elif sign == "yield-main":
                must_wait = self._main_road_vehicle_present(obs)  # T: cede a la via principal
            else:
                must_wait = False
            if not must_wait:
                available = self._available_turns()
                if available:
                    planned = self.route[0] if self.route else None
                    self._turn = choose_turn(available, planned=planned)

        # 5) Orden de marcha segun el estado.
        v, omega = 0.0, 0.0
        if state in (LANE_FOLLOWING, APPROACHING_INTERSECTION):
            omega = self.pid.step(self._lane_error(obs), dt)
            v = self.base_speed * (1.0 - decision["brake"])      # SLOW reduce; GO pleno
        elif state == TURNING:
            v = self.base_speed * 0.5
            omega = _TURN_OMEGA.get(self._turn, 0.0)
        # AT_INTERSECTION y STOPPED -> v=omega=0 (detenido)

        return {
            "v": v, "omega": omega, "state": state, "turn": self._turn,
            "decision": decision, "reached": self.reached(),
            "lane_pose": self._last_pose,
        }
