"""Maquina de estados de conduccion (paridad con dt-core fsm).

Orquesta de forma EXPLICITA y depurable los modos del robot, en vez de logica
reactiva dispersa. La seguridad (obstaculo) tiene prioridad maxima sobre todo.

Estados:
  LANE_FOLLOWING          - conduccion normal siguiendo el carril.
  APPROACHING_INTERSECTION- vio la linea roja de stop, sigue avanzando hacia ella.
  AT_INTERSECTION         - detenido en la linea, esperando decision de giro.
  TURNING                 - ejecutando la maniobra de giro.
  STOPPED                 - parada de seguridad por obstaculo (visual+ToF).

Entradas por tick (eventos del resto del stack):
  obstacle  : "GO" | "SLOW" | "STOP"  (de obstacle_fusion)
  stop_line : bool  (object_detection.stop_line detecto la franja roja)
  at_stop_line : bool  (estamos sobre la linea: distancia/proxy por debajo del umbral)
  turn_ready : bool  (el navegador/planeador tiene un giro decidido)
  turn_done  : bool  (la maniobra de giro termino)

Nota: tras una parada de seguridad se reanuda en LANE_FOLLOWING y el pipeline
vuelve a detectar la linea/interseccion (no se reanuda una maniobra a medias).
"""
LANE_FOLLOWING = "LANE_FOLLOWING"
APPROACHING_INTERSECTION = "APPROACHING_INTERSECTION"
AT_INTERSECTION = "AT_INTERSECTION"
TURNING = "TURNING"
STOPPED = "STOPPED"

# Estados en los que el robot esta en movimiento (vs detenido).
_MOVING = {LANE_FOLLOWING, APPROACHING_INTERSECTION, TURNING}
_STOPPED = {AT_INTERSECTION, STOPPED}


class DrivingFSM:
    """Maquina de estados de conduccion. `tick()` avanza un paso y devuelve el estado."""

    def __init__(self, state=LANE_FOLLOWING):
        self.state = state
        self._pre_stop = None                # estado previo a una parada de seguridad

    def tick(self, obstacle="GO", stop_line=False, at_stop_line=False,
             turn_ready=False, turn_done=False):
        # 1) Seguridad primero: un obstaculo manda parar desde cualquier estado.
        if obstacle == "STOP":
            if self.state != STOPPED:
                self._pre_stop = self.state       # recordar de donde veniamos
            self.state = STOPPED
            return self.state
        # 2) Si veniamos de una parada de seguridad y ya se despejo, reanudar.
        #    Si estabamos detenidos EN la interseccion (la franja ya quedo bajo el
        #    robot y no se volvera a detectar), volver a AT_INTERSECTION para no
        #    saltarnos el giro; en cualquier otro caso, reanudar en LANE_FOLLOWING
        #    (un giro a medias NO se reanuda: se vuelve a detectar la interseccion).
        if self.state == STOPPED:
            self.state = (AT_INTERSECTION if self._pre_stop == AT_INTERSECTION
                          else LANE_FOLLOWING)
            self._pre_stop = None

        # 3) Transiciones normales.
        if self.state == LANE_FOLLOWING:
            if stop_line:
                self.state = APPROACHING_INTERSECTION
        elif self.state == APPROACHING_INTERSECTION:
            if at_stop_line:
                self.state = AT_INTERSECTION
        elif self.state == AT_INTERSECTION:
            if turn_ready:
                self.state = TURNING
        elif self.state == TURNING:
            if turn_done:
                self.state = LANE_FOLLOWING
        return self.state

    def is_moving(self):
        """True si el robot debe avanzar en el estado actual."""
        return self.state in _MOVING

    def is_stopped(self):
        """True si el robot debe estar detenido (en interseccion o por obstaculo)."""
        return self.state in _STOPPED
