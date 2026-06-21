"""Tests de la maquina de estados de conduccion."""
from duckietown_planning.fsm import (
    DrivingFSM,
    LANE_FOLLOWING, APPROACHING_INTERSECTION, AT_INTERSECTION, TURNING, STOPPED,
)


def test_arranca_en_lane_following():
    assert DrivingFSM().state == LANE_FOLLOWING


def test_ciclo_completo_de_interseccion():
    fsm = DrivingFSM()
    assert fsm.tick(stop_line=True) == APPROACHING_INTERSECTION
    assert fsm.tick(stop_line=True) == APPROACHING_INTERSECTION       # aun no en la linea
    assert fsm.tick(at_stop_line=True) == AT_INTERSECTION
    assert fsm.tick() == AT_INTERSECTION                              # espera decision
    assert fsm.tick(turn_ready=True) == TURNING
    assert fsm.tick() == TURNING                                     # girando
    assert fsm.tick(turn_done=True) == LANE_FOLLOWING


def test_obstaculo_para_desde_cualquier_estado():
    for inicial in (LANE_FOLLOWING, APPROACHING_INTERSECTION, AT_INTERSECTION, TURNING):
        fsm = DrivingFSM(state=inicial)
        assert fsm.tick(obstacle="STOP") == STOPPED


def test_obstaculo_despejado_reanuda():
    fsm = DrivingFSM()
    fsm.tick(obstacle="STOP")
    assert fsm.state == STOPPED
    assert fsm.tick(obstacle="GO") == LANE_FOLLOWING


def test_seguridad_tiene_prioridad_sobre_stop_line():
    fsm = DrivingFSM()
    # aunque haya linea roja, si hay obstaculo manda parar
    assert fsm.tick(obstacle="STOP", stop_line=True) == STOPPED


def test_slow_no_cambia_estado():
    fsm = DrivingFSM()
    assert fsm.tick(obstacle="SLOW") == LANE_FOLLOWING
    fsm.tick(stop_line=True)
    assert fsm.tick(obstacle="SLOW") == APPROACHING_INTERSECTION


def test_is_moving_vs_is_stopped():
    fsm = DrivingFSM()
    assert fsm.is_moving() and not fsm.is_stopped()            # LANE_FOLLOWING
    fsm.state = AT_INTERSECTION
    assert fsm.is_stopped() and not fsm.is_moving()
    fsm.state = TURNING
    assert fsm.is_moving()
    fsm.state = STOPPED
    assert fsm.is_stopped()


def test_no_salta_etapas():
    fsm = DrivingFSM()
    # estar sobre la linea sin haberla visto antes no debe ir directo a AT_INTERSECTION
    assert fsm.tick(at_stop_line=True) == LANE_FOLLOWING


def test_freno_en_interseccion_reanuda_en_interseccion():
    # Si para por seguridad ESTANDO en la interseccion (linea ya bajo el robot, no
    # se volvera a detectar), al despejarse debe volver a AT_INTERSECTION para no
    # saltarse el giro, no a LANE_FOLLOWING.
    fsm = DrivingFSM()
    fsm.tick(stop_line=True)                          # APPROACHING
    fsm.tick(at_stop_line=True)                       # AT_INTERSECTION
    assert fsm.tick(obstacle="STOP") == STOPPED       # pato cruza
    assert fsm.tick(obstacle="GO") == AT_INTERSECTION  # se va -> sigue en la interseccion
    assert fsm.tick(turn_ready=True) == TURNING       # y ejecuta el giro pendiente


def test_freno_durante_giro_reanuda_en_lane_following():
    # En cambio, interrumpir un GIRO a medias si reanuda en LANE_FOLLOWING
    # (no se reanuda una maniobra parcial; se vuelve a detectar la interseccion).
    fsm = DrivingFSM()
    fsm.tick(stop_line=True)
    fsm.tick(at_stop_line=True)
    fsm.tick(turn_ready=True)                         # TURNING
    assert fsm.tick(obstacle="STOP") == STOPPED
    assert fsm.tick(obstacle="GO") == LANE_FOLLOWING
