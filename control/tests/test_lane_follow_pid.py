"""Tests del PID de seguir-linea (error cross-track + anti-windup).

Patron portado (adaptado) del repo del colega Luissalamanca23/
Duckietown_Follow_Line_Pi (robot/controller.py). El anti-windup evita que el
integral se dispare con el robot quieto (banco/sim) y el "volante se clave".
"""
from lane_follow_pid import cross_track_error, LanePID


def test_error_signo():
    # Linea a la IZQUIERDA del centro (x chico) -> target_x < goal -> error negativo.
    e_izq = cross_track_error(x=50, y=0, h=20, center_x=320)
    e_der = cross_track_error(x=500, y=0, h=20, center_x=320)
    assert e_izq < 0 < e_der


def test_omega_corrige_hacia_la_linea():
    pid = LanePID(kp=1.0, ki=0.0, kd=0.0)
    # linea a la izquierda (error<0) -> omega>0 (gira a la izquierda hacia la linea).
    omega = pid.update(cross_track_error(x=50, y=0, h=20, center_x=320), dt=0.1)
    assert omega > 0


def test_clamp_omega_max():
    pid = LanePID(kp=100.0, ki=0.0, kd=0.0, omega_max=8.0)
    omega = pid.update(cross_track_error(x=0, y=0, h=20, center_x=320), dt=0.1)
    assert abs(omega) <= 8.0


def test_anti_windup_limita_integral():
    """Con error constante y robot quieto, el integral NO debe crecer sin tope:
    el omega se satura pero al invertir el signo del error se descarga rapido."""
    pid = LanePID(kp=0.0, ki=1.0, kd=0.0, omega_max=8.0)
    for _ in range(1000):                       # mucho tiempo con error +
        pid.update(5.0, dt=0.1)
    assert abs(pid.integral) <= 8.0 / 1.0 + 1e-9    # clamp = omega_max/|ki|
    # al invertir el error, pocas iteraciones bastan para cruzar a omega>0.
    signos = [pid.update(-5.0, dt=0.1) > 0 for _ in range(20)]
    assert any(signos)


def test_reset_limpia_estado():
    pid = LanePID(ki=1.0)
    pid.update(5.0, dt=0.1)
    pid.reset()
    assert pid.integral == 0.0 and pid.prev_diff == 0.0
