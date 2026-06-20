"""Tests del control de motores (secuencia segura)."""
from motor_test import (
    pulse_sequence, clamp_velocity, VMAX, velocity_to_command, SPEED_MAX, SPEED_CAP,
)


def test_clamp_velocity():
    assert clamp_velocity(0.9) == VMAX
    assert clamp_velocity(-0.9) == -VMAX
    assert clamp_velocity(0.2) == 0.2


def test_secuencia_termina_en_parar():
    seq = pulse_sequence()
    assert seq[-1] == (0.0, 0.0, seq[-1][2])      # ultimo paso = parar (seguridad)


def test_secuencia_cubre_adelante_atras_giros():
    vl = [s[0] for s in pulse_sequence(v=0.3)]
    vr = [s[1] for s in pulse_sequence(v=0.3)]
    assert (0.3, 0.3) == (vl[0], vr[0])            # adelante
    assert (-0.3, -0.3) == (vl[2], vr[2])          # atras
    assert vl[4] < vr[4] and vl[5] > vr[5]         # giros izq/der


def test_velocidades_acotadas():
    for l, r, _ in pulse_sequence(v=10.0):         # pide exceso
        assert -VMAX <= l <= VMAX and -VMAX <= r <= VMAX


def test_velocity_to_command_direccion():
    assert velocity_to_command(0.0)[0] == "RELEASE"
    assert velocity_to_command(0.3)[0] == "FORWARD"
    assert velocity_to_command(-0.3)[0] == "BACKWARD"


def test_velocity_to_command_release_speed_cero():
    assert velocity_to_command(0.0) == ("RELEASE", 0)


def test_velocity_to_command_speed_acotado_y_con_tope():
    # a velocidad maxima, el speed nunca supera SPEED_MAX*SPEED_CAP (tope de seguridad)
    _, speed = velocity_to_command(VMAX)
    assert speed == int(round(SPEED_MAX * SPEED_CAP))
    assert 0 <= speed <= SPEED_MAX
    # exceso de velocidad sigue acotado
    _, speed_exceso = velocity_to_command(10.0)
    assert speed_exceso == speed


def test_velocity_to_command_monotona():
    # mas velocidad -> mas speed (en magnitud)
    s1 = velocity_to_command(0.1)[1]
    s2 = velocity_to_command(0.4)[1]
    assert s2 > s1
