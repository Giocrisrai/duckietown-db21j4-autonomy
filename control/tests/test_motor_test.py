"""Tests del control de motores (secuencia segura)."""
from motor_test import pulse_sequence, clamp_velocity, VMAX


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
