"""Tests para la lógica de freno de emergencia basada en ToF VL53L0X."""
from duckietown_planning.safety import should_brake


def test_frena_si_cerca():
    assert should_brake(0.10) is True


def test_no_frena_si_lejos():
    assert should_brake(0.50) is False


def test_lectura_invalida_no_frena():
    assert should_brake(0.0) is False
    assert should_brake(None) is False


def test_umbral_personalizado():
    assert should_brake(0.25, threshold_m=0.30) is True
