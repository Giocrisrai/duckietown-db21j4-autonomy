"""Tests para la lógica de freno de emergencia basada en ToF VL53L0X."""
import math
from duckietown_planning.safety import should_brake, should_brake_range


def test_frena_si_cerca():
    assert should_brake(0.10) is True


def test_no_frena_si_lejos():
    assert should_brake(0.50) is False


def test_lectura_invalida_no_frena():
    assert should_brake(0.0) is False
    assert should_brake(None) is False


def test_umbral_personalizado():
    assert should_brake(0.25, threshold_m=0.30) is True


def test_range_nan_frena():
    assert should_brake_range(float("nan"), 0.03, 1.2) is True


def test_range_menor_a_min_frena():
    assert should_brake_range(0.01, 0.03, 1.2) is True          # objeto demasiado cerca


def test_range_menos_infinito_frena():
    assert should_brake_range(float("-inf"), 0.03, 1.2) is True


def test_range_mas_infinito_no_frena():
    assert should_brake_range(float("inf"), 0.03, 1.2) is False  # sin objeto


def test_range_mayor_a_max_no_frena():
    assert should_brake_range(2.0, 0.03, 1.2) is False


def test_range_en_rango_cerca_frena():
    assert should_brake_range(0.10, 0.03, 1.2, threshold_m=0.15) is True


def test_range_en_rango_lejos_no_frena():
    assert should_brake_range(0.50, 0.03, 1.2, threshold_m=0.15) is False
