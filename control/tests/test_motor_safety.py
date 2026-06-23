"""Tests de la logica de seguridad de motores (watchdog + clamp).

Patron portado (adaptado) del repo del colega Luissalamanca23/
Duckietown_Follow_Line_Pi (robot/motor_bridge.py): watchdog que corta si no
llega un comando dentro de WATCHDOG_S, y tope de duty para bring-up.
"""
import pytest
from motor_safety import watchdog_expired, safe_duty, STOP


def test_watchdog_expira_pasado_el_umbral():
    assert watchdog_expired(last_cmd_t=10.0, now=10.6, watchdog_s=0.5) is True
    assert watchdog_expired(last_cmd_t=10.0, now=10.4, watchdog_s=0.5) is False
    # En el umbral exacto NO expira todavia (corta solo si se PASA).
    assert watchdog_expired(last_cmd_t=10.0, now=10.5, watchdog_s=0.5) is False


def test_stop_es_cero():
    assert STOP == (0.0, 0.0)


def test_safe_duty_clampa():
    assert safe_duty(0.9, max_duty=0.5) == 0.5
    assert safe_duty(-0.9, max_duty=0.5) == -0.5
    assert safe_duty(0.3, max_duty=0.5) == pytest.approx(0.3)


def test_safe_duty_nan_va_a_cero():
    assert safe_duty(float("nan"), max_duty=0.5) == 0.0
    assert safe_duty(float("inf"), max_duty=0.5) == 0.5
