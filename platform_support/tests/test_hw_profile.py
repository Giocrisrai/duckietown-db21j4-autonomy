"""Tests del perfil de hardware central (bus I2C + direcciones, configurable)."""
import pytest
from hw_profile import (
    list_profiles,
    active_profile,
    motor_target,
    sensor_addr,
    DEFAULT_PROFILE,
)


def test_default_es_hut():
    assert DEFAULT_PROFILE == "hut"
    p = active_profile()
    assert p["motor_bus"] == 1 and p["motor_addr"] == 0x60


def test_perfiles_disponibles():
    profs = list_profiles()
    assert "hut" in profs
    assert "external-bus1" in profs      # PCA9685 externo en el bus sano, otra dir
    assert "external-gpio" in profs      # bus i2c-gpio nuevo en pines libres


def test_motor_target_por_perfil():
    assert motor_target("hut") == (1, 0x60)
    # external-bus1: reusa el bus 1 (vivo) pero esquiva el 0x60 muerto.
    bus, addr = motor_target("external-bus1")
    assert bus == 1 and addr != 0x60
    # external-gpio: bus distinto al 1, motor en 0x60 (bus limpio nuevo).
    gbus, gaddr = motor_target("external-gpio")
    assert gbus != 1 and gaddr == 0x60


def test_imu_sigue_en_0x68_en_perfiles_de_bus1():
    # En external-bus1 la IMU del HUT (0x68) sigue sirviendo: se reutiliza.
    assert sensor_addr("imu", "external-bus1") == 0x68


def test_env_var_selecciona_perfil(monkeypatch):
    monkeypatch.setenv("DT_HW_PROFILE", "external-gpio")
    assert active_profile()["name"] == "external-gpio"


def test_arg_tiene_prioridad_sobre_env(monkeypatch):
    monkeypatch.setenv("DT_HW_PROFILE", "external-gpio")
    assert active_profile("hut")["name"] == "hut"


def test_perfil_desconocido_falla():
    with pytest.raises(KeyError):
        active_profile("inexistente")
