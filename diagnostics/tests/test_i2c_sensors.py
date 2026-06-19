"""Tests del mapa/validacion de sensores I2C del HUT."""
from i2c_sensors import expected_addresses, identify, whoami_register, whoami_ok


def test_direcciones_esperadas():
    addrs = expected_addresses()
    assert 0x68 in addrs and 0x29 in addrs   # IMU y ToF


def test_identify():
    assert "MPU6050" in identify(0x68)
    assert "VL53L0X" in identify(0x29)
    assert identify(0x11) == "desconocido"


def test_whoami_register():
    assert whoami_register(0x68) == 0x75
    assert whoami_register(0x40) is None      # PWM no se valida por whoami


def test_whoami_ok():
    assert whoami_ok(0x68, 0x68) is True      # IMU correcto
    assert whoami_ok(0x68, 0x00) is False     # IMU responde mal
    assert whoami_ok(0x29, 0xEE) is True      # ToF correcto
    assert whoami_ok(0x40, 0x12) is None      # no aplica
