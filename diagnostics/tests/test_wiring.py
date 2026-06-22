"""Tests del cableado directo al header de 40 pines (bypass de una HUT danada).

Cuando la HUT tiene la seccion I2C muerta, se pueden cablear los sensores I2C
directo al header del Jetson y usar un driver de motor externo (PCA9685 @ 0x60,
drop-in con el codigo de motores existente). Este modulo codifica ese cableado.
"""
from wiring import (
    header_i2c1_pins,
    directly_wireable,
    external_motor_driver_addr,
    wiring_summary,
)
from i2c_sensors import expected_addresses


def test_pines_i2c1_del_header():
    """Jetson Nano 40-pin, bus I2C-1: SDA=3, SCL=5, 3V3=1, GND=6."""
    p = header_i2c1_pins()
    assert p["sda"] == 3
    assert p["scl"] == 5
    assert p["v3v3"] == 1
    assert p["gnd"] == 6


def test_motor_driver_externo_es_drop_in_0x60():
    """El driver de motor externo debe ir en 0x60 para ser drop-in con el stack."""
    assert external_motor_driver_addr() == 0x60
    # …y 0x60 debe seguir siendo una direccion esperada del mapa I2C del stack.
    assert 0x60 in expected_addresses()


def test_sensores_cableables_directo():
    """Los sensores I2C (IMU, ToF, OLED) se pueden cablear directo; comparten bus."""
    d = directly_wireable()
    addrs = {x["addr"] for x in d}
    assert 0x68 in addrs   # IMU MPU-6050
    assert 0x29 in addrs   # ToF VL53L0X
    assert 0x3C in addrs   # OLED SSD1306
    # Todo lo cableable directo debe estar en el mapa I2C conocido del stack.
    assert addrs.issubset(set(expected_addresses()))


def test_wiring_summary_menciona_pines_y_motor_externo():
    s = wiring_summary().lower()
    assert "pin 3" in s and "pin 5" in s        # SDA / SCL
    assert "0x60" in s                           # driver de motor externo
    assert "externo" in s or "h-bridge" in s or "puente" in s
