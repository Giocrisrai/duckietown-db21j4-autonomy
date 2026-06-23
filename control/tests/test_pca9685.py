"""Tests del driver PCA9685 directo (sin ROS), con un bus I2C falso.

Patron portado (adaptado) del repo del colega Luissalamanca23/
Duckietown_Follow_Line_Pi (drivers/duckie_motor.py). Da un driver de motor propio
y testeable, parametrizable por hw_profile (bus, addr) -> sirve para la HUT o un
PCA9685 externo (bypass).
"""
import pytest
from pca9685 import prescale_for, duty_to_off, pwm_registers, PCA9685, LED0_ON_L


def test_prescale_1600hz():
    # round(25e6 / (4096*1600)) - 1 = 4 - 1 = 3
    assert prescale_for(1600) == 3


def test_duty_to_off():
    assert duty_to_off(0.0) == 0
    assert duty_to_off(1.0) == 4095
    assert duty_to_off(0.5) == pytest.approx(2047, abs=1)
    assert duty_to_off(2.0) == 4095          # clamp arriba
    assert duty_to_off(-1.0) == 0            # clamp abajo


def test_pwm_registers_canal():
    regs = pwm_registers(0, on=0, off=2048)
    # canal 0 -> LED0_ON_L..+3 ; orden on_l, on_h, off_l, off_h
    assert [r for r, _ in regs] == [LED0_ON_L, LED0_ON_L + 1, LED0_ON_L + 2, LED0_ON_L + 3]
    assert regs[2][1] == 2048 & 0xFF
    assert regs[3][1] == (2048 >> 8) & 0xFF


class _FakeBus:
    """Bus I2C falso: registra escrituras y simula MODE1 leible."""
    def __init__(self):
        self.writes = []
        self.regs = {0x00: 0x11}

    def write_byte_data(self, addr, reg, val):
        self.writes.append((addr, reg, val & 0xFF))
        self.regs[reg] = val & 0xFF

    def read_byte_data(self, addr, reg):
        return self.regs.get(reg, 0)


def test_init_y_set_duty_escriben_al_driver():
    bus = _FakeBus()
    pca = PCA9685(bus, addr=0x60)
    pca.init(freq_hz=1600)
    # set_duty del canal 8 a 0.5 escribe los 4 registros del canal.
    bus.writes.clear()
    pca.set_duty(8, 0.5)
    regs_escritos = [reg for (_, reg, _) in bus.writes]
    base = LED0_ON_L + 4 * 8
    assert regs_escritos == [base, base + 1, base + 2, base + 3]
    # todas al addr correcto
    assert all(a == 0x60 for (a, _, _) in bus.writes)


def test_set_duty_clampa_canal_apagado_en_cero():
    bus = _FakeBus()
    pca = PCA9685(bus, addr=0x60)
    pca.set_duty(8, 0.0)
    off_l = [v for (_, reg, v) in bus.writes if reg == LED0_ON_L + 4 * 8 + 2]
    assert off_l and off_l[-1] == 0
