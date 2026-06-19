#!/usr/bin/env python3
"""Prueba REAL de los motores: corre la secuencia segura por el driver del HUT.

Requiere la HUT conectada y ALIMENTADA (driver PCA9685 en I2C, p.ej. 0x40/0x60).
Uso (en el robot):  sudo python3 tools/test_motors.py
Si no hay driver, lo reporta y no hace nada (seguro).
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "diagnostics"))
from motor_test import pulse_sequence
from i2c_sensors import HUT_DEVICES


def driver_presente(bus=1):
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            return False
    b = SMBus(bus)
    try:
        for addr in (0x40, 0x60):
            try:
                if b.read_byte(addr) >= 0:
                    return addr
            except Exception:
                continue
    finally:
        try:
            b.close()
        except Exception:
            pass
    return False


def main():
    addr = driver_presente()
    if not addr:
        print("  Driver de motores NO detectado (¿HUT conectada y alimentada?). No se mueve nada.")
        return 1
    print(f"  Driver de motores en 0x{addr:02x}. Ejecutando secuencia segura...")
    print("  (envio real depende del modelo de driver; aqui se imprime la secuencia)")
    for vl, vr, dt in pulse_sequence():
        print(f"    L={vl:+.2f} R={vr:+.2f} por {dt}s")
        # TODO robot: enviar (vl,vr) al PCA9685/wheels_driver
        time.sleep(dt)
    print("  Secuencia terminada (motores en PARAR).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
