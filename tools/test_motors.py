#!/usr/bin/env python3
"""Detección del driver de motores + puntero al test REAL.

OJO: este script NO mueve los motores. Solo detecta si el driver PCA9685 del HUT
responde en el bus I2C e imprime la secuencia segura que se enviaría. El envío
REAL a los motores (con el driver oficial `hat_driver` HATv3) está en
`tools/robot_motor_test.py`, y el flujo completo con compuerta de seguridad en
`tools/validate_robot.sh`.

Uso (en el robot):
    sudo python3 tools/test_motors.py            # solo detecta + imprime la secuencia
    sudo bash tools/validate_robot.sh --motors   # <- prueba REAL (gira los motores)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control"))
from motor_test import pulse_sequence, velocity_to_command


def driver_presente(bus=1):
    """Devuelve la direccion del PCA9685 (0x40/0x60) si responde, si no False."""
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            return False
    b = SMBus(bus)
    try:
        for addr in (0x60, 0x40):           # HATv3 usa 0x60; 0x40 como alternativa
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
        print("  Driver de motores NO detectado (¿HUT conectada/alimentada y bus I2C 1 ok?).")
        print("  No se mueve nada. Revisa el acople de 40 pines / la HUT.")
        return 1
    print(f"  Driver de motores en 0x{addr:02x}. Secuencia segura (dirección@PWM):")
    for vl, vr, dt in pulse_sequence():
        dl, sl = velocity_to_command(vl)
        dr, sr = velocity_to_command(vr)
        print(f"    L:{dl}@{sl:<3}  R:{dr}@{sr:<3}  ({dt}s)")
    print("  Para GIRARLOS de verdad: sudo bash tools/validate_robot.sh --motors")
    return 0


if __name__ == "__main__":
    sys.exit(main())
