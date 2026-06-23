#!/usr/bin/env python3
"""Detección del driver de motores + puntero al test REAL.

OJO: este script NO mueve los motores. Solo detecta si el driver PCA9685 del HUT
responde en el bus I2C e imprime la secuencia segura que se enviaría. El envío
REAL a los motores (con el driver oficial `hat_driver` HATv3) está en
`tools/robot_motor_test.py`, y el flujo completo con compuerta de seguridad en
`tools/validate_robot.sh`.

Uso (en el robot):
    sudo python3 tools/detect_motors.py            # solo detecta + imprime la secuencia
    DT_HW_PROFILE=external-bus1 sudo -E python3 tools/detect_motors.py  # driver externo
    sudo bash tools/validate_robot.sh --motors   # <- prueba REAL (gira los motores)

El bus y la direccion del driver salen del PERFIL DE HARDWARE central
(`platform_support/hw_profile.py`), asi la MISMA logica vale para la HUT o un
driver externo en otro puerto/direccion (ver DT_HW_PROFILE).
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "control"))
sys.path.insert(0, str(ROOT / "platform_support"))
from motor_test import pulse_sequence, velocity_to_command   # noqa: E402
from hw_profile import motor_target, active_profile           # noqa: E402


def driver_presente(bus=None, addr=None):
    """Devuelve la direccion del driver de motores si responde, si no False.

    Bus/direccion vienen del perfil de hardware activo (override por args).
    OJO: solo prueba el driver de MOTORES; no toca el de LEDs (moveria LEDs)."""
    if bus is None or addr is None:
        pbus, paddr = motor_target()
        bus = pbus if bus is None else bus
        addr = paddr if addr is None else addr
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            return False
    b = SMBus(bus)
    try:
        try:
            if b.read_byte(addr) >= 0:
                return addr
        except Exception:
            return False
    finally:
        try:
            b.close()
        except Exception:
            pass
    return False


def main():
    prof = active_profile()
    bus, _ = motor_target()
    addr = driver_presente()
    if not addr:
        print(f"  Driver de motores NO detectado en perfil '{prof['name']}' "
              f"(bus {bus}, 0x{prof['motor_addr']:02x}).")
        print("  Revisa HUT/alimentacion, o prueba otro perfil: DT_HW_PROFILE=external-bus1")
        return 1
    print(f"  Driver de motores en bus {bus} 0x{addr:02x} (perfil '{prof['name']}'). "
          f"Secuencia segura (direccion@PWM):")
    for vl, vr, dt in pulse_sequence():
        dl, sl = velocity_to_command(vl)
        dr, sr = velocity_to_command(vr)
        print(f"    L:{dl}@{sl:<3}  R:{dr}@{sr:<3}  ({dt}s)")
    print("  Para GIRARLOS de verdad: sudo bash tools/validate_robot.sh --motors")
    return 0


if __name__ == "__main__":
    sys.exit(main())
