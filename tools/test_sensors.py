#!/usr/bin/env python3
"""Prueba REAL de los sensores del Duckiebot por I2C (IMU/ToF/PWM/OLED).

Escanea las direcciones del HUT, verifica presencia y WHO_AM_I. Uso (en el robot):
    sudo python3 tools/test_sensors.py [bus]      # bus por defecto: 1
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "diagnostics"))
from i2c_sensors import HUT_DEVICES, identify, whoami_register, whoami_ok


def main(bus=1):
    try:
        from smbus2 import SMBus
    except ImportError:
        from smbus import SMBus
    encontrados = 0
    print(f"== Probando sensores en I2C bus {bus} ==")
    with SMBus(bus) as b:
        for addr in sorted(HUT_DEVICES):
            try:
                b.read_byte(addr)
            except Exception:
                continue
            encontrados += 1
            estado = "presente"
            reg = whoami_register(addr)
            if reg is not None:
                try:
                    val = b.read_byte_data(addr, reg)
                    ok = whoami_ok(addr, val)
                    estado = f"WHO_AM_I=0x{val:02x} -> {'OK' if ok else 'MISMATCH'}"
                except Exception as e:
                    estado = f"presente (whoami no leido: {e})"
            print(f"  0x{addr:02x} {identify(addr)}: {estado}")
    if not encontrados:
        print("  Ningun sensor del HUT detectado. ¿HUT conectada y ALIMENTADA?")
    return encontrados


if __name__ == "__main__":
    sys.exit(0 if main(int(sys.argv[1]) if len(sys.argv) > 1 else 1) else 1)
