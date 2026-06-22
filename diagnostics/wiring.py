"""Cableado directo al header de 40 pines del Jetson — bypass de una HUT danada.

Si la seccion I2C de la HUT esta muerta (linea SDA/SCL tomada en bajo, ver
`i2c_bus.py`), se puede seguir desarrollando sin la HUT:

  - SENSORES I2C (IMU 0x68, ToF 0x29, OLED 0x3c): se cablean directo a los pines
    del header (mismo bus 1, mismas direcciones) -> el stack los reconoce sin
    cambios (escanea por direccion en `i2c_sensors.py` / `tools/test_sensors.py`).
  - MOTORES: el Jetson NO puede mover un motor DC directo (sus GPIO dan uA). Hace
    falta un DRIVER EXTERNO con puente H. Si el PCA9685 externo se pone en 0x60,
    es DROP-IN con el codigo de motores del stack (que espera el PWM en 0x60).
  - CAMARA: es CSI, independiente de la HUT -> ya funciona.

Modulo puro/testeable: solo describe el cableado, no toca hardware.
"""
from i2c_sensors import HUT_DEVICES

# Jetson Nano 40-pin header, bus I2C-1 (7000c400.i2c). El dev kit ya trae las
# pull-ups en este bus, asi que los breakouts I2C funcionan sin la HUT.
_HEADER_I2C1 = {"sda": 3, "scl": 5, "v3v3": 1, "gnd": 6}

# Direccion donde debe quedar el PCA9685 EXTERNO para ser drop-in con el stack.
_EXTERNAL_MOTOR_DRIVER_ADDR = 0x60

# Sensores I2C que son breakouts y se cablean directo (no necesitan la HUT).
_DIRECT_SENSORS = (0x68, 0x29, 0x3C)   # IMU, ToF, OLED


def header_i2c1_pins():
    """Pines del header de 40 pines para el bus I2C-1 (SDA/SCL/3V3/GND)."""
    return dict(_HEADER_I2C1)


def external_motor_driver_addr():
    """Direccion I2C del PCA9685 externo para ser drop-in con el codigo de motores."""
    return _EXTERNAL_MOTOR_DRIVER_ADDR


def directly_wireable():
    """Sensores I2C que se pueden cablear directo al header, con su nombre."""
    return [{"addr": a, "name": HUT_DEVICES[a][0]} for a in _DIRECT_SENSORS]


def wiring_summary():
    """Resumen legible del cableado de bypass."""
    p = _HEADER_I2C1
    sensores = ", ".join(f"{s['name']} (0x{s['addr']:02x})" for s in directly_wireable())
    return (
        f"Bypass de la HUT — cableado directo al header de 40 pines (bus I2C-1):\n"
        f"  SDA -> pin {p['sda']}, SCL -> pin {p['scl']}, "
        f"3V3 -> pin {p['v3v3']}, GND -> pin {p['gnd']}.\n"
        f"  Sensores I2C directos (comparten el bus): {sensores}.\n"
        f"  Motores: requieren DRIVER EXTERNO con puente H. Pon el PCA9685 externo "
        f"en 0x{_EXTERNAL_MOTOR_DRIVER_ADDR:02x} y es drop-in con el stack.\n"
        f"  Camara: CSI, independiente de la HUT (ya funciona)."
    )
