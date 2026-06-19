"""Mapa I2C de los sensores del Duckiebot DB21 (HUT) e identificacion/validacion.

Permite probar cada sensor por I2C (presencia + WHO_AM_I) sin el stack ROS completo.
Las funciones son puras/testeables; la lectura real usa smbus (robot-side).
"""

# direccion I2C -> (nombre, registro WHO_AM_I/ID, valor esperado | None)
HUT_DEVICES = {
    0x68: ("MPU6050 (IMU)", 0x75, 0x68),        # WHO_AM_I = 0x68
    0x29: ("VL53L0X (ToF)", 0xC0, 0xEE),        # model id reg 0xC0 = 0xEE
    0x40: ("PCA9685 PWM (motores)", None, None),
    0x60: ("Motor HAT", None, None),
    0x3C: ("SSD1306 (OLED)", None, None),
}


def expected_addresses():
    """Direcciones I2C esperadas del HUT del Duckiebot."""
    return sorted(HUT_DEVICES)


def identify(addr):
    """Nombre del dispositivo del HUT en esa direccion, o 'desconocido'."""
    return HUT_DEVICES.get(addr, ("desconocido", None, None))[0]


def whoami_register(addr):
    """Registro WHO_AM_I/ID a leer para esa direccion (o None si no aplica)."""
    info = HUT_DEVICES.get(addr)
    return info[1] if info else None


def whoami_ok(addr, value):
    """True/False si el valor WHO_AM_I leido coincide con el esperado;
    None si esa direccion no se valida por WHO_AM_I."""
    info = HUT_DEVICES.get(addr)
    if not info or info[2] is None:
        return None
    return value == info[2]
