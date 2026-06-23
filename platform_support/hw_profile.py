"""Perfil de hardware: UN solo lugar que define el bus I2C y las direcciones de
motores/LEDs/sensores. Hace la arquitectura agnostica al cableado, para que la
MISMA logica de motores/sensores funcione sobre la HUT o sobre un driver externo
en otro puerto/direccion (bypass de un PCA9685 danado).

Se elige con el argumento `name` o la variable de entorno `DT_HW_PROFILE`.
Perfiles:
  - "hut"           DB21-J4 estandar: bus 1, motores 0x60, LEDs 0x40, IMU 0x68,
                    ToF 0x29, OLED 0x3c.
  - "external-bus1" El PCA9685 de la HUT (0x60) murio pero el BUS 1 vive (la IMU
                    responde). Se cuelga un PCA9685 EXTERNO en 0x61 (esquiva el
                    chip muerto) en los MISMOS pines 3/5. IMU de la HUT (0x68) se
                    reutiliza.
  - "external-gpio" Bus i2c-gpio NUEVO (bit-bang en pines GPIO libres, via overlay)
                    totalmente independiente de la HUT; PCA9685 externo en 0x60.
                    El numero de bus lo asigna el overlay (ajustable con
                    DT_HW_GPIO_BUS, por defecto 6).
"""
import os

DEFAULT_PROFILE = "hut"


def _gpio_bus():
    try:
        return int(os.environ.get("DT_HW_GPIO_BUS", "6"))
    except ValueError:
        return 6


def _profiles():
    return {
        "hut": {
            "motor_bus": 1, "motor_addr": 0x60, "led_addr": 0x40,
            "imu": 0x68, "tof": 0x29, "oled": 0x3c,
        },
        "external-bus1": {
            "motor_bus": 1, "motor_addr": 0x61, "led_addr": 0x41,
            "imu": 0x68, "tof": 0x29, "oled": 0x3c,   # IMU del HUT se reutiliza
        },
        "external-gpio": {
            "motor_bus": _gpio_bus(), "motor_addr": 0x60, "led_addr": 0x40,
            "imu": 0x68, "tof": 0x29, "oled": 0x3c,
        },
    }


def list_profiles():
    """Nombres de perfiles disponibles."""
    return sorted(_profiles())


def active_profile(name=None):
    """Perfil activo. Prioridad: arg `name` > env DT_HW_PROFILE > DEFAULT_PROFILE.
    Devuelve un dict con el campo extra 'name'. Lanza KeyError si no existe."""
    chosen = name or os.environ.get("DT_HW_PROFILE") or DEFAULT_PROFILE
    profs = _profiles()
    if chosen not in profs:
        raise KeyError(f"perfil de hardware desconocido: {chosen}. Opciones: {list_profiles()}")
    return {"name": chosen, **profs[chosen]}


def motor_target(name=None):
    """(bus, addr) del driver de motores en el perfil activo."""
    p = active_profile(name)
    return p["motor_bus"], p["motor_addr"]


def sensor_addr(which, name=None):
    """Direccion I2C de un sensor ('imu'|'tof'|'oled'|'led') en el perfil activo."""
    p = active_profile(name)
    key = "led_addr" if which == "led" else which
    return p[key]
