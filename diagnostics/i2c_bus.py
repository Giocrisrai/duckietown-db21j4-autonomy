"""Clasificacion de fallas a nivel de BUS I2C del Jetson Nano (Tegra X1).

A diferencia de `i2c_sensors.py` (mapa de DISPOSITIVOS del HUT), este modulo
diagnostica el BUS: distingue un cuelgue del CONTROLADOR (que el reset por
software puede liberar) de una LINEA tomada en bajo ("stuck-low"), donde el reset
no sirve y la unica salida es corte total de energia + reasentar/cambiar la HUT.

Firma del caso real (`prueba`, DB21-J4):
- `i2cdetect` se cuelga en la PRIMERA direccion del escaneo (0x03), y
- dmesg: `tegra-i2c ...: i2c transfer timed out addr: 0x3`, y
- ambos buses del header (i2c-0 pines 27/28, i2c-1 pines 3/5) caen a la vez.
Eso = SDA o SCL forzada a 0 por un esclavo latcheado / pin en corto / HUT en falla.

Funciones puras y testeables (sin tocar hardware).
"""
import re

# i2cdetect arranca el escaneo en 0x03 por defecto. Un timeout AHI = la linea
# esta tomada antes de poder hablar con nada: no es un dispositivo puntual.
FIRST_SCAN_ADDR = 0x03

# Buses del header de 40 pines del Jetson Nano (los unicos que llegan al HUT).
HEADER_BUSES = (0, 1)

_TIMEOUT_RE = re.compile(r"i2c transfer timed out addr:\s*0x([0-9a-fA-F]+)")


def parse_timeout_addr(dmesg_lines):
    """Devuelve la direccion (int) del ultimo 'i2c transfer timed out addr: 0xNN'
    en el dmesg, o None si no hay ninguno."""
    addr = None
    for line in dmesg_lines:
        m = _TIMEOUT_RE.search(line)
        if m:
            addr = int(m.group(1), 16)
    return addr


def classify_i2c_fault(bus_status, dmesg_lines):
    """Clasifica la falla del bus I2C.

    bus_status: dict {bus_num: 'ok'|'hung'} para los buses del header.
    dmesg_lines: lineas de dmesg (para buscar la direccion del timeout).

    Devuelve un dict con:
      fault: 'none' | 'stuck_low' | 'controller'
      header_buses_hung: lista ordenada de buses del header colgados
      both_header_buses_hung: bool
      timeout_addr: int | None
      software_reset_useful: bool   (si el unbind/bind del controlador puede ayudar)
      action: 'none' | 'controller_reset' | 'power_cycle_reseat'
      summary: str legible
    """
    hung = sorted(b for b in HEADER_BUSES if bus_status.get(b) == "hung")
    both = set(hung) == set(HEADER_BUSES)
    timeout_addr = parse_timeout_addr(dmesg_lines)

    if not hung:
        fault, action, reset_ok = "none", "none", False
        summary = "Bus I2C sano: ningun bus del header colgado."
    elif timeout_addr is not None and timeout_addr <= FIRST_SCAN_ADDR + 0x05:
        # Timeout en (o muy cerca de) la primera direccion -> linea tomada en bajo.
        fault, action, reset_ok = "stuck_low", "power_cycle_reseat", False
        donde = "ambos buses del header" if both else f"bus {hung[0]}"
        summary = (
            f"Linea SDA/SCL tomada en bajo: timeout en 0x{timeout_addr:02x} "
            f"(primera direccion) en {donde}. El reset por software NO libera la "
            f"linea; requiere corte total de energia y reasentar/cambiar la HUT."
        )
    else:
        # Bus colgado pero sin firma de stuck-low -> probable cuelgue del controlador.
        fault, action, reset_ok = "controller", "controller_reset", True
        summary = (
            f"Bus {','.join(map(str, hung))} colgado sin firma de linea-en-bajo: "
            f"probable cuelgue del controlador tegra-i2c; intentar reset por software."
        )

    return {
        "fault": fault,
        "header_buses_hung": hung,
        "both_header_buses_hung": both,
        "timeout_addr": timeout_addr,
        "software_reset_useful": reset_ok,
        "action": action,
        "summary": summary,
    }


def recommended_steps(diag):
    """Pasos de remediacion ordenados segun la clasificacion."""
    action = diag["action"]
    if action == "none":
        return ["Bus I2C operativo: re-escanear con `i2cdetect -y -r 1` para confirmar dispositivos."]
    if action == "power_cycle_reseat":
        return [
            "Apagar y DESCONECTAR la bateria por completo (~30 s); el corte total "
            "resetea cualquier chip del HUT latcheado que mantenga SDA/SCL en bajo.",
            "Reasentar la HUT en el header de 40 pines (derecho y a ras); revisar "
            "pines 3/5 (SDA/SCL) y 27/28 por pin doblado, suciedad o corto.",
            "Encender una sola vez y re-escanear el bus 1 (`i2cdetect -y -r 1`).",
            "Si sigue colgado con el header bien puesto: probar OTRA HUT/bateria; "
            "el fallo esta en la placa o un chip I2C (PCA9685/IMU/ToF), no en el montaje.",
        ]
    if action == "controller_reset":
        return [
            "Reset del controlador por software: "
            "`echo 7000c400.i2c | sudo tee /sys/bus/platform/drivers/tegra-i2c/unbind` "
            "y luego `.../bind`.",
            "Re-escanear (`sudo timeout 10 i2cdetect -y -r 1`). Si responde, era el "
            "controlador. Si sigue colgado, tratar como linea tomada (power_cycle_reseat).",
        ]
    return []
