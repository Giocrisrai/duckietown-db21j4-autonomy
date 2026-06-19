"""Deteccion del microcontrolador de la HUT del Duckiebot DB21-J4 (USB-serial).

HALLAZGO (hardware real): en el DB21-J4 los MOTORES, ENCODERS y LEDs los maneja un
microcontrolador Atmel/Microchip que expone un puerto serie virtual (CDC ACM,
/dev/ttyACM*) por USB -> NO por I2C. dt-duckiebot-interface habla con el por ahi.
Por eso los buses I2C estaban vacios para motores: el control va por serial.
"""

# USB del MCU de la HUT (Atmel/Microchip "Virtual COM-Port").
HUT_USB_VENDOR = "04d8"     # Microchip Technology / Atmel
HUT_USB_PRODUCT = "ecb7"


def _norm(x):
    return str(x).lower().replace("0x", "").lstrip("0") or "0"


def is_hut_device(vendor_id, product_id):
    """True si el (vendor, product) USB corresponde al MCU de la HUT del DB21-J4."""
    return (_norm(vendor_id) == _norm(HUT_USB_VENDOR)
            and _norm(product_id) == _norm(HUT_USB_PRODUCT))


def find_hut_serial(devices):
    """Dada una lista de dicts {path, vendor, product}, devuelve el puerto serie de
    la HUT (p.ej. '/dev/ttyACM0') o None si no esta."""
    for d in devices:
        if is_hut_device(d.get("vendor", ""), d.get("product", "")):
            return d.get("path")
    return None
