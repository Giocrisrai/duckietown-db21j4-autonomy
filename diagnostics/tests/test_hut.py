"""Tests de deteccion del MCU serial de la HUT (DB21-J4)."""
from hut import is_hut_device, find_hut_serial, HUT_USB_VENDOR, HUT_USB_PRODUCT


def test_identifica_mcu_hut():
    assert is_hut_device("04d8", "ecb7") is True
    assert is_hut_device("0x04d8", "0xecb7") is True   # con prefijo 0x


def test_ignora_otros_usb():
    assert is_hut_device("0bda", "5411") is False      # hub Realtek


def test_find_hut_serial():
    devs = [
        {"path": "/dev/ttyUSB0", "vendor": "10c4", "product": "ea60"},
        {"path": "/dev/ttyACM0", "vendor": "04d8", "product": "ecb7"},
    ]
    assert find_hut_serial(devs) == "/dev/ttyACM0"


def test_find_hut_serial_ausente():
    assert find_hut_serial([{"path": "/dev/ttyUSB0", "vendor": "10c4", "product": "ea60"}]) is None
