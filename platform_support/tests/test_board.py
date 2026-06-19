"""Tests de deteccion/configuracion de placa (hardware-agnostico)."""
import pytest
from board import detect_board, board_config, inference_backend, BOARDS


def test_detecta_jetson_nano():
    assert detect_board("NVIDIA Jetson Nano Developer Kit") == "jetson-nano"


def test_detecta_jetson_orin():
    assert detect_board("NVIDIA Jetson Orin Nano") == "jetson-orin"


def test_detecta_raspberry():
    assert detect_board("Raspberry Pi 4 Model B Rev 1.4") == "raspberry-pi"


def test_generic_por_defecto():
    assert detect_board("Some x86 PC") == "generic"


def test_inference_backend_por_placa():
    assert inference_backend("jetson-nano") == "tensorrt"
    assert inference_backend("raspberry-pi") == "onnx"
    assert inference_backend("generic") == "onnx"


def test_placa_no_soportada():
    with pytest.raises(KeyError):
        board_config("commodore-64")
