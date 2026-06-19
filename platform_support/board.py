"""Deteccion y configuracion de la PLACA de computo.

Hace el stack agnostico del hardware: misma autonomia en Jetson Nano/Orin o
Raspberry Pi. Cada placa define su backend de inferencia, tipo de camara, arch e
imagen base de despliegue.
"""

BOARDS = {
    "jetson-nano": {
        "inference": "tensorrt", "camera": "csi-nvargus", "arch": "arm64",
        "base_image": "duckietown/dt-ros-commons", "nota": "Jetson Nano (TensorRT, CUDA 10.2)",
    },
    "jetson-orin": {
        "inference": "tensorrt", "camera": "csi-nvargus", "arch": "arm64",
        "base_image": "duckietown/dt-ros-commons", "nota": "Jetson Orin (TensorRT, mucho mas potente)",
    },
    "raspberry-pi": {
        "inference": "onnx", "camera": "csi-libcamera", "arch": "arm64",
        "base_image": "ros:noetic-ros-base", "nota": "Raspberry Pi 4/5 (ONNX Runtime CPU; opcional Coral TPU)",
    },
    "generic": {
        "inference": "onnx", "camera": "usb", "arch": "amd64",
        "base_image": "ros:noetic-ros-base", "nota": "PC/Linux generico (ONNX Runtime, camara USB)",
    },
}


def detect_board(model_str="", machine=""):
    """Detecta la placa desde /proc/device-tree/model (model_str) o uname -m (machine)."""
    s = (model_str or "").lower()
    if "orin" in s:
        return "jetson-orin"
    if "jetson" in s or "tegra" in s or "nvidia" in s:
        return "jetson-nano"
    if "raspberry" in s:
        return "raspberry-pi"
    return "generic"


def board_config(board):
    """Configuracion de la placa. Lanza KeyError si no esta soportada."""
    if board not in BOARDS:
        raise KeyError(f"placa no soportada: {board}. Opciones: {sorted(BOARDS)}")
    return BOARDS[board]


def inference_backend(board):
    """Backend de inferencia recomendado para la placa (tensorrt / onnx)."""
    return board_config(board)["inference"]
