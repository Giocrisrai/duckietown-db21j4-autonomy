"""Selecciona el backend de inferencia segun la placa (hardware-agnostico).

- Jetson (Nano/Orin): TensorRT (engine .engine) -> maximo rendimiento con GPU.
- Raspberry Pi / PC: ONNX Runtime (modelo .onnx) -> CPU, sin CUDA.

Asi el mismo modelo entrenado corre en distintas placas; solo cambia el backend.
"""


def select_backend(board=None, prefer="auto"):
    """Devuelve 'tensorrt' u 'onnx' segun la placa (o `prefer` explicito)."""
    if prefer != "auto":
        return prefer
    if board and str(board).startswith("jetson"):
        return "tensorrt"
    return "onnx"


def make_detector(model_path, backend="onnx", **kwargs):
    """Crea el detector adecuado al backend.

    backend='tensorrt' -> TRTDetector (espera un .engine, en la Jetson).
    backend='onnx'     -> ONNXDetector (espera un .onnx, en Pi/CPU).
    Los imports son perezosos para no exigir tensorrt/onnxruntime donde no esten.
    """
    if backend == "tensorrt":
        from .trt_inference import TRTDetector
        return TRTDetector(model_path, **kwargs)
    if backend == "onnx":
        from .onnx_inference import ONNXDetector
        return ONNXDetector(model_path, **kwargs)
    raise ValueError(f"backend de inferencia desconocido: {backend}")
