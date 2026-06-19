"""Tests del backend de inferencia agnostico de hardware."""
import numpy as np
from object_detection.inference_backend import select_backend, make_detector


def test_jetson_usa_tensorrt():
    assert select_backend("jetson-nano") == "tensorrt"
    assert select_backend("jetson-orin") == "tensorrt"


def test_pi_y_generico_usan_onnx():
    assert select_backend("raspberry-pi") == "onnx"
    assert select_backend("generic") == "onnx"


def test_prefer_explicito():
    assert select_backend("jetson-nano", prefer="onnx") == "onnx"


class _FakeSession:
    def get_inputs(self):
        class _I: name = "images"
        return [_I()]
    def run(self, _o, _f):
        return [np.zeros((1, 1, 6), dtype=np.float32)]


def test_make_detector_onnx_inyectando_sesion():
    det = make_detector("m.onnx", backend="onnx", session=_FakeSession())
    from object_detection.onnx_inference import ONNXDetector
    assert isinstance(det, ONNXDetector)
