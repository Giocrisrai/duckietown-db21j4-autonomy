import numpy as np
from object_detection.onnx_inference import ONNXDetector


class _FakeSession:
    """Sesion ONNX falsa: devuelve una deteccion fija en espacio letterbox 416."""
    def __init__(self, salida):
        self._salida = salida
    def get_inputs(self):
        class _I: name = "images"
        return [_I()]
    def run(self, _outs, _feeds):
        return [self._salida]


def test_infer_encadena_y_escala():
    # una deteccion (x1,y1,x2,y2,score,cls) en espacio 416 y una bajo umbral
    salida = np.array([[[100., 100., 200., 200., 0.9, 0.],
                        [10., 10., 20., 20., 0.1, 1.]]], dtype=np.float32)
    det = ONNXDetector("dummy.onnx", session=_FakeSession(salida))
    img = np.zeros((416, 416, 3), dtype=np.uint8)   # cuadrada: r=1, sin padding
    out = det.infer(img)
    assert out.shape == (1, 6)          # la de score 0.1 se filtra
    assert out[0, 5] == 0.0             # clase duckie
    assert abs(out[0, 0] - 100.0) < 1e-6  # r=1, pad=0 -> coords iguales
