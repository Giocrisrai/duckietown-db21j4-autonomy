import numpy as np
from sim_perception import make_dummy_onnx, run_perception_sim


def test_sim_percepcion_end_to_end(tmp_path):
    dets = np.array([[[100., 100., 200., 200., 0.9, 0.],
                      [10., 10., 20., 20., 0.1, 1.]]], dtype=np.float32)
    path = make_dummy_onnx(str(tmp_path / "dummy.onnx"), dets)
    img = np.zeros((416, 416, 3), dtype=np.uint8)
    out = run_perception_sim(path, img)
    assert out.shape == (1, 6)      # la de score 0.1 se filtra
    assert out[0, 5] == 0.0         # clase duckie
