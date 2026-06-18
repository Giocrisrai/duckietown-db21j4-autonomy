"""Simulacion de percepcion end-to-end con ONNX Runtime (sin robot, sin entrenar).

Genera un modelo ONNX sintetico que emite detecciones fijas y lo corre con la
misma clase de inferencia del proyecto (ONNXDetector), validando toda la ruta
letterbox -> ONNX Runtime -> postproceso en la Mac.
"""
import numpy as np
from object_detection.onnx_inference import ONNXDetector


def make_dummy_onnx(path, dets):
    """Crea un ONNX que ignora la entrada y emite `dets` (1,N,6) como salida."""
    import onnx
    from onnx import helper, TensorProto

    dets = np.asarray(dets, dtype=np.float32)

    # Define input y output de la red
    inp = helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 416, 416])
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, list(dets.shape))

    # Nodo Constant que emite las detecciones, ignora la entrada
    const = helper.make_node(
        "Constant", [], ["output"],
        value=helper.make_tensor("c", TensorProto.FLOAT, dets.shape, dets.flatten().tolist())
    )

    # Crea el grafo
    graph = helper.make_graph([const], "dummy", [inp], [out])

    # Crea el modelo con opset 12
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 12)])

    # Guarda el modelo
    onnx.save(model, path)

    return path


def run_perception_sim(model_path, img):
    """Corre ONNXDetector (onnxruntime real) sobre `img`. Devuelve detecciones (N,6)."""
    det = ONNXDetector(model_path)
    return det.infer(img)
