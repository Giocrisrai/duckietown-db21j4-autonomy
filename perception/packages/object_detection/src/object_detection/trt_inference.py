# perception/packages/object_detection/src/object_detection/trt_inference.py
"""Inferencia TensorRT para YOLO26 en la Jetson Nano (CUDA 10.2, TensorRT 8.2).

Solo se ejecuta en la Nana (requiere tensorrt + pycuda). El pre/post-proceso
reutiliza modulos testeados en la Mac (letterbox, postprocess).
"""
import numpy as np
from .letterbox import letterbox
from .postprocess import scale_boxes, filter_detections

class TRTDetector:
    def __init__(self, engine_path: str, imgsz: int = 416, conf_thres: float = 0.25):
        import tensorrt as trt
        import pycuda.autoinit  # noqa: F401  (inicializa el contexto CUDA)
        import pycuda.driver as cuda

        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self._cuda = cuda
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

    def _preprocess(self, img_bgr):
        img, r, (left, top) = letterbox(img_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)            # BGR->RGB, HWC->CHW
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob[None], r, (left, top)

    def infer(self, img_bgr) -> np.ndarray:
        """Devuelve detecciones (N,6)=[x1,y1,x2,y2,score,cls] en coords de img_bgr."""
        blob, r, pad = self._preprocess(img_bgr)
        raw = self._run(blob)                                # (N,6) espacio letterbox
        raw = filter_detections(raw, self.conf_thres)
        if raw.size == 0:
            return raw
        boxes = scale_boxes(raw[:, :4], r, pad, img_bgr.shape[:2])
        return np.concatenate([boxes, raw[:, 4:6]], axis=1)

    def _run(self, blob) -> np.ndarray:
        """Ejecuta el engine. Implementacion con buffers pycuda."""
        cuda = self._cuda
        inp = self.engine.get_binding_shape(0)
        out_idx = 1
        out_shape = tuple(self.context.get_binding_shape(out_idx))
        out_host = np.empty(out_shape, dtype=np.float32)
        d_in = cuda.mem_alloc(blob.nbytes)
        d_out = cuda.mem_alloc(out_host.nbytes)
        cuda.memcpy_htod(d_in, np.ascontiguousarray(blob))
        self.context.execute_v2([int(d_in), int(d_out)])
        cuda.memcpy_dtoh(out_host, d_out)
        return out_host.reshape(-1, 6)
