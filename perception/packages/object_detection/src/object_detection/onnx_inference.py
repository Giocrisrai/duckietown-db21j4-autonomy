"""Inferencia con ONNX Runtime: espejo de TRTDetector para CPU/Mac/Colab.

Permite SIMULAR y VALIDAR el pipeline de deteccion completo
(letterbox -> modelo -> postproceso) SIN TensorRT ni robot. Reutiliza los
modulos ya testeados. La sesion es inyectable para pruebas (session=...).
"""
import numpy as np
from .letterbox import letterbox
from .postprocess import scale_boxes, filter_detections


class ONNXDetector:
    def __init__(self, model_path, imgsz=416, conf_thres=0.25, session=None):
        self.imgsz = imgsz
        self.conf_thres = conf_thres
        if session is not None:
            self.session = session                       # inyectable en tests
        else:
            import onnxruntime as ort
            self.session = ort.InferenceSession(
                model_path, providers=["CPUExecutionProvider"])
        self._input_name = self.session.get_inputs()[0].name

    def _preprocess(self, img_bgr):
        img, r, (left, top) = letterbox(img_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)        # BGR->RGB, HWC->CHW
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob[None], r, (left, top)

    def infer(self, img_bgr):
        """Devuelve detecciones (N,6)=[x1,y1,x2,y2,score,cls] en coords de img_bgr."""
        blob, r, pad = self._preprocess(img_bgr)
        outputs = self.session.run(None, {self._input_name: blob})
        raw = filter_detections(np.asarray(outputs[0]).reshape(-1, 6), self.conf_thres)
        if raw.size == 0:
            return raw
        boxes = scale_boxes(raw[:, :4], r, pad, img_bgr.shape[:2])
        return np.concatenate([boxes, raw[:, 4:6]], axis=1)
