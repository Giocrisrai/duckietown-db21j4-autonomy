"""Exporta los pesos YOLO26 entrenados a ONNX, listo para TensorRT 8.2 (Jetson)."""


def build_export_kwargs(imgsz: int = 416, opset: int = 12) -> dict:
    """Construye los parámetros de exportación a ONNX para TensorRT 8.2.

    Args:
        imgsz: Tamaño de imagen (debe ser múltiplo de 32).
        opset: Versión del operador ONNX (debe estar en rango 9-17 para TensorRT 8.2).

    Returns:
        Diccionario con parámetros listos para Ultralytics.

    Raises:
        AssertionError: Si imgsz no es múltiplo de 32 o opset está fuera de rango.
    """
    assert imgsz % 32 == 0, "imgsz debe ser multiplo de 32"
    assert 9 <= opset <= 17, "opset fuera de rango seguro para TensorRT 8.2"
    return dict(format="onnx", imgsz=imgsz, opset=opset, simplify=True, dynamic=False)


def export(weights: str, imgsz: int = 416, opset: int = 12) -> str:
    """Carga los pesos y exporta a ONNX. Devuelve la ruta del .onnx generado.

    Requiere ultralytics instalado (Colab/M1). No se ejecuta en el robot.

    Args:
        weights: Ruta a los pesos en formato .pt de Ultralytics.
        imgsz: Tamaño de imagen para exportación.
        opset: Versión del operador ONNX.

    Returns:
        Ruta al archivo .onnx generado.
    """
    from ultralytics import YOLO

    kwargs = build_export_kwargs(imgsz=imgsz, opset=opset)
    return YOLO(weights).export(**kwargs)
