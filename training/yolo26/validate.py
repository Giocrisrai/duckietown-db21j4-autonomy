"""Validacion del modelo YOLO26 entrenado: metricas (mAP) + chequeo de umbral."""

MIN_MAP50 = 0.70  # umbral razonable para competencia


def passes_threshold(map50, min_map50: float = MIN_MAP50) -> bool:
    """True si el mAP50 alcanza el umbral minimo.

    Args:
        map50: Valor de mAP50 a validar.
        min_map50: Umbral minimo de mAP50 (por defecto 0.70).

    Returns:
        bool: True si map50 >= min_map50, False en caso contrario.
    """
    return float(map50) >= float(min_map50)


def validate(weights: str, data: str, imgsz: int = 416) -> float:
    """Corre Ultralytics val y devuelve el mAP50.

    Esta funcion requiere ultralytics instalado (disponible en Colab/M1 con soporte).
    Utiliza import perezoso para permitir que el modulo sea importable sin ultralytics.

    Args:
        weights: Ruta al archivo de pesos del modelo YOLO.
        data: Ruta al archivo de configuracion del dataset.
        imgsz: Tamaño de imagen para validacion (por defecto 416).

    Returns:
        float: Valor de mAP50 del modelo validado.
    """
    from ultralytics import YOLO
    metrics = YOLO(weights).val(data=data, imgsz=imgsz)
    return float(metrics.box.map50)
