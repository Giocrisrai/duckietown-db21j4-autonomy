"""Logica pura para grabar un dataset de frames (sin tocar la camara).

La captura real (cv2/gstreamer) vive en `tools/record_dataset.py`; aqui van los
helpers testeables: nombres de frame, validacion del plan y manifiesto.
"""


def frame_filename(prefix, index, ext="jpg"):
    """Nombre de archivo con indice zero-padded a 5 digitos."""
    return f"{prefix}_{index:05d}.{ext}"


def capture_plan(count, interval_s):
    """Valida y describe un plan de captura.

    count >= 1, interval_s >= 0. Devuelve dict con la duracion estimada
    ((count-1)*interval_s, porque el primer frame es inmediato)."""
    if count < 1:
        raise ValueError("count debe ser >= 1")
    if interval_s < 0:
        raise ValueError("interval_s no puede ser negativo")
    return {
        "count": count,
        "interval_s": interval_s,
        "duracion_estimada_s": (count - 1) * interval_s,
    }


def manifest(out_dir, frames, model, width, height):
    """Manifiesto del dataset grabado (serializable a JSON)."""
    return {
        "out_dir": out_dir,
        "model": model,
        "width": width,
        "height": height,
        "count": len(frames),
        "frames": list(frames),
    }
