"""Soporte MULTI-CAMARA para el Duckiebot/Jetson.

Permite usar distintos sensores (IMX219, IMX219-B, IMX477) y un fallback USB/UVC,
generando el pipeline GStreamer correcto y unos intrinsecos iniciales para
recalibrar. Asi siempre hay alternativa si una camara no integra.

IMX219-B: es una variante de PLACA del sensor IMX219. Suele requerir el overlay
de device-tree correcto (via `jetson-io`) y el `sensor-id` adecuado del puerto CSI.
Ver docs/camaras.md para el procedimiento de integracion/recalibracion.
"""

CAMERAS = {
    "imx219": {
        "tipo": "csi", "width": 1280, "height": 720, "fps": 30, "flip": 0,
        "nota": "Sony IMX219 (Raspberry Pi Cam v2 / CSI estandar Duckietown)",
    },
    "imx219-b": {
        "tipo": "csi", "width": 1280, "height": 720, "fps": 30, "flip": 0,
        "nota": "Variante de placa del IMX219; requiere overlay device-tree (jetson-io) y sensor-id correcto",
    },
    "imx477": {
        "tipo": "csi", "width": 1920, "height": 1080, "fps": 30, "flip": 0,
        "nota": "Sony IMX477 (HQ Camera)",
    },
    "raspberry": {
        "tipo": "libcamera", "width": 1280, "height": 720, "fps": 30,
        "nota": "Raspberry Pi Camera (libcamera) - placa alternativa",
    },
    "usb": {
        "tipo": "v4l2", "width": 640, "height": 480, "fps": 30,
        "nota": "Camara USB/UVC generica (fallback universal)",
    },
}


def supported_cameras():
    """Lista de identificadores de camara soportados."""
    return sorted(CAMERAS)


def camera_info(sensor):
    """Propiedades del sensor. Lanza KeyError si no esta soportado."""
    if sensor not in CAMERAS:
        raise KeyError(f"camara no soportada: {sensor}. Opciones: {supported_cameras()}")
    return CAMERAS[sensor]


def gst_pipeline(sensor, width=None, height=None, fps=None, sensor_id=0, device="/dev/video0"):
    """Devuelve el pipeline GStreamer correcto para el sensor (CSI nvargus o V4L2)."""
    info = camera_info(sensor)
    w = width or info["width"]
    h = height or info["height"]
    f = fps or info["fps"]
    if info["tipo"] == "csi":
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM),width={w},height={h},framerate={f}/1 ! "
            f"nvvidconv flip-method={info.get('flip', 0)} ! video/x-raw,format=BGRx ! "
            f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
        )
    if info["tipo"] == "libcamera":               # Raspberry Pi Camera
        return (
            f"libcamerasrc ! video/x-raw,width={w},height={h},framerate={f}/1 ! "
            f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
        )
    return (
        f"v4l2src device={device} ! video/x-raw,width={w},height={h},framerate={f}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
    )


# Orden de preferencia de sensores CSI conocidos (imx219 cubre tambien imx219-b).
_CSI_SENSORS = ["imx477", "imx219"]


def detect_camera(dmesg="", devices=()):
    """Detecta AUTOMATICAMENTE el modelo de camara conectado.

    Asi el modelo no es una limitante: el dispositivo se adapta a la camara que
    tenga. `dmesg`: salida de `dmesg` (str, para sensores CSI). `devices`: lista
    de /dev/video* presentes (para USB/UVC). Devuelve el id de modelo soportado,
    o None si no se detecta ninguna camara.
    """
    text = (dmesg or "").lower()
    for sensor in _CSI_SENSORS:
        if sensor in text:                 # imx219-b aparece como 'imx219'
            return sensor
    if devices:                            # hay /dev/video* -> USB/UVC
        return "usb"
    return None


def auto_pipeline(dmesg="", devices=(), **kwargs):
    """Detecta la camara y devuelve (modelo, pipeline_gstreamer) listo para usar.

    Lanza RuntimeError si no hay camara. El resto del stack no necesita saber
    que modelo es: se adapta solo.
    """
    model = detect_camera(dmesg, devices)
    if model is None:
        raise RuntimeError("no se detecto ninguna camara (revisa overlay CSI o conecta USB)")
    return model, gst_pipeline(model, **kwargs)


def default_intrinsics(sensor, width=None, height=None):
    """Intrinsecos INICIALES para recalibrar (focal ~= ancho, centro en el medio).

    Devuelve un dict valido para object_detection.camera_calibration.
    Es solo un punto de partida; la calibracion real con tablero lo afina.
    """
    info = camera_info(sensor)
    w = width or info["width"]
    h = height or info["height"]
    fx = fy = float(w)
    return {
        "image_width": w,
        "image_height": h,
        "camera_matrix": {"rows": 3, "cols": 3, "data": [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]},
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.0, 0.0, 0.0, 0.0, 0.0]},
    }
