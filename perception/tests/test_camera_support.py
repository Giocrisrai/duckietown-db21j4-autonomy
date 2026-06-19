"""Tests de soporte multi-camara (IMX219/IMX219-B/IMX477/USB) + intrinsecos default."""
import pytest
from object_detection.camera_support import (
    supported_cameras, camera_info, gst_pipeline, default_intrinsics,
)
from object_detection.camera_calibration import validate_intrinsics


def test_camaras_soportadas():
    cams = supported_cameras()
    assert "imx219" in cams and "imx219-b" in cams and "imx477" in cams and "usb" in cams


def test_pipeline_csi_imx219b():
    p = gst_pipeline("imx219-b", sensor_id=0)
    assert "nvarguscamerasrc" in p and "sensor-id=0" in p and "appsink" in p


def test_pipeline_usb_v4l2():
    p = gst_pipeline("usb", device="/dev/video1")
    assert "v4l2src" in p and "/dev/video1" in p


def test_camara_no_soportada():
    with pytest.raises(KeyError):
        camera_info("inventada")


def test_intrinsecos_default_son_validos():
    # los intrinsecos iniciales para recalibrar deben pasar la validacion
    intr = default_intrinsics("imx219-b")
    assert validate_intrinsics(intr) is intr
    assert intr["image_width"] == 1280


# --- Autodeteccion: que el modelo de camara NO sea una limitante ---
from object_detection.camera_support import detect_camera, auto_pipeline
import pytest as _pytest


def test_detecta_imx219_de_dmesg():
    assert detect_camera(dmesg="... tegra-camera imx219 6-0010 ...") == "imx219"


def test_imx219b_se_detecta_como_imx219():
    # la IMX219-B aparece como 'imx219' en dmesg -> queda cubierta
    assert detect_camera(dmesg="imx219 detected on CAM port (board rev B)") == "imx219"


def test_detecta_imx477():
    assert detect_camera(dmesg="imx477 sensor probed") == "imx477"


def test_fallback_usb_si_hay_video_device():
    assert detect_camera(dmesg="", devices=["/dev/video0"]) == "usb"


def test_sin_camara_devuelve_none():
    assert detect_camera(dmesg="", devices=[]) is None


def test_auto_pipeline_adapta_solo():
    model, pipe = auto_pipeline(dmesg="imx219 ready")
    assert model == "imx219" and "nvarguscamerasrc" in pipe


def test_auto_pipeline_sin_camara_lanza():
    with _pytest.raises(RuntimeError):
        auto_pipeline(dmesg="", devices=[])
