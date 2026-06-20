"""Tests de calibracion de camara (intrinsecos formato CameraInfo)."""
import pytest
import numpy as np
from object_detection.camera_calibration import (
    validate_intrinsics, camera_matrix, dist_coeffs, CameraCalibrationError,
    save_intrinsics, load_intrinsics,
)

SAMPLE = {
    "image_width": 640,
    "image_height": 480,
    "camera_matrix": {"rows": 3, "cols": 3, "data": [300, 0, 320, 0, 300, 240, 0, 0, 1]},
    "distortion_coefficients": {"rows": 1, "cols": 5, "data": [-0.2, 0.05, 0.0, 0.0, 0.0]},
}


def test_valida_intrinsecos_ok():
    assert validate_intrinsics(SAMPLE) is SAMPLE


def test_camera_matrix_3x3():
    K = camera_matrix(SAMPLE)
    assert K.shape == (3, 3)
    assert K[0, 0] == 300 and K[0, 2] == 320


def test_dist_coeffs():
    D = dist_coeffs(SAMPLE)
    assert D.shape == (5,)
    assert D[0] == -0.2


def test_falta_clave():
    with pytest.raises(CameraCalibrationError):
        validate_intrinsics({"image_width": 640})


def test_camera_matrix_mal_dimensionada():
    bad = dict(SAMPLE, camera_matrix={"rows": 2, "cols": 2, "data": [1, 2, 3, 4]})
    with pytest.raises(CameraCalibrationError):
        validate_intrinsics(bad)


def test_save_load_roundtrip(tmp_path):
    dest = tmp_path / "camera_intrinsic" / "prueba.yaml"
    save_intrinsics(dest, SAMPLE)
    assert dest.exists()
    cargado = load_intrinsics(dest)
    assert cargado["image_width"] == 640
    assert camera_matrix(cargado)[0, 0] == 300


def test_save_valida_antes_de_escribir(tmp_path):
    dest = tmp_path / "mala.yaml"
    with pytest.raises(CameraCalibrationError):
        save_intrinsics(dest, {"image_width": 640})
    assert not dest.exists()
