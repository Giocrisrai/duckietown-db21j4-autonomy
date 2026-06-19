"""Tests de carga/guardado/validacion de calibracion cinematica."""
import pytest
from duckietown_planning.calibration import (
    validate_kinematics, load_kinematics, save_kinematics, CalibrationError,
)


def test_defaults_completados():
    full = validate_kinematics({"gain": 1.2})
    assert full["gain"] == 1.2
    assert full["baseline"] == 0.10 and full["radius"] == 0.0318


def test_gain_fuera_de_rango():
    with pytest.raises(CalibrationError):
        validate_kinematics({"gain": 5.0})


def test_trim_fuera_de_rango():
    with pytest.raises(CalibrationError):
        validate_kinematics({"trim": 0.9})


def test_round_trip(tmp_path):
    p = save_kinematics(tmp_path / "kin.yaml", {"gain": 1.1, "trim": 0.05})
    cargado = load_kinematics(p)
    assert cargado["gain"] == 1.1 and cargado["trim"] == 0.05
