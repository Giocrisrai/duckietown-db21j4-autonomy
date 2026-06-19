"""Tests de los chequeos de salud de hardware."""
from health import battery_status, encoders_ok, imu_sane, tof_ok, summarize


def test_bateria():
    assert battery_status(80) == "ok"
    assert battery_status(15) == "bajo"
    assert battery_status(5) == "critico"
    assert battery_status(None) == "desconocido"


def test_encoders_falla_si_no_cuentan_moviendo():
    assert encoders_ok(0, 0, comandado_moviendo=True) is False     # motor/encoder muerto
    assert encoders_ok(5, 4, comandado_moviendo=True) is True
    assert encoders_ok(0, 0, comandado_moviendo=False) is True     # quieto: OK


def test_imu():
    assert imu_sane(9.8) is True
    assert imu_sane(0.0) is False        # IMU caido / sin lectura valida
    assert imu_sane(None) is False


def test_tof():
    assert tof_ok(0.15) is True
    assert tof_ok(5.0) is False          # fuera de rango fisico
    assert tof_ok(None) is False


def test_summarize():
    r = {"battery_percent": 8, "left_delta": 0, "right_delta": 0, "moving": True,
         "accel_magnitude": 9.8, "tof_range": 0.3}
    s = summarize(r)
    assert s["battery"] == "critico"
    assert s["encoders"] is False
    assert s["imu"] is True
    assert s["tof"] is True
