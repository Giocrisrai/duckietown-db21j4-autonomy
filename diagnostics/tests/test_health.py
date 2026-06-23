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


def test_bateria_fronteras_exactas():
    """Limites exactos: critical=10 y low=20 son <= (un off-by-one se detecta aqui)."""
    assert battery_status(10.0) == "critico"     # == critical -> critico (<=)
    assert battery_status(10.01) == "bajo"       # justo por encima
    assert battery_status(20.0) == "bajo"        # == low -> bajo (<=)
    assert battery_status(20.01) == "ok"         # justo por encima


def test_imu_fronteras_tolerancia():
    """El borde es inclusivo (<=). Usamos g/tol exactos en float (10.0/3.0) para
    que la frontera 13.0 y 7.0 sea exacta y se distinga <= de <."""
    assert imu_sane(13.0, g=10.0, tol=3.0) is True     # diff == tol -> incluido
    assert imu_sane(13.001, g=10.0, tol=3.0) is False  # apenas afuera
    assert imu_sane(7.0, g=10.0, tol=3.0) is True       # diff == tol (otro lado)
    assert imu_sane(6.999, g=10.0, tol=3.0) is False


def test_tof_fronteras_rango():
    """Rango fisico [0.02, 1.2] inclusive."""
    assert tof_ok(0.02) is True
    assert tof_ok(0.019) is False
    assert tof_ok(1.2) is True
    assert tof_ok(1.21) is False


def test_summarize():
    r = {"battery_percent": 8, "left_delta": 0, "right_delta": 0, "moving": True,
         "accel_magnitude": 9.8, "tof_range": 0.3}
    s = summarize(r)
    assert s["battery"] == "critico"
    assert s["encoders"] is False
    assert s["imu"] is True
    assert s["tof"] is True
