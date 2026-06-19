"""Tests del monitoreo en vivo de salud de hardware."""
import json
from monitor import HealthMonitor
from monitor_server import health_payload

GOOD = {"battery_percent": 80, "left_delta": 5, "right_delta": 5, "moving": True,
        "accel_magnitude": 9.8, "tof_range": 0.5}
BAD_ENC = {"battery_percent": 80, "left_delta": 0, "right_delta": 0, "moving": True,
           "accel_magnitude": 9.8, "tof_range": 0.5}


def test_todo_bien_es_ok():
    m = HealthMonitor()
    r = m.update(GOOD)
    assert r["overall"] == "OK"


def test_una_lectura_mala_no_es_falla_aun():
    m = HealthMonitor(fault_threshold=3)
    r = m.update(BAD_ENC)               # 1 mala -> degraded, no fault (debounce)
    assert r["overall"] == "DEGRADED"
    assert r["new_faults"] == []


def test_fallas_consecutivas_declaran_fault():
    m = HealthMonitor(fault_threshold=3)
    m.update(BAD_ENC); m.update(BAD_ENC)
    r = m.update(BAD_ENC)              # 3a consecutiva -> FAULT
    assert r["overall"] == "FAULT"
    assert "encoders" in r["new_faults"]


def test_recuperacion_resetea():
    m = HealthMonitor(fault_threshold=2)
    m.update(BAD_ENC); m.update(BAD_ENC)   # FAULT
    r = m.update(GOOD)                      # se recupera
    assert r["overall"] == "OK"


def test_payload_serializable_por_red():
    m = HealthMonitor()
    m.update(GOOD)
    p = health_payload(m)
    assert p["overall"] == "OK" and "components" in p
    json.dumps(p)                          # debe ser serializable a JSON
