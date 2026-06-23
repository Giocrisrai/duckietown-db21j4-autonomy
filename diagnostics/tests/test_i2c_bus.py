"""Tests del clasificador de fallas a nivel de BUS I2C (no de dispositivo).

Codifica el caso real del Duckiebot `prueba`: el bus se cuelga en la primera
direccion del escaneo (0x03) en AMBOS buses del header -> linea SDA/SCL tomada en
bajo ("stuck-low"). En ese escenario el reset por software del controlador NO sirve;
la accion correcta es corte total de energia + reasentar la HUT.
"""
from i2c_bus import (
    parse_timeout_addr,
    classify_i2c_fault,
    recommended_steps,
    status_from_returncode,
    FIRST_SCAN_ADDR,
)

# Linea real de dmesg del robot `prueba`.
DMESG_REAL = [
    "[  548.972655] tegra-i2c 7000c000.i2c: --- register dump for debugging ----",
    "[  549.014400] tegra-i2c 7000c000.i2c: i2c transfer timed out addr: 0x3",
]


def test_parse_timeout_addr_encuentra_0x3():
    assert parse_timeout_addr(DMESG_REAL) == 0x03


def test_parse_timeout_addr_sin_evento():
    assert parse_timeout_addr(["[1.0] todo ok", "[2.0] sin timeouts"]) is None


def test_parse_timeout_addr_direccion_alta():
    assert parse_timeout_addr(["x: i2c transfer timed out addr: 0x60"]) == 0x60


def test_stuck_low_ambos_buses():
    """Caso `prueba`: ambos buses del header colgados + timeout en 0x03."""
    diag = classify_i2c_fault({0: "hung", 1: "hung"}, DMESG_REAL)
    assert diag["fault"] == "stuck_low"
    assert diag["both_header_buses_hung"] is True
    assert diag["timeout_addr"] == FIRST_SCAN_ADDR
    # El reset por software NO ayuda con una linea tomada en bajo.
    assert diag["software_reset_useful"] is False
    assert diag["action"] == "power_cycle_reseat"


def test_bus_sano():
    diag = classify_i2c_fault({0: "ok", 1: "ok"}, [])
    assert diag["fault"] == "none"
    assert diag["both_header_buses_hung"] is False
    assert diag["action"] == "none"


def test_controlador_colgado_un_bus_sin_stuck_low():
    """Un solo bus colgado y sin timeout en la primera direccion -> probable
    cuelgue del controlador; ahi SI vale la pena el reset por software."""
    diag = classify_i2c_fault({0: "ok", 1: "hung"}, [])
    assert diag["fault"] == "controller"
    assert diag["software_reset_useful"] is True
    assert diag["action"] == "controller_reset"


def test_un_bus_stuck_low_tambien_es_fisico():
    """Aunque solo cuelgue un bus, si el timeout es en 0x03 es linea tomada."""
    diag = classify_i2c_fault({0: "ok", 1: "hung"}, DMESG_REAL)
    assert diag["fault"] == "stuck_low"
    assert diag["software_reset_useful"] is False


def test_recommended_steps_stuck_low_no_menciona_reset_software_como_fix():
    diag = classify_i2c_fault({0: "hung", 1: "hung"}, DMESG_REAL)
    pasos = recommended_steps(diag)
    assert isinstance(pasos, list) and len(pasos) >= 2
    texto = " ".join(pasos).lower()
    assert "energ" in texto or "bateria" in texto    # corte total de energia
    assert "hut" in texto                              # reasentar HUT


def test_recommended_steps_controlador_sugiere_reset():
    diag = classify_i2c_fault({0: "ok", 1: "hung"}, [])
    texto = " ".join(recommended_steps(diag)).lower()
    assert "reset" in texto and "unbind" in texto


def test_stuck_low_barrido_de_frontera():
    """El umbral stuck-low es addr <= FIRST_SCAN_ADDR+5 (0x08). Barremos la frontera:
    0x03..0x08 -> stuck_low; 0x09 -> ya NO es stuck_low (controlador)."""
    for addr in range(0x03, 0x09):               # 0x03..0x08 inclusive
        dmesg = [f"tegra-i2c 7000c400.i2c: i2c transfer timed out addr: 0x{addr:x}"]
        d = classify_i2c_fault({1: "hung"}, dmesg)
        assert d["fault"] == "stuck_low", f"0x{addr:02x} deberia ser stuck_low"
    # 0x09 ya esta fuera del rango de 'primera direccion' -> controlador.
    d09 = classify_i2c_fault({1: "hung"},
                             ["tegra-i2c 7000c400.i2c: i2c transfer timed out addr: 0x9"])
    assert d09["fault"] == "controller"
    assert d09["software_reset_useful"] is True


def test_status_from_returncode():
    """i2cdetect: rc 0 = bus respondio; rc!=0 (124 timeout) = colgado."""
    assert status_from_returncode(0) == "ok"
    assert status_from_returncode(124) == "hung"     # timeout (coreutils)
    assert status_from_returncode(1) == "hung"
