#!/usr/bin/env python3
"""Diagnostico AUTOMATICO del bus I2C del Duckiebot (correr en el robot/Jetson).

Escanea los buses del header (0 y 1) con `i2cdetect` (envuelto en `timeout` por si
el bus esta colgado en estado no-interrumpible), lee el `dmesg`, y aplica el
clasificador `diagnostics/i2c_bus.py` para dar el veredicto y los pasos a seguir:

    sudo python3 tools/diagnose_i2c.py

Distingue una linea tomada en bajo ("stuck_low" -> corte de energia + reasentar/
cambiar HUT) de un cuelgue del controlador ("controller" -> reset por software).
"""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "diagnostics"))
from i2c_bus import (                       # noqa: E402
    classify_i2c_fault,
    recommended_steps,
    status_from_returncode,
    HEADER_BUSES,
)


def scan_bus(bus, timeout_s=9):
    """Corre `i2cdetect -y -r <bus>` con timeout duro; devuelve 'ok'|'hung'."""
    try:
        rc = subprocess.call(
            ["timeout", "-k", "3", str(timeout_s), "i2cdetect", "-y", "-r", str(bus)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        print("  Falta i2cdetect. Instala: sudo apt-get install -y i2c-tools")
        sys.exit(2)
    return status_from_returncode(rc)


def read_dmesg():
    """Lineas de dmesg (vacio si no hay permiso)."""
    try:
        out = subprocess.run(["dmesg"], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, check=False)
        return out.stdout.decode("utf-8", "replace").splitlines()
    except Exception:
        return []


def main():
    print("== Diagnostico I2C (buses del header) ==")
    bus_status = {}
    for bus in HEADER_BUSES:
        st = scan_bus(bus)
        bus_status[bus] = st
        print(f"  bus {bus}: {'RESPONDE' if st == 'ok' else 'COLGADO'}")

    diag = classify_i2c_fault(bus_status, read_dmesg())
    print(f"\nVeredicto: {diag['fault']}  (accion: {diag['action']})")
    print(diag["summary"])
    print("\nPasos recomendados:")
    for i, paso in enumerate(recommended_steps(diag), 1):
        print(f"  {i}. {paso}")
    # 0 si todo OK, 1 si hay falla (util para scripts).
    return 0 if diag["fault"] == "none" else 1


if __name__ == "__main__":
    sys.exit(main())
