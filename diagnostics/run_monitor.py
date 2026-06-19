#!/usr/bin/env python3
"""Monitoreo EN VIVO: agrega salud en un loop y la sirve por HTTP (red).

En el robot, pasa un proveedor de lecturas (desde topicos ROS) y el monitor
expone el estado en http://<robot>:8080/ para conectarte por la red mientras corre.
"""
import threading
import time

from monitor import HealthMonitor
from monitor_server import health_payload, run_monitor_server


def main(read_sensors, port=8080, hz=2.0):
    mon = HealthMonitor()

    def loop():
        period = 1.0 / hz
        while True:
            try:
                mon.update(read_sensors())
            except Exception as e:           # nunca tumbar el monitor por una lectura
                print("monitor: lectura fallo:", e)
            time.sleep(period)

    threading.Thread(target=loop, daemon=True).start()
    print(f"Monitor de salud en http://0.0.0.0:{port}/  (Ctrl+C para salir)")
    run_monitor_server(lambda: health_payload(mon), port=port)


if __name__ == "__main__":
    # DEMO con lecturas dummy. En el robot, reemplazar por lecturas reales de ROS.
    main(lambda: {"battery_percent": 80, "left_delta": 1, "right_delta": 1,
                  "moving": True, "accel_magnitude": 9.8, "tof_range": 0.5})
