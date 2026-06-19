"""Monitoreo en vivo de salud de hardware mientras el robot corre.

Agrega las lecturas de los sensores (via health.summarize), aplica un debounce
(N lecturas malas consecutivas) para no falsear alarmas, detecta NUEVAS fallas y
expone un estado global OK / DEGRADED / FAULT. Pensado para correr en un loop y
publicarse por la red (ver monitor_server).
"""
from health import summarize


class HealthMonitor:
    def __init__(self, fault_threshold=3):
        # lecturas malas consecutivas para declarar FALLA (evita falsos positivos)
        self.fault_threshold = fault_threshold
        self._bad_counts = {}
        self._status = {}
        self.updates = 0

    @staticmethod
    def _is_bad(value):
        if isinstance(value, bool):
            return value is False                  # encoders/imu/tof: False = malo
        return value in ("bajo", "critico")        # bateria

    def update(self, readings):
        """Ingiere un set de lecturas y devuelve el estado actual + nuevas fallas."""
        comp = summarize(readings)
        nuevas_fallas = []
        for k, v in comp.items():
            if self._is_bad(v):
                self._bad_counts[k] = self._bad_counts.get(k, 0) + 1
            else:
                self._bad_counts[k] = 0
            if self._bad_counts[k] >= self.fault_threshold:
                estado = "fault"
            elif self._bad_counts[k] > 0:
                estado = "warn"
            else:
                estado = "ok"
            if estado == "fault" and self._status.get(k) != "fault":
                nuevas_fallas.append(k)
            self._status[k] = estado
        self.updates += 1
        return {"components": dict(self._status),
                "overall": self.overall_status(),
                "new_faults": nuevas_fallas}

    def overall_status(self):
        vals = list(self._status.values())
        if any(v == "fault" for v in vals):
            return "FAULT"
        if any(v == "warn" for v in vals):
            return "DEGRADED"
        return "OK"
