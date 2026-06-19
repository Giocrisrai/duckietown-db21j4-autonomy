"""Chequeos puros de salud de hardware del Duckiebot DB21-J4.

Detectan fallas comunes a partir de lecturas de sensores: bateria baja,
encoder/motor muerto, IMU fuera de rango, y ToF invalido. Son funciones puras
(testeables) que un nodo de diagnostico en el robot puede invocar.
"""


def battery_status(percent, low=20.0, critical=10.0):
    """Estado de la bateria por porcentaje de carga."""
    if percent is None:
        return "desconocido"
    if percent <= critical:
        return "critico"
    if percent <= low:
        return "bajo"
    return "ok"


def encoders_ok(left_delta, right_delta, comandado_moviendo):
    """Falla (False) si se comanda movimiento pero ningun encoder cuenta.

    Indica motor o encoder muerto / cable suelto. Si no se comanda movimiento,
    siempre se considera OK.
    """
    if not comandado_moviendo:
        return True
    return not (left_delta == 0 and right_delta == 0)


def imu_sane(accel_magnitude, g=9.81, tol=3.0):
    """En reposo, la magnitud de la aceleracion del IMU debe aproximarse a g."""
    if accel_magnitude is None:
        return False
    return abs(accel_magnitude - g) <= tol


def tof_ok(range_m, min_r=0.02, max_r=1.2):
    """Lectura del ToF VL53L0X dentro de su rango fisico valido."""
    if range_m is None:
        return False
    return min_r <= range_m <= max_r


def summarize(readings):
    """Resumen de salud a partir de un dict de lecturas.

    Claves opcionales: battery_percent, left_delta, right_delta, moving,
    accel_magnitude, tof_range. Devuelve dict {componente: estado/ok}.
    """
    out = {}
    if "battery_percent" in readings:
        out["battery"] = battery_status(readings["battery_percent"])
    if {"left_delta", "right_delta", "moving"} <= readings.keys():
        out["encoders"] = encoders_ok(
            readings["left_delta"], readings["right_delta"], readings["moving"])
    if "accel_magnitude" in readings:
        out["imu"] = imu_sane(readings["accel_magnitude"])
    if "tof_range" in readings:
        out["tof"] = tof_ok(readings["tof_range"])
    return out
