"""Logica pura de seguridad de motores: watchdog + tope de duty.

Adaptado del repo del colega Luissalamanca23/Duckietown_Follow_Line_Pi
(robot/motor_bridge.py): un watchdog corta los motores si no llega un comando
`drive()` dentro de WATCHDOG_S (evita que el robot siga si el lazo de control se
cuelga), y un tope de duty para bring-up. Funciones puras (sin hilos ni I2C) para
poder testearlas; el bridge robot-side las invoca.
"""
import math

STOP = (0.0, 0.0)


def watchdog_expired(last_cmd_t, now, watchdog_s):
    """True si paso mas de watchdog_s desde el ultimo comando -> hay que frenar.
    En el umbral exacto aun NO expira (corta solo al PASARSE)."""
    return (now - last_cmd_t) > watchdog_s


def safe_duty(u, max_duty):
    """Acota el duty a [-max_duty, max_duty]; NaN -> 0.0 (fail-safe)."""
    if u is None or math.isnan(u):
        return 0.0
    return max(-max_duty, min(max_duty, u))
