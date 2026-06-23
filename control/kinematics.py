"""Cinematica inversa: comando de coche (v, omega) -> duty de ruedas (u_l, u_r).

Adaptado del repo del colega Luissalamanca23/Duckietown_Follow_Line_Pi
(robot/kinematics.py), que porta la formula del kinematics_node oficial de
Duckietown. Complementa nuestro `duckietown_planning.calibration.apply_trim_gain`
(que trabaja en espacio de velocidad de rueda) con el mapeo directo a DUTY [-1,1]
que esperan los drivers de motor (HATv3 / PCA9685).

    omega_r = (v + 0.5*omega*baseline) / radius     # vel. angular rueda (rad/s)
    omega_l = (v - 0.5*omega*baseline) / radius
    u_r = omega_r * (gain + trim) / k               # duty normalizado
    u_l = omega_l * (gain - trim) / k
    clamp a [-limit, +limit]
"""

# Constantes del kinematics_node oficial (dts). Se calibran por robot.
DEFAULTS = {
    "baseline": 0.1,     # m, separacion entre ruedas
    "radius": 0.0318,    # m, radio de rueda
    "gain": 1.0,         # escala global de velocidad
    "trim": 0.0,         # corrige asimetria izquierda/derecha del HAT
    "k": 27.0,           # escala motor->duty
    "limit": 1.0,        # tope de duty
}


def vw_to_duty(v, omega, params=None):
    """(v [m/s], omega [rad/s]) -> (u_l, u_r) en [-limit, +limit]."""
    p = params or DEFAULTS
    baseline, radius = p["baseline"], p["radius"]
    gain, trim, k, limit = p["gain"], p["trim"], p["k"], p["limit"]

    omega_r = (v + 0.5 * omega * baseline) / radius
    omega_l = (v - 0.5 * omega * baseline) / radius
    u_r = omega_r * (gain + trim) / k
    u_l = omega_l * (gain - trim) / k

    clamp = lambda u: max(-limit, min(limit, u))
    return clamp(u_l), clamp(u_r)
