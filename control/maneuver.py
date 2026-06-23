"""Maniobra de interseccion por DEMOSTRACION (replay a distancia por rueda).

Adaptado del repo del colega Luissalamanca23/Duckietown_Follow_Line_Pi
(robot/maneuver.py). Se graba cuanto avanzo CADA rueda al cruzar (ΔsL, ΔsR con
encoders) y al reproducir se conduce en LAZO CERRADO sobre los encoders hasta
alcanzar esas distancias por rueda PRESERVANDO LA RAZON -> reproduce la FORMA del
arco (curvatura constante), no solo el punto final. Robusto a un diametro de rueda
mal calibrado (grabar y reproducir usan el mismo mapa tick->metro; el factor se
cancela).

Complementa nuestro FSM (que hace giros de arco abierto por tiempo) con un giro
cerrado repetible. Funciones puras y testeables (sin encoders ni I2C).
"""

TOL_M = 0.005      # tolerancia de llegada por rueda (m)
KP = 4.0           # ganancia P sobre distancia restante (duty por metro)
MIN_DUTY = 0.10    # piso para vencer el deadband del motor


def reached(target, measured, tol=TOL_M):
    """Llegada DIRECCIONAL: monotona, nunca ordena retroceder (evita oscilar)."""
    return measured >= target - tol if target >= 0 else measured <= target + tol


def replay_step(tL, tR, mL, mR, max_duty, tol=TOL_M, kp=KP):
    """(uL, uR, done): lleva cada rueda a su distancia objetivo preservando la razon.

    La rueda con mas restante va a 'base'; la otra a base*(restante/restante_max),
    asi uL/uR = ΔsL_rest/ΔsR_rest se mantiene -> curvatura constante = arco limpio.
    """
    done_l = reached(tL, mL, tol)
    done_r = reached(tR, mR, tol)
    if done_l and done_r:
        return 0.0, 0.0, True
    e_l = 0.0 if done_l else (tL - mL)
    e_r = 0.0 if done_r else (tR - mR)
    emax = max(abs(e_l), abs(e_r))
    if emax <= 0.0:
        return 0.0, 0.0, True
    base = min(max_duty, max(MIN_DUTY, kp * emax))   # velocidad de la rueda lider
    u_l = 0.0 if done_l else base * (e_l / emax)
    u_r = 0.0 if done_r else base * (e_r / emax)
    return u_l, u_r, False
