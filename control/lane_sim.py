"""Simulacion cinematica simple de seguir-linea para sintonizar el PID.

Modelo de bicicleta lateral simplificado: estado (d, psi) = desviacion lateral
respecto al centro del carril (m) y angulo de rumbo (rad). El PID actua sobre la
desviacion lateral y comanda la velocidad angular (steering). Permite validar y
optimizar las ganancias sin robot.
"""
import math


def simulate_lane(pid, d0=0.20, psi0=0.0, v=0.3, dt=0.05, steps=240,
                  steer_limit=4.0):
    """Simula el lazo cerrado. Devuelve (d_final, trayectoria_de_d).

    d0: desviacion lateral inicial (m). v: velocidad lineal (m/s).
    El objetivo es llevar d -> 0 (centro del carril).
    """
    pid.reset()
    d, psi = float(d0), float(psi0)
    traj = []
    for _ in range(steps):
        u = pid.step(-d, dt)                       # error = -d (queremos d=0)
        u = max(-steer_limit, min(steer_limit, u))
        psi += u * dt
        d += v * math.sin(psi) * dt
        traj.append(d)
    return d, traj


def settling_metric(traj, tol=0.05):
    """Indice del paso a partir del cual |d| se mantiene bajo `tol` (o None)."""
    n = len(traj)
    for i in range(n):
        if all(abs(x) <= tol for x in traj[i:]):
            return i
    return None
