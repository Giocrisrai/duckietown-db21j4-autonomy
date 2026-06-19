"""Tests de la simulacion de seguir-linea y del tuning del PID."""
from pid import PID
from lane_sim import simulate_lane, settling_metric


def test_buen_tuning_converge_al_centro():
    pid = PID(6.0, 0.0, 4.0, out_limits=(-5.0, 5.0))
    dfin, traj = simulate_lane(pid, d0=0.20)
    assert abs(dfin) < 0.05                     # vuelve al centro del carril
    assert settling_metric(traj, 0.05) is not None


def test_buen_tuning_asienta_mas_rapido_que_sin_damping():
    bueno = PID(6.0, 0.0, 4.0, out_limits=(-5.0, 5.0))
    malo = PID(12.0, 0.0, 0.0, out_limits=(-5.0, 5.0))   # sin derivativo
    _, t_bueno = simulate_lane(bueno, d0=0.20)
    _, t_malo = simulate_lane(malo, d0=0.20)
    assert settling_metric(t_bueno, 0.05) < settling_metric(t_malo, 0.05)
