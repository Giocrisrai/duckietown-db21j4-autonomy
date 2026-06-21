"""Tests de odometria / deadreckoning."""
import math
from duckietown_planning.odometry import ticks_to_distance, update_pose


def test_ticks_a_distancia_una_revolucion():
    d = ticks_to_distance(135)                 # 1 revolucion
    assert abs(d - 2 * math.pi * 0.0318) < 1e-9


def test_avanza_recto():
    x, y, th = update_pose(0, 0, 0, 0.1, 0.1)  # ambas ruedas igual
    assert abs(x - 0.1) < 1e-9
    assert abs(y) < 1e-9
    assert abs(th) < 1e-9                       # sin rotacion


def test_gira_en_el_sitio():
    x, y, th = update_pose(0, 0, 0, -0.05, 0.05, baseline=0.1)
    assert th > 0                               # gira a la izquierda
    assert abs(x) < 1e-9 and abs(y) < 1e-9      # sin desplazamiento neto


def test_arco_invariante_a_dt():
    # Robustez a dt variable: integrar el MISMO arco en 1 paso o en N pasos
    # debe dar la misma pose (un hipo del loop en el Jetson no debe sesgar la odometria).
    dl, dr = 0.10, 0.30                          # dtheta_total = 2.0 rad (giro cerrado)
    x1, y1, th1 = update_pose(0, 0, 0, dl, dr)
    x, y, th = 0.0, 0.0, 0.0
    for _ in range(50):
        x, y, th = update_pose(x, y, th, dl / 50, dr / 50)
    assert abs(x1 - x) < 1e-9
    assert abs(y1 - y) < 1e-9
    assert abs(th1 - th) < 1e-9


def test_arco_valor_exacto():
    # Cuarto-de-giro avanzando: forma cerrada del arco de curvatura constante.
    x, y, th = update_pose(0, 0, 0, 0.10, 0.30, baseline=0.10)
    R = 0.20 / 2.0                               # d/dtheta = 0.20 / 2.0 rad
    assert abs(x - R * math.sin(2.0)) < 1e-9
    assert abs(y - R * (1.0 - math.cos(2.0))) < 1e-9
    assert abs(th - 2.0) < 1e-9
