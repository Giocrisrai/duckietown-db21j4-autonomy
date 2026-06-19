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
