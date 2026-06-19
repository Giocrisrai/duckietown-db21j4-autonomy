"""Tests del entorno Gymnasium de seguir-linea."""
import numpy as np
from lane_env import LaneFollowEnv


def test_reset_obs_en_espacio():
    env = LaneFollowEnv()
    obs, info = env.reset(seed=0)
    assert env.observation_space.contains(obs)
    assert isinstance(info, dict)


def test_step_devuelve_5_tupla():
    env = LaneFollowEnv()
    env.reset(seed=0)
    obs, r, term, trunc, info = env.step(np.array([0.0], dtype=np.float32))
    assert env.observation_space.contains(obs)
    assert isinstance(r, float)
    assert isinstance(term, bool) and isinstance(trunc, bool)


def test_termina_si_se_sale_del_carril():
    env = LaneFollowEnv(d_limit=0.3)
    env.reset(seed=0)
    env.d, env.psi = 0.29, 0.8         # cerca del limite, rumbo hacia afuera
    terminado = False
    for _ in range(20):
        _, _, term, _, _ = env.step(np.array([0.0], dtype=np.float32))
        if term:
            terminado = True
            break
    assert terminado is True


def test_truncate_en_max_steps():
    env = LaneFollowEnv(max_steps=10)
    env.reset(seed=0)
    trunc = False
    for _ in range(10):
        _, _, term, trunc, _ = env.step(np.array([0.0], dtype=np.float32))
    assert trunc is True
