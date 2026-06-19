"""Entorno Gymnasium de seguir-linea para Deep RL (Duckiebot, simulado).

Estado (obs): [offset_normalizado, rumbo] -> (d/d_limit, psi).
Accion: steering continuo en [-1, 1] (se escala a velocidad angular).
Reward: -|offset| + bonus por seguir en carril; penalizacion si se sale.

Es una version ligera y robusta (no depende de gym-duckietown) para entrenar y
analizar agentes RL en la Mac. Comparte la dinamica con control/lane_sim.
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces


class LaneFollowEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, d0_range=0.25, psi0_range=0.3, v=0.3, dt=0.05,
                 max_steps=240, d_limit=0.5, steer_gain=4.0):
        super().__init__()
        self.observation_space = spaces.Box(
            low=np.array([-1.0, -np.pi], dtype=np.float32),
            high=np.array([1.0, np.pi], dtype=np.float32))
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.d0_range = d0_range
        self.psi0_range = psi0_range
        self.v = v
        self.dt = dt
        self.max_steps = max_steps
        self.d_limit = d_limit
        self.steer_gain = steer_gain
        self.d = 0.0
        self.psi = 0.0
        self.t = 0

    def _obs(self):
        return np.array([np.clip(self.d / self.d_limit, -1.0, 1.0), self.psi],
                        dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.d = float(self.np_random.uniform(-self.d0_range, self.d0_range))
        self.psi = float(self.np_random.uniform(-self.psi0_range, self.psi0_range))
        self.t = 0
        return self._obs(), {}

    def step(self, action):
        u = float(np.clip(np.asarray(action).reshape(-1)[0], -1.0, 1.0)) * self.steer_gain
        self.psi += u * self.dt
        self.d += self.v * np.sin(self.psi) * self.dt
        self.t += 1
        reward = -abs(self.d) + 0.05               # penaliza offset, premia seguir vivo
        terminated = abs(self.d) > self.d_limit    # se salio del carril
        truncated = self.t >= self.max_steps
        if terminated:
            reward -= 1.0
        return self._obs(), float(reward), bool(terminated), bool(truncated), {}
