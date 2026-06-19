"""Analitica para evaluar politicas/agentes RL de seguir-linea."""
import numpy as np


def evaluate_policy(env, policy, episodes=20):
    """Evalua una politica `policy(obs)->action` sobre `env`.

    Devuelve metricas: recompensa media, offset absoluto medio (calidad del
    seguimiento) y tasa de exito (episodios que NO se salieron del carril).
    """
    rewards, offsets, exitos = [], [], 0
    for ep in range(episodes):
        obs, _ = env.reset(seed=ep)
        total, eps_off, salio = 0.0, [], False
        while True:
            obs, r, terminated, truncated, _ = env.step(policy(obs))
            total += r
            eps_off.append(abs(float(obs[0])))
            if terminated:
                salio = True
            if terminated or truncated:
                break
        rewards.append(total)
        offsets.append(float(np.mean(eps_off)))
        if not salio:
            exitos += 1
    return {
        "mean_reward": float(np.mean(rewards)),
        "mean_abs_offset": float(np.mean(offsets)),
        "success_rate": exitos / episodes,
    }


def proportional_policy(kp=0.8, kd=0.5):
    """Politica base PD (controlador clasico) sobre offset y rumbo.

    action = -(kp*offset + kd*rumbo). Referencia para comparar contra el agente RL.
    """
    def _pd(obs):
        return np.array([-(kp * float(obs[0]) + kd * float(obs[1]))], dtype=np.float32)
    return _pd


def zero_policy(obs):
    """Politica nula (no actua) — referencia inferior."""
    return np.array([0.0], dtype=np.float32)
