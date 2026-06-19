#!/usr/bin/env python3
"""Entrena un agente Deep RL (PPO) de seguir-linea y lo compara con la base PD.

Usa stable-baselines3 sobre LaneFollowEnv (Gymnasium). Entrega analitica:
recompensa media, offset medio y tasa de exito del agente vs el controlador PD.

Uso:  python -m rl.train_ppo --timesteps 30000
"""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--timesteps", type=int, default=30000)
    p.add_argument("--episodes-eval", type=int, default=20)
    p.add_argument("--save", default="runs/rl/ppo_lane")
    args = p.parse_args()

    from stable_baselines3 import PPO
    from lane_env import LaneFollowEnv
    from analytics import evaluate_policy, proportional_policy

    model = PPO("MlpPolicy", LaneFollowEnv(), verbose=0)
    model.learn(total_timesteps=args.timesteps)
    model.save(args.save)

    def rl_policy(obs):
        action, _ = model.predict(obs, deterministic=True)
        return action

    rl = evaluate_policy(LaneFollowEnv(), rl_policy, args.episodes_eval)
    base = evaluate_policy(LaneFollowEnv(), proportional_policy(), args.episodes_eval)
    print("\n==== DEEP RL (PPO) vs BASE PD ====")
    print(f"  PPO: reward={rl['mean_reward']:.2f} offset={rl['mean_abs_offset']:.3f} exito={rl['success_rate']:.2f}")
    print(f"  PD : reward={base['mean_reward']:.2f} offset={base['mean_abs_offset']:.3f} exito={base['success_rate']:.2f}")


if __name__ == "__main__":
    main()
