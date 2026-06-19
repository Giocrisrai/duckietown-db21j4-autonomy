# Deep RL — entorno propio de seguir-linea (validado)

Entorno Gymnasium `rl/lane_env.py` (LaneFollowEnv) + analitica `rl/analytics.py`
+ entrenador PPO `rl/train_ppo.py` (stable-baselines3). No depende de
gym-duckietown (fragil); corre en M1.

## Resultados (LaneFollowEnv, 30 episodios de evaluacion)

| Politica | Reward medio | Offset abs. medio | Tasa de exito |
|---|---|---|---|
| **PPO (200k pasos)** | -7.12 | 0.159 | **1.00** |
| Base PD (clasico) | 9.28 | 0.023 | 1.00 |

- El agente PPO **aprendio a seguir el carril**: tasa de exito 0% (30k pasos) -> **100%** (200k).
- El **control PD clasico es mas preciso** (centra mejor). En tareas simples bien
  modeladas el control clasico suele superar al RL; el robot usa el PD.
- Lo validado: el **pipeline Deep RL funciona end-to-end** (entorno + entrenamiento + analitica).

## Reproducir
```bash
pip install -r requirements-rl.txt
python -m rl.train_ppo --timesteps 200000 --episodes-eval 30
```

## Siguiente (opcional)
- Tunear hiperparametros de PPO / mas pasos para acercarse al PD.
- Para el simulador 3D oficial: gym-duckietown (instalacion fragil en M1/Py3.13).
