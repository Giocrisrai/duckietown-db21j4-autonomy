"""Tests de la analitica RL: la politica PD debe superar a la nula."""
from lane_env import LaneFollowEnv
from analytics import evaluate_policy, proportional_policy, zero_policy


def test_pd_supera_a_politica_nula():
    env = LaneFollowEnv()
    pd = evaluate_policy(env, proportional_policy(), episodes=15)
    zero = evaluate_policy(env, zero_policy, episodes=15)
    assert pd["mean_reward"] > zero["mean_reward"]
    assert pd["mean_abs_offset"] < zero["mean_abs_offset"]
    assert pd["success_rate"] >= zero["success_rate"]


def test_pd_alcanza_buen_seguimiento():
    env = LaneFollowEnv()
    pd = evaluate_policy(env, proportional_policy(), episodes=15)
    assert pd["success_rate"] >= 0.9          # casi siempre se mantiene en carril
    assert pd["mean_abs_offset"] < 0.1
