#!/usr/bin/env python3
"""Run Env0 with the simple P_req driver and the baseline EMS policy.

This script does not change your conf.yaml on disk; it injects a small
manual_p_req_profile into the loaded config in-memory.
"""
from pathlib import Path
import numpy as np

from rl_hyb_train import Config
from rl_hyb_train.env0_env import Env0
from rl_hyb_train.policies.balanced import BalancedEMS


def main():
    conf_path = Path('conf.yaml')
    config = Config.from_yaml(conf_path)

    # Inject a simple segment-based profile for the SimplePReqDriver
    config.driver.manual_p_req_profile = [
        {"duration_s": 120, "mode": "dwell", "label": "Station A"},
        {"duration_s": 180, "mode": "accelerate", "base_kw": 600, "ramp_kw_per_s": 300, "label": "Depart A"},
        {"duration_s": 600, "mode": "cruise", "base_kw": 500, "label": "Cruise"},
        {"duration_s": 300, "mode": "climb", "base_kw": 520, "grade_bias_kw": 120, "label": "Climb"},
        {"duration_s": 180, "mode": "brake", "brake_kw": 400, "label": "Approach/Brake"},
        {"duration_s": 90,  "mode": "dwell", "label": "Station B"},
    ]
    config.driver.manual_loop = True

    # Shorten episode for demo
    config.sim.episode_steps_min = 600
    config.sim.episode_steps_max = 600

    # Create environment directly from Config to use injected profile
    env = Env0(config, seed=42)
    ems = BalancedEMS(env.config)

    obs, info = env.reset()
    ems.reset()
    total_reward = 0.0
    for step in range(300):
        action = ems.act(info=info)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if step % 50 == 0:
            print(f"Step {step:3d} | P_req={info['p_req_kw']:.1f} kW | FC={info['p_fc_kw']:.1f} kW | "
                  f"Batt={info['p_batt_kw']:.1f} kW | Unmet={info['p_unmet_kw']:.1f} kW | "
                  f"SOC={info['soc']:.3f}")
        if terminated or truncated:
            break

    print("\nEpisode summary:")
    print(f"  Steps: {step+1}")
    print(f"  Total reward: {total_reward:.2f}")
    print(f"  Final SOC: {info['soc']:.3f}")
    print(f"  Final tank: {info['tank_level']:.3f}")
    print(f"  Unmet (last): {info['p_unmet_kw']:.1f} kW")


if __name__ == "__main__":
    main()
