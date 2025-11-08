#!/usr/bin/env python3
"""Test baseline EMS integration with NIL route."""

import numpy as np
from pathlib import Path
from rl_hyb_train import make_env
from rl_hyb_train.policies.baseline_ems import BaselineEMS

def main():
    # Create environment
    env = make_env(Path('conf.yaml'), seed=42)
    
    # Create baseline EMS policy
    ems = BaselineEMS(env.config)
    
    # Test environment with EMS
    obs, info = env.reset()
    print(f"Environment reset successfully")
    print(f"Initial SOC: {obs[1]:.2f}, speed: {obs[0]:.2f} m/s")
    print(f"Target speed: {info.get('target_speed_mps', 0):.2f} m/s")
    
    total_reward = 0.0
    for step in range(100):
        # Get EMS action
        action = ems.compute_action(obs, info)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        if step % 20 == 0:
            print(f"Step {step}: speed={info['speed_mps']:.1f}m/s, "
                  f"SOC={obs[1]:.2f}, "
                  f"reward={reward:.4f}, "
                  f"fc={action[0]:.2f}, "
                  f"batt={action[1]:.2f}")
        
        if terminated or truncated:
            print(f"Episode ended at step {step}")
            break
    
    print(f"Total reward: {total_reward:.2f}")
    env.close()

if __name__ == "__main__":
    main()
