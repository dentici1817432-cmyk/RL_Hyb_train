"""Main entry point for RL Hybrid Train environment."""
import numpy as np
from pathlib import Path
from rl_hyb_train import make_env


def main():
    """Run a smoke test of the Env0 environment."""
    print("RL Hybrid Train - Env0 Smoke Test")
    print("=" * 50)
    
    # Load configuration and create environment
    config_path = Path(__file__).parent / "conf.yaml"
    env = make_env(config_path, seed=42)
    print(f"Loaded config from {config_path}")
    print(f"Created environment: {env}")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    print()
    
    # Run a short episode
    print("Running episode...")
    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Initial SOC: {info['soc']:.3f}, Tank: {info['tank_level']:.3f}")
    
    total_reward = 0.0
    step_count = 0
    max_steps = 100  # Short test
    
    for step in range(max_steps):
        # Random action
        action = env.action_space.sample()
        
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        step_count += 1
        
        # Test renderer in human mode
        if step % 5 == 0:  # Render every 5 steps (matching render_every config)
            env.render(mode="human")
        
        if step % 20 == 0:
            print(
                f"Step {step:3d}: "
                f"SOC={info['soc']:.3f}, "
                f"Tank={info['tank_level']:.3f}, "
                f"Speed={info['speed_mps']:.1f} m/s, "
                f"P_req={info['p_req_kw']:.1f} kW, "
                f"Reward={reward:.4f}"
            )
        
        if terminated or truncated:
            print(f"\nEpisode ended: terminated={terminated}, truncated={truncated}")
            break
    
    print(f"\nEpisode summary:")
    print(f"  Steps: {step_count}")
    print(f"  Total reward: {total_reward:.4f}")
    print(f"  Distance: {info['distance_km']:.3f} km")
    print(f"  Final SOC: {info['soc']:.3f}")
    print(f"  Final tank: {info['tank_level']:.3f}")
    print(f"  Constraint violations: {info['constraint_violations']}")
    print("\nSmoke test completed successfully!")


if __name__ == "__main__":
    main()
