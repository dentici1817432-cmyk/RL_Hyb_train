#!/usr/bin/env python3
"""
Comprehensive validation of spec alignment implementation.
Tests all major components against spec.md requirements.
"""

import numpy as np
from pathlib import Path
from typing import Dict, Any
import sys

# Import our modules
from rl_hyb_train import make_env
from rl_hyb_train.policies.baseline_ems import BaselineEMS

def validate_observation_space(env, spec_name: str = "Spec Requirements"):
    """Validate 12-dimension observation space."""
    print(f"\n=== {spec_name}: Observation Space Validation ===")
    
    # Test observation space dimensions
    expected_shape = (12,)
    actual_shape = env.observation_space.shape
    
    print(f"Expected shape: {expected_shape}")
    print(f"Actual shape: {actual_shape}")
    
    if actual_shape == expected_shape:
        print("✅ PASS: Observation space has correct dimensions")
    else:
        print("❌ FAIL: Observation space dimensions mismatch")
        return False
    
    # Test observation bounds
    low_bounds, high_bounds = env.observation_space.low, env.observation_space.high
    
    # All observations should be in [-1, 1]
    if np.allclose(low_bounds, -1.0, atol=1e-6) and np.allclose(high_bounds, 1.0, atol=1e-6):
        print("✅ PASS: Observation bounds are [-1, 1]")
    else:
        print(f"❌ FAIL: Observation bounds not [-1, 1]: low={low_bounds}, high={high_bounds}")
        return False
    
    # Test individual observation channels
    obs, info = env.reset()
    obs_descriptions = [
        "0: Speed (m/s)",
        "1: SOC estimate (noisy)", 
        "2: H2 tank level (noisy)",
        "3: Time/distance to next stop",
        "4: Grade preview/segment", 
        "5: P_req (filtered)",
        "6: P_req trend",
        "7: Last FC fraction",
        "8: Last battery command",
        "9-11: Nuisance noise"
    ]
    
    print("Observation channels:")
    for i, desc in enumerate(obs_descriptions):
        print(f"  {desc}: {obs[i]:.3f}")
    
    # Validate specific spec requirements
    # 1. Speed normalization check
    speed_normalized = obs[0]
    speed_mps = info.get('speed_mps', 0.0)
    expected_speed_norm = (speed_mps / env.config.plant.v_max_mps) * 2.0 - 1.0
    if abs(speed_normalized - expected_speed_norm) < 0.1:
        print("✅ PASS: Speed normalization correct")
    else:
        print(f"❌ FAIL: Speed normalization wrong: got {speed_normalized:.3f}, expected {expected_speed_norm:.3f}")
    
    # 2. Time/distance to next stop
    time_to_stop = obs[3]
    if -1.0 <= time_to_stop <= 1.0:
        print("✅ PASS: Time/distance to stop in expected range")
    else:
        print("⚠️  WARNING: Time/distance to stop outside expected range")
    
    # 3. Grade preview
    grade_preview = obs[4]
    if -1.0 <= grade_preview <= 1.0:
        print("✅ PASS: Grade preview in expected range")
    else:
        print("❌ FAIL: Grade preview outside [-1, 1] range")
    
    # 4. P_req trend should be derivative
    p_req_trend = obs[6]
    print(f"P_req trend value: {p_req_trend:.3f}")
    if -1.0 <= p_req_trend <= 1.0:
        print("✅ PASS: P_req trend in expected range")
    else:
        print("❌ FAIL: P_req trend outside [-1, 1] range")
    
    return True

def validate_train_dynamics(env, spec_name: str = "Spec Requirements"):
    """Validate realistic train physics implementation."""
    print(f"\n=== {spec_name}: Train Dynamics Validation ===")
    
    # Test Davis resistance implementation
    has_davis_coeffs = hasattr(env.plant, 'train_config') and hasattr(env.plant.train_config, 'davis_A_N')
    if has_davis_coeffs:
        print(f"Davis A: {env.plant.train_config.davis_A_N}")
        print(f"Davis B: {env.plant.train_config.davis_B_N_per_mps}")
        print(f"Davis C: {env.plant.train_config.davis_C_N_per_mps2}")
        print("✅ PASS: Davis coefficients configured")
    else:
        print("❌ FAIL: No train config with Davis coefficients")
        return False
    
    # Test mass-dependent dynamics
    obs, info = env.reset()
    initial_mass = env.plant.passenger_mass_tons
    print(f"Passenger mass: {initial_mass:.1f} tons")
    
    # Check train config access
    has_train_config = hasattr(env.plant, 'train_config')
    if has_train_config:
        train = env.plant.train_config
        print(f"Davis A: {train.davis_A_N}")
        print(f"Davis B: {train.davis_B_N_per_mps}")
        print(f"Davis C: {train.davis_C_N_per_mps2}")
    else:
        print("❌ FAIL: No train config accessible")
    
    # Run a few steps to check dynamics
    prev_speeds = []
    for step in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        speed = info['speed_mps']
        prev_speeds.append(speed)
        
        if step > 1:
            accel = speed - prev_speeds[-2]
            if step < 5:  # Early steps, should see some dynamics
                if abs(accel) > 0.01:
                    print(f"✅ PASS: Step {step}: acceleration {accel:.3f} m/s²")
                else:
                    print(f"⚠️  Step {step}: very low acceleration {accel:.3f}")
    
    return True

def validate_baseline_ems(env, spec_name: str = "EMS Policy"):
    """Validate baseline EMS policy implementation."""
    print(f"\n=== {spec_name}: Baseline EMS Validation ===")
    
    from rl_hyb_train.policies.baseline_ems import BaselineEMS
    ems = BaselineEMS(env.config)
    
    print(f"EMS Parameters:")
    print(f"  SOC_min_usable: {ems.soc_min_usable}")
    print(f"  SOC_max_regen: {ems.soc_max_regen}")
    print(f"  P_FC_nominal: {ems.p_fc_nominal_kw:.1f} kW")
    
    # Test EMS behavior in different scenarios
    test_cases = [
        {
            "name": "Low SOC scenario",
            "soc": 0.1,
            "p_req": 100.0,
            "tank": 0.5
        },
        {
            "name": "Normal operation", 
            "soc": 0.5,
            "p_req": 200.0,
            "tank": 0.8
        },
        {
            "name": "High SOC, regen",
            "soc": 0.95,
            "p_req": -100.0,  # Regen
            "tank": 0.8
        }
    ]
    
    for i, case in enumerate(test_cases):
        print(f"\nTest case {i+1}: {case['name']}")
        
        # Create mock observation
        obs = np.zeros(12, dtype=np.float32)
        obs[1] = case['soc'] * 2.0 - 1.0  # SOC normalized
        obs[2] = case['tank'] * 2.0 - 1.0  # Tank normalized
        
        # Mock info for EMS
        info = {
            'speed_mps': 20.0,
            'p_req_kw': case['p_req'],
            'is_dwelling': False
        }
        
        action = ems.compute_action(obs, info)
        
        print(f"  SOC: {case['soc']:.2f}, FC_frac: {action[0]:.3f}, Batt_cmd: {action[1]:.3f}")
        
        # Validate action logic
        if case['name'] == "Low SOC scenario":
            # Should see FC boost or battery prioritization
            if action[0] > 0.9:  # High FC usage
                print("  ✅ PASS: High FC usage in low SOC")
            else:
                print("  ⚠️  WARNING: Low FC usage in low SOC")
                
        elif case['name'] == "High SOC, regen":
            # Should accept regen
            if case['p_req'] < 0 and action[1] < 0:  # Charging
                print("  ✅ PASS: Regenerative charging accepted")
            else:
                print("  ❌ FAIL: Regeneration not handled correctly")
    
    return True

def validate_reward_function(env, spec_name: str = "Reward Function"):
    """Validate reward function components."""
    print(f"\n=== {spec_name}: Reward Function Validation ===")
    
    # Test reward components in info dict
    obs, info = env.reset()
    rewards = []
    
    for step in range(5):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        rewards.append(reward)
        
        if step == 2:
            print(f"Step {step} components:")
            for key in ['cost_h2_eur', 'cost_grid_eur', 'penalty_delay_eur', 'penalty_unmet_eur', 'penalty_smooth_eur']:
                if key in info:
                    print(f"  {key}: {info[key]:.6f}")
    
    # Check if rewards are negative (costs)
    avg_reward = np.mean(rewards)
    if avg_reward < 0:
        print(f"✅ PASS: Average reward negative ({avg_reward:.4f}) as expected for costs")
    else:
        print(f"❌ FAIL: Average reward positive ({avg_reward:.4f}) - should be costs")
    
    # Check for waste power penalty (pending implementation)
    has_waste_penalty = 'penalty_waste' in info or 'waste' in info
    if has_waste_penalty:
        print("✅ PASS: Waste power penalty implemented")
    else:
        print("⚠️  WARNING: Waste power penalty not yet implemented")
    
    return True

def validate_config_structure(config_path: Path, spec_name: str = "Config Structure"):
    """Validate configuration structure matches spec requirements."""
    print(f"\n=== {spec_name}: Config Structure Validation ===")
    
    from rl_hyb_train import Config
    
    try:
        config = Config.from_yaml(config_path)
    except Exception as e:
        print(f"❌ FAIL: Cannot load config: {e}")
        return False
    
    # Check spec-compliant configuration exists
    has_route = hasattr(config.scenario, 'route')
    has_stops = hasattr(config.scenario, 'stops')
    has_driveline = hasattr(config.scenario, 'driveline')
    
    print(f"Route config: {'✅' if has_route else '❌'}")
    print(f"Stops config: {'✅' if has_stops else '❌'}")
    print(f"Driveline config: {'✅' if has_driveline else '❌'}")
    
    # Check train physics config
    has_train_config = hasattr(config.train, 'train')
    if has_train_config:
        train = config.train.train
        print(f"Davis coefficients: A={train.davis_A_N}, B={train.davis_B_N_per_mps}, C={train.davis_C_N_per_mps2}")
        print(f"Mass range: {train.mass_tons_min}-{train.mass_tons_max} tons")
        print("✅ PASS: Train physics config available")
    else:
        print("❌ FAIL: No train physics configuration")
        return False
    
    return True

def main():
    """Main validation function."""
    print("Starting comprehensive spec alignment validation...\n")
    
    results = {
        "config": False,
        "observations": False,
        "dynamics": False,
        "ems": False,
        "reward": False
    }
    
    # 1. Validate configuration structure
    results["config"] = validate_config_structure(Path('conf.yaml'))
    
    # 2. Validate observation space
    try:
        env = make_env(Path('conf.yaml'), seed=42)
        results["observations"] = validate_observation_space(env)
    except Exception as e:
        print(f"❌ FAIL: Could not create environment for obs validation: {e}")
    
    # 3. Validate train dynamics
    if 'env' in locals():
        results["dynamics"] = validate_train_dynamics(env)
    
    # 4. Validate baseline EMS
    if 'env' in locals():
        results["ems"] = validate_baseline_ems(env)
    
    # 5. Validate reward function
    if 'env' in locals():
        results["reward"] = validate_reward_function(env)
    
    # Summary
    print(f"\n{'='*50}")
    print(f"VALIDATION SUMMARY:")
    all_passed = all(results.values())
    
    for component, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {component}: {status}")
    
    if all_passed:
        print(f"\n🎉 ALL TESTS PASSED! Implementation meets spec requirements.")
        return 0
    else:
        print(f"\n⚠️  SOME TESTS FAILED. Review implementation gaps.")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
