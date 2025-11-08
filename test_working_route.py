#!/usr/bin/env python3
"""Simple test to show route simulation is working correctly."""

import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

from rl_hyb_train import make_env
from rl_hyb_train.policies.baseline_ems import BaselineEMS

def main():
    print("🚂 Testing Working Route Simulation...")
    
    # Create environment
    env = make_env(Path('conf.yaml'), seed=42)
    
    # Create EMS
    ems = BaselineEMS(env.config)
    
    # Reset and run a working episode
    obs, info = env.reset()
    
    print(f"Route: {len(env.driver.speed_segments)} segments with speed profile")
    print(f"Initial state: speed={info.get('target_speed_mps', 0):.1f}m/s, dwell={info.get('at_stop', False)}")
    
    # Simulate with dynamic movement
    times = []
    speeds = []
    fc_powers = []
    batt_powers = []
    target_speeds = []  # Track what the driver is targeting
    
    # Initial setup
    for step in range(20):
        # Set target speed (simple profile: accelerate to cruise speed)
        if step < 5:
            target_speed = 0.0  # Stay at station
        elif step < 10:
            target_speed = 10.0  # Acceleration phase  
        elif step < 15:
            target_speed = 20.0  # High speed phase
        else:
            target_speed = 22.0  # Cruise phase
        
        target_speeds.append(target_speed)
        
        # Use EMS to compute action
        action = ems.compute_action(obs, info)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        
        times.append(step * env.config.sim.dt_seconds)
        speeds.append(info.get('speed_mps', 0.0))
        fc_powers.append(info.get('p_fc_kw', 0.0))
        batt_powers.append(info.get('p_batt_kw', 0.0))
        
        # Print progress
        print(f"Step {step:2d}: target={target_speed:.1f}m/s, "
              f"actual={info.get('speed_mps', 0):.1f}m/s, "
              f"fc={action[0]:.2f}kW, batt={action[1]:.2f}kW, "
              f"reward={reward:.4f}, dwell={info.get('at_stop', False)}")
    
    # Analysis
    print(f"\n📊 SIMULATION ANALYSIS:")
    avg_actual_speed = np.mean(speeds[20:])  # Exclude stops
    avg_target_speed = np.mean(target_speeds[20:])
    speed_efficiency = avg_actual_speed / avg_target_speed if avg_target_speed > 0 else 0
    print(f"Average target speed: {avg_target_speed:.1f} m/s")
    print(f"Average actual speed: {avg_actual_speed:.1f} m/s") 
    print(f"Speed efficiency: {speed_efficiency:.2f}")
    
    print(f"\n📈 POWER CONSUMPTION:")
    avg_fc = np.mean(fc_powers[20:])
    avg_batt = np.mean([abs(p) for p in batt_powers[20:]])  # Absolute values
    print(f"Average FC power: {avg_fc:.1f} kW")
    print(f"Average battery power: {avg_batt:.1f} kW")
    
    # Simple plot
    import matplotlib.pyplot as plt
    plt.figure(figsize=(10, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(times, target_speeds, 'k--', label='Target Speed', alpha=0.7)
    plt.plot(times, speeds, 'b-', label='Actual Speed', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed Tracking')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.plot(times, fc_powers, 'r-', label='FC Power', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Power (kW)')
    plt.title('Power Generation')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('working_route_simulation_fixed.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"\n✅ Working route simulation complete!")
    print(f"Speed efficiency: {speed_efficiency:.1f} (target vs actual)")
    
    for step in range(300):
        # Use EMS to compute action
        action = ems.compute_action(obs, info)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        
        times.append(step * env.config.sim.dt_seconds)
        speeds.append(info.get('speed_mps', 0.0))
        fc_powers.append(info.get('p_fc_kw', 0.0))
        batt_powers.append(info.get('p_batt_kw', 0.0))
        
        # Print progress
        if step % 50 == 0:
            print(f"Step {step:3d}: speed={info.get('speed_mps', 0):.1f}m/s, "
                  f"fc={action[0]:.2f}kW, batt={action[1]:.2f}kW, "
                  f"reward={reward:.4f}, dwell={info.get('at_stop', False)}")
    
    # Simple analysis
    avg_speed = np.mean(speeds[50:])  # Skip dwell periods
    max_speed = np.max(speeds)
    distance = info.get('distance_km', 0.0)
    
    print(f"\n📊 SIMULATION RESULTS:")
    print(f"Average speed (excluding dwell): {avg_speed:.1f} m/s")
    print(f"Max speed reached: {max_speed:.1f} m/s") 
    print(f"Total distance: {distance:.2f} km")
    print(f"Average FC power: {np.mean(fc_powers[50:]):.1f} kW")
    print(f"Average battery power: {np.mean(np.abs(batt_powers[50:])):.1f} kW")
    
    # Simple plot
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(times, speeds, 'b-', label='Speed', alpha=0.7)
    plt.title('Speed Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.plot(times, fc_powers, 'r-', label='FC Power', alpha=0.7)
    plt.title('FC Power Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (kW)')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 3)
    plt.plot(times, np.abs(batt_powers), 'b-', label='Battery Power', alpha=0.7)
    plt.title('Battery Power Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (kW)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('working_route_simulation.png', dpi=150, bbox_inches='tight')
    
    print(f"\n✅ Working route simulation complete!")
    print(f"Plot saved to 'working_route_simulation.png'")

if __name__ == "__main__":
    main()
