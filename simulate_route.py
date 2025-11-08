#!/usr/bin/env python3
"""
Simulate the train running on the NIL route with baseline EMS policy.
Shows how the train responds to the route and manages energy.
"""

import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

from rl_hyb_train import make_env
from rl_hyb_train.policies.baseline_ems import BaselineEMS

def plot_results(results):
    """Plot simulation results."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 10))
    
    # Plot 1: Speed profile
    axes[0, 0].plot(results['times'], results['speeds'], 'b-', linewidth=2)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Speed (m/s)')
    axes[0, 0].set_title('Speed Profile')
    axes[0, 0].grid(True)
    
    # Plot 2: Power consumption
    axes[1, 0].plot(results['times'], results['p_fc'], 'r-', label='FC Power', linewidth=2)
    axes[1, 0].plot(results['times'], results['p_batt'], 'b-', label='Battery Power', linewidth=2)
    axes[1, 0].plot(results['times'], results['p_req'], 'k--', label='P_req', linewidth=1, alpha=0.7)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Power (kW)')
    axes[1, 0].set_title('Power Consumption')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Plot 3: Energy levels
    axes[2, 0].plot(results['times'], results['soc'], 'g-', linewidth=2)
    axes[2, 0].plot(results['times'], results['tank'], 'b-', linewidth=2)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Level / Normalized')
    axes[2, 0].set_title('Energy Levels (SOC & Tank)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    plt.tight_layout()
    plt.savefig('route_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()

def simulate_route():
    """Simulate train on NIL route with baseline EMS."""
    print("🚂 Starting NIL route simulation with Baseline EMS...")
    
    # Create environment and EMS
    env = make_env(Path('conf.yaml'), seed=42)
    ems = BaselineEMS(env.config)
    
    # Reset environment
    obs, info = env.reset()
    print(f"Route: {len(env.driver.speed_segments)} segments")
    print(f"Stops: {len(env.driver.speed_segments)} stops")
    print(f"Episode length: {env.episode_length} steps ({env.episode_length/60:.1f} min)")
    
    # Simulation data collection
    results = {
        'times': [],
        'speeds': [],
        'p_req': [],
        'p_fc': [],
        'p_batt': [],
        'soc': [],
        'tank': [],
        'distances': []
    }
    
    # Run simulation
    step_count = 0
    while True:
        # Get EMS action
        action = ems.compute_action(obs, info)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1
        
        # Store results
        results['times'].append(step_count * env.config.sim.dt_seconds)
        results['speeds'].append(info['speed_mps'])
        results['p_req'].append(info['p_req_kw'])
        results['p_fc'].append(info['p_fc_kw'])
        results['p_batt'].append(info['p_batt_kw'])
        results['soc'].append(obs[1])  # SOC normalized
        results['tank'].append(obs[2])  # Tank normalized
        results['distances'].append(info['distance_km'])
        
        # Print progress
        if step_count % 100 == 0:
            avg_speed = np.mean(results['speeds'][-100:])
            avg_soc = np.mean(results['soc'][-100:])
            avg_p_fc = np.mean(results['p_fc'][-100:])
            avg_p_batt = np.mean(np.abs(results['p_batt'][-100:]))
            
            print(f"Step {step_count:4d}: speed={avg_speed:.1f}m/s, "
                  f"SOC={avg_soc:.2f}, FC={avg_p_fc:.0f}kW, "
                  f"Batt={avg_p_batt:.0f}kW, dist={results['distances'][-1]:.2f}km")
        
        # Check termination
        if terminated or truncated:
            print(f"Episode ended at step {step_count}")
            break
        
        # Stop after full episode
        if step_count >= env.episode_length:
            print("Episode complete!")
            break
    
    # Summary statistics
    total_distance = results['distances'][-1] if results['distances'] else 0
    total_h2_used = env.episode_info.delta_m_h2_kg
    total_energy_stored = env.config.battery.e_batt_kwh * np.mean(results['soc'][:100])  # Approximate
    
    print(f"\n=== ROUTE SIMULATION SUMMARY ===")
    print(f"Total distance: {total_distance:.2f} km")
    print(f"Total H2 used: {total_h2_used:.2f} kg")
    print(f"Average energy stored: {total_energy_stored:.2f} kWh")
    print(f"Number of steps: {step_count}")
    
    # Plot results
    plot_results(results)

if __name__ == "__main__":
    simulate_route()
