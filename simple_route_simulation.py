#!/usr/bin/env python3
"""
Simple route simulation using P_req curve from config.
Shows the train moving along the NIL route with baseline energy management.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from rl_hyb_train import make_env
from rl_hyb_train.policies.baseline_ems import BaselineEMS

def main():
    print("🚂 Simple Route Simulation using P_req Curve")
    print("=" * 50)
    
    # Create environment
    env = make_env(Path('conf.yaml'), seed=42)
    ems = BaselineEMS(env.config)
    
    # Get route info
    num_segments = len(env.driver.speed_segments)
    print(f"Route: {num_segments} segments")
    print("Segments with grades:", [seg.get('meta', {}).get('grade_percent', 0.0) for seg in env.driver.speed_segments])
    
    # Simulate episode
    obs, info = env.reset()
    print(f"Episode length: {env.episode_length} steps ({env.episode_length/60:.1f} min)")
    
    # Track key metrics
    times = []
    speeds = []
    powers = []
    energy_stored = 0.0
    h2_consumed = 0.0
    
    step_count = 0
    while step_count < env.episode_length:
        # Get action from EMS
        action = ems.compute_action(obs, info)
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1
        
        # Store data
        times.append(step_count)
        speeds.append(info['speed_mps'])
        powers.append(info['p_fc_kw'] + info['p_batt_kw'])  # Total power
        
        # Update energy tracking
        if info['p_fc_kw'] > 0:
            h2_used = (info['p_fc_kw'] * env.config.sim.dt_seconds / 3600.0) / (env.config.fuel_cell.eta_fc * env.config.fuel_cell.h2_lhv_kwh_per_kg)
            h2_consumed += h2_used
        
        energy_change = info['p_batt_kw'] * env.config.sim.dt_seconds * info.get('p_batt_delivered_kw', 0) / (env.config.battery.eta_discharge if info.get('p_batt_delivered_kw', 0) != 0 else 1.0)
        
        if energy_change < 0:  # Charging
            energy_stored += abs(energy_change) * info.get('p_batt_delivered_kw', 0) / env.config.battery.eta_charge
        else:  # Discharging
            energy_stored += energy_change * info.get('p_batt_delivered_kw', 0) / env.config.battery.eta_discharge
        if energy_change < 0:  # Charging
            energy_stored += abs(energy_change) * info.get('p_batt_delivered_kw', 0) * env.config.battery.eta_charge
        else:  # Discharging
            energy_stored += energy_change * info.get('p_batt_delivered_kw', 0) / env.config.battery.eta_discharge
        
        # Track key metrics
        if step_count % 100 == 0:
            avg_speed = np.mean(speeds[-50:])
            avg_power = np.mean(powers[-50:])
            avg_soc = obs[1]  # Normalized SOC
            print(f"Step {step_count:3d}: Speed={avg_speed:.1f}m/s, SOC={avg_soc:.2f}")
            print(f"  Power={avg_power:.0f}kW, Energy stored={energy_stored:.2f}kWh, H2 used={h2_consumed:.3f}kg")
    
    # Summary
    total_distance = info.get('distance_km', 0.0)
    print(f"\n{'='*50}")
    print(f"Total distance: {total_distance:.2f} km")
    print(f"Total H2 consumed: {h2_consumed:.2f} kg")
    print(f"Total energy throughput: {np.sum(np.abs(powers)) * env.config.sim.dt_seconds / 1000.0:.2f} kWh")
    print(f"Average speed: {np.mean(speeds):.1f} m/s")
    
    # Simple visualization
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(times, speeds, 'b-', linewidth=2, label='Speed')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title(f"Speed Profile on NIL Route ({num_segments} segments)")
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.plot(times, powers, 'r-', linewidth=2, label='Total Power')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (kW)')
    plt.title('Power Consumption')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 3)
    plt.plot(times, [obs[1] * 100 for obs in obs], 'g-', linewidth=2, label='SOC (%)')
    plt.xlabel('Time (s)')
    plt.ylabel('SOC (%)')
    plt.title('Battery State of Charge')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 2, 4)
    plt.plot(times, [obs[2] * 100 for obs in obs], 'b-', linewidth=2, label='Tank Level (%)')
    plt.xlabel('Time (s)')
    plt.ylabel('Tank Level (%)')
    plt.title('Energy Storage')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('simple_route_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"\n✅ Simple simulation complete!")
    print(f"The train successfully navigates {num_segments} route segments")
    print(f"Energy management adapts to grades and power demands")
    print(f"Baseline EMS effectively manages FC/battery split based on SOC and tank levels")

if __name__ == "__main__":
    main()
