import os
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from rl_hyb_train import Config

def generate_mermaid(config: Config) -> str:
    """Generate detailed Mermaid flowchart with formulas and constants."""
    # Extract constants
    dt_s = config.sim.dt_seconds
    dt_h = dt_s / 3600.0
    eta_fc = config.fuel_cell.eta_fc
    lh_v = config.fuel_cell.h2_lhv_kwh_per_kg
    tank_cap = config.fuel_cell.tank_capacity_kg
    p_fc_max = config.fuel_cell.p_fc_max_kw
    ramp_rate = config.fuel_cell.ramp_kw_per_s
    eta_chg = config.battery.eta_charge
    eta_dis = config.battery.eta_discharge
    e_batt = config.battery.e_batt_kwh
    p_batt_max_chg = config.battery.p_batt_max_charge_kw
    p_batt_max_dis = config.battery.p_batt_max_discharge_kw
    soc_soft_min = config.shield.soc_soft_min
    soc_soft_max = config.shield.soc_soft_max
    soc_hard_min = config.battery.soc_hard_min
    tank_hard_min = config.fuel_cell.tank_hard_min
    c_h2 = config.costs.c_h2_eur_per_kg
    c_grid = config.costs.c_grid_eur_per_kwh
    lambda_smooth = config.reward_weights.lambda_smooth
    lambda_delay = config.reward_weights.lambda_delay
    lambda_unmet = config.reward_weights.lambda_unmet
    tau_preq = config.driver.p_req_smoothing_tau_s
    p_aux_base = config.plant.p_aux_base_watts / 1000.0
    aux_sigma = config.randomization.aux_bias_rw_sigma_kw
    v_max = config.plant.v_max_mps
    p_req_max = config.observations.p_req_max_kw
    soc_noise = config.observations.soc_noise_std
    tank_noise = config.observations.tank_noise_std
    noise_std = config.observations.nuisance_noise_std
    alpha_preq = dt_s / (tau_preq + dt_s)
    p_loss = config.driver.p_loss_watts / 1000.0
    kinematic_gain = config.plant.kinematic_gain_mps_per_watt
    davis_a = 5000.0  # Fallback, assume train_config if available
    davis_b = 100.0
    davis_c = 5.0
    mass_min = config.randomization.passenger_mass_tons_min
    mass_max = config.randomization.passenger_mass_tons_max

    # Build Mermaid with quoted edges
    mermaid = f"""flowchart TD
    subgraph Agent ["Agent (External RL Policy)"]
        A["(obs<br/>12D normalized<br/>vector)"]
        E["Agent<br/>Input: obs<br/>Output: action<br/><br/>action = (fc_frac: 0-1,<br/>batt_cmd: -1 to 1)"]
    end

    subgraph Shield ["Shield (Safety Constraints)"]
        F["Shield.apply<br/><br/>p_fc_cmd = fc_frac * P_fc_max ({p_fc_max} kW)<br/>p_batt_cmd = batt_cmd * (P_chg_max if <0 else P_dis_max)<br/>(P_chg_max={p_batt_max_chg} kW, P_dis_max={p_batt_max_dis} kW)<br/><br/>FC Ramp: |Δp_fc| ≤ ramp_rate * dt ({ramp_rate} kW/s)<br/>p_fc = clip(p_fc_cmd, 0, P_fc_max)<br/>If tank ≤ tank_hard_min ({tank_hard_min}): p_fc = 0<br/><br/>SOC Corridor (enforce):<br/>If SOC < soc_soft_min ({soc_soft_min}):<br/>  - Block discharge (p_batt ≥0 → 0)<br/>  - If very low: force charge (-0.3*P_chg_max)<br/>If SOC > soc_soft_max ({soc_soft_max}):<br/>  - Block charge (p_batt <0 → 0)<br/>  - Limit discharge to 0.5*P_dis_max<br/>Hard: If SOC ≤ soc_hard_min ({soc_hard_min}): only charge/zero<br/>If SOC ≥ soc_hard_max (1.00): only discharge/zero<br/><br/>C-rate Caps (enforce):<br/>clip(p_batt, -P_chg_max, P_dis_max)<br/><br/>Output: ShieldedAction<br/>(p_fc_kw, p_batt_kw, violations,<br/>soc_low/high, fc_ramp_limited,<br/>crate_capped, regen_clipped)"]
    end

    subgraph Plant ["Plant Dynamics"]
        G["Plant.step<br/><br/>aux_bias += N(0, aux_sigma) ({aux_sigma} kW)<br/>p_aux = p_aux_base ({p_aux_base} kW) + aux_bias<br/>p_dem = p_req + p_aux<br/>p_loss = {p_loss} kW (fixed)<br/><br/>%% Battery Flow<br/>dis_kw = max(p_batt, 0)<br/>chg_kw = max(-p_batt, 0)<br/>delivered = dis_kw * η_dis ({eta_dis})<br/><br/>%% Regen Flow (if p_req <0 & speed >0.01 m/s)<br/>regen_total = |p_req|<br/>regen_aux = min(regen_total, p_aux)<br/>post_aux = max(0, regen_total - regen_aux)<br/>capture = min(chg_cmd, post_aux)<br/>friction = max(0, regen_total - regen_aux - capture)<br/>p_brake_total = regen_total<br/>p_brake_aux = regen_aux<br/>p_brake_regen = regen_aux + capture<br/>p_brake_friction = friction<br/>p_regen_post_aux = post_aux<br/><br/>%% FC Excess for charging<br/>fc_excess = max(0, p_fc - p_dem)<br/>chg_from_regen = min(chg_cmd, post_aux)<br/>chg_from_fc = min(max(0, fc_excess), chg_cmd - chg_from_regen)<br/>actual_chg = chg_from_regen + chg_from_fc<br/>p_batt_actual = -actual_chg if chg else p_batt<br/>p_batt_chg_regen = chg_from_regen<br/>p_batt_chg_fc = chg_from_fc<br/>p_batt_dis = max(p_batt_actual, 0)<br/>p_batt_chg = max(-p_batt_actual, 0)<br/>p_batt_delivered = p_batt_dis * η_dis<br/><br/>%% Power Balance<br/>p_supply = p_fc + p_batt_delivered<br/>residual = p_dem - p_supply<br/>p_unmet = max(residual, 0)<br/>p_delivered = p_supply - p_aux - p_unmet<br/><br/>%% Energy Updates<br/>ΔSOC = -p_batt_dis / (E_batt * η_dis) * dt_h<br/>     + p_batt_chg * η_chg / E_batt * dt_h<br/>     (E_batt={e_batt} kWh, η_chg={eta_chg}, dt_h={dt_h})<br/>SOC = clip(SOC + ΔSOC, 0, 1)<br/><br/>Δm_H2 = (p_fc * dt_h) / (η_fc * LHV)<br/>       (η_fc={eta_fc}, LHV={lh_v} kWh/kg)<br/>If Δm_H2 >0: tank = max(0, tank - Δm_H2 / tank_cap ({tank_cap} kg))<br/><br/>%% Kinematics (if speed ≥0.01 m/s & p_req >0)<br/>If stopped (speed<0.01 & p_req≤0): speed=0, no distance update<br/>Else:<br/>  If speed>0.1: F_traction = p_delivered *1000 / speed<br/>  Else if p_delivered>0: F_traction = min(p_delivered*1000/0.1, 50000 N)<br/>  Else: F_traction=0<br/><br/>  %% Davis Resistance<br/>  F_resist = A + B*speed + C*speed²<br/>             (A={davis_a} N, B={davis_b} N/(m/s), C={davis_c} N/(m/s)²)<br/><br/>  %% Grade Force (from driver)<br/>  F_grade = mass * g * grade/100<br/>            (mass ~U[{mass_min},{mass_max}] tons, g=9.81 m/s²)<br/><br/>  net_F = F_traction - F_resist - F_grade<br/>  a = net_F / (mass *1000)<br/>  speed_new = clip(speed + a*dt, 0, v_max ({v_max} m/s))<br/>  distance += speed * dt /1000 (km)<br/>  If dwelling: speed=0<br/><br/>Output: PlantState (SOC, tank, speed, p_fc/batt/unmet/dem/supply/delivered/loss,<br/>p_batt_chg/disch/reg/del, p_regen_post_aux, p_brake_*)"]
    end

    subgraph Driver ["Driver (Traction Demand)"]
        B["Driver.step(dt={dt_s}s, current_speed)<br/><br/>%% Schedule (manual_speed_profile from conf)<br/>%% Segments: dwell/accel/cruise/climb/descend/stop<br/>%% e.g., Station A (0 m/s, 200s), Depart (20 m/s, 300s, grade=0.9%), etc.<br/>%% Interpolation: linear or hold<br/>%% Loop: false (one episode)<br/><br/>%% Target Speed from Profile<br/>v_target = interpolate(segment, time)<br/>v_smoothed = lowpass(v_target, tau={config.driver.speed_target_smoothing_tau_s}s)<br/>is_dwell = v_target ≤{config.driver.dwell_speed_threshold_mps} m/s or segment.dwell<br/>next_stop_time = dwell_end or phase-based fallback<br/>grade = segment.grade_percent (e.g., 0.009 to 0.0238 %)<br/><br/>%% Speed Tracking (PID enabled)<br/>error = v_smoothed - speed_filtered (tau_meas={config.driver.speed_measurement_filter_tau_s}s)<br/>If |error| ≤ deadband ({config.driver.speed_error_deadband_mps} m/s): error=0<br/>If |error| < sep ({config.driver.speed_integral_separation_mps} m/s): no integral<br/>If dwelling/saturated: no integral (anti-windup)<br/>integral = clip(integral + error*dt, ±{config.driver.speed_pid_integral_limit} m/s)<br/>deriv = filter((error - prev_error)/dt, tau_d={config.driver.speed_pid_derivative_filter_tau_s}s)<br/>a_pid = Kp*error + Ki*integral + Kd*deriv<br/>       (Kp={config.driver.speed_pid_kp}, Ki={config.driver.speed_pid_ki}, Kd={config.driver.speed_pid_kd})<br/><br/>%% Constraints<br/>a_cmd = clip(a_pid + a_profile, -brake_lim (-{config.driver.speed_tracking_brake_limit_mps2} m/s²), accel_lim ({config.driver.speed_tracking_accel_limit_mps2} m/s²))<br/>If jerk_lim ({config.driver.speed_tracking_jerk_limit_mps3} m/s³): a_cmd = clip(a_cmd, prev_a ± jerk*dt)<br/>If planner_enable: preview over horizon ({config.driver.speed_planner_horizon_s}s) with penalties<br/>  (penalty_accel={config.driver.speed_planner_penalty_accel}, penalty_jerk={config.driver.speed_planner_penalty_jerk}; min_h={config.driver.speed_planner_min_horizon_s}s)<br/><br/>%% Power from Accel<br/>p_net = a_cmd / (kinematic_gain *1000) (gain={kinematic_gain} m/s/W)<br/>p_req_raw = p_loss ({p_loss} kW) + p_net + segment.p_bias (e.g., 50-100 kW)<br/>clip(p_req_raw, -clip_kw (-{config.driver.speed_control_power_clip_kw} kW), clip_kw ({config.driver.speed_control_power_clip_kw} kW))<br/><br/>%% Rate Limit & Filter<br/>dp_max = rate_lim * dt ({config.driver.p_req_rate_limit_kw_per_s} kW/s)<br/>p_req = clip(p_req_raw, prev ± dp_max)<br/>alpha = dt/(tau_smoothing + dt) (tau={tau_preq}s → α={alpha_preq:.2f})<br/>p_req_filtered = α*p_req + (1-α)*prev_filtered<br/><br/>Output: p_req_filtered (to plant & obs),<br/>target_speed, is_dwelling, grade, next_stop_time"]
    end

    subgraph Observation ["Observation Builder"]
        D["_get_observation<br/><br/>obs = zeros(12, float32); clip([-1,1])<br/><br/>obs[0] = clip((speed / v_max)*2 -1, [-1,1]) (v_max={v_max} m/s)<br/>obs[1] = (clip(SOC + N(0,σ_soc), [0,1])*2 -1) (σ_soc={soc_noise})<br/>obs[2] = (clip(tank + N(0,σ_tank), [0,1])*2 -1) (σ_tank={tank_noise})<br/>obs[3] = if next_stop_time finite:<br/>           clip((next_stop_time / 600s)*2 -1, [-1,1])  // ~10 min max<br/>         else: (step / episode_len)*2 -1  // phase [0,1]→[-1,1]<br/>obs[4] = clip((grade / 5%)*2 -1, [-1,1])  // grades ±5%<br/>obs[5] = clip((p_req_filtered / p_req_max)*2 -1, [-1,1]) (max={p_req_max} kW)<br/>obs[6] = clip( (p_req_filt - prev_filt)/dt / 1000kW/s *2 -1, [-1,1] )  // trend, max 1000 kW/s<br/>obs[7] = last_fc_frac *2 -1  // [0,1]→[-1,1]<br/>obs[8] = last_batt_cmd  // already [-1,1]<br/>obs[9:12] = N(0, σ_nuisance={noise_std})  // 3 noise channels<br/><br/>Return: obs.astype(float32)"]
    end

    subgraph Reward ["Reward Calculator"]
        H["_compute_reward<br/><br/>dt_h = dt_s /3600 ({dt_h} h)<br/><br/>%% H2 Cost<br/>if p_fc >0:<br/>  Δm_H2 = (p_fc * dt_h) / (η_fc * LHV)  // kg<br/>cost_h2 = c_h2 * Δm_H2 ({c_h2} €/kg)<br/>else: cost_h2=0, Δm_H2=0<br/><br/>%% Grid Cost (net charging only)<br/>p_chg = max(-p_batt_shielded, 0)<br/>ΔE_chg = p_chg * dt_h * η_chg  // kWh<br/>cost_grid = c_grid * ΔE_chg ({c_grid} €/kWh)<br/><br/>%% Smoothness<br/>action_diff = ||action - last_action||_2<br/>penalty_smooth = λ_smooth * action_diff ({lambda_smooth})<br/><br/>%% Delay (cumulative, renderer-enabled)<br/>delay_s = cumulative_delay (from speed_error vs target)<br/>penalty_delay = λ_delay * delay_s * dt_s ({lambda_delay})<br/><br/>%% Unmet<br/>penalty_unmet = λ_unmet * p_unmet ({lambda_unmet})<br/><br/>reward = -(cost_h2 + cost_grid + penalty_smooth + penalty_delay) - penalty_unmet<br/>%% Negative costs; higher better<br/><br/>%% Episode totals += this step<br/>violations += shielded.violations<br/><br/>Return: float(reward)"]
    end

    %% Main Flow
    Reset["Start Episode<br/>(Reset)"] --> DriverInit["Driver.reset<br/>Randomize: mass U[{mass_min},{mass_max}]t,<br/>SOC U[{config.battery.soc_init_min},{config.battery.soc_init_max}], tank U[{config.fuel_cell.tank_init_min},{config.fuel_cell.tank_init_max}],<br/>len U[{config.sim.episode_steps_min},{config.sim.episode_steps_max}] steps"] --> PlantInit["Plant.reset<br/>state = (SOC, tank={config.fuel_cell.tank_init_max}, speed=0,<br/>p_fc/batt/unmet=0, dist=0,...)"]
    PlantInit --> B
    B -.->|"p_req_next (precompute)"| G
    B -.->|"grade, next_stop_time, is_dwell"| D
    G -.->|"state (SOC,tank,speed,...)"| D
    G -.->|"p_unmet, Δm_H2, p_chg"| H
    E -.->|"last_action"| D
    A --> E
    E --> F
    F -.->|"p_fc/batt_shielded, violations"| G
    F -.->|"violations"| H
    G --> H
    H --> Check["Check Termination<br/>terminated = SOC ≤{soc_hard_min} or tank ≤{tank_hard_min}<br/>step +=1; truncated = step ≥ len<br/>If dwell: speed=0"]
    Check -->|Continue| D
    Check -->|End| End["Episode End<br/>Info: totals (reward, costs,<br/>violations, dist_km)<br/>Optional Render<br/>(power flows, SOC/tank,<br/>delays, schedule)"]
    End -.->|"Reset"| Reset

    style Reset fill:#e1f5fe
    style Check fill:#fff3e0
    style End fill:#f3e5f5
    style E fill:#e8f5e8
    style Agent fill:#e3f2fd
    style Shield fill:#f3e5f5
    style Plant fill:#e8f5e8
    style Driver fill:#fff3e0
    style Observation fill:#f1f8e9
    style Reward fill:#fce4ec"""
    
    return mermaid

def main():
    conf_path = project_root / "conf.yaml"
    config = Config.from_yaml(conf_path)
    
    mmd_content = generate_mermaid(config)
    
    docs_dir = project_root / "docs"
    docs_dir.mkdir(exist_ok=True)
    
    mmd_path = docs_dir / "env0_diagram.mmd"
    with open(mmd_path, "w") as f:
        f.write(mmd_content)
    
    print(f"Generated {mmd_path}")

if __name__ == "__main__":
    main()
