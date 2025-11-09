import os
import sys
from pathlib import Path
import subprocess

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from rl_hyb_train import Config

def generate_dot(config: Config) -> str:
    """Generate Simulink-style block diagram in DOT for Graphviz."""
    # Extract constants (same as before)
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
    davis_a = 5000.0
    davis_b = 100.0
    davis_c = 5.0
    mass_min = config.randomization.passenger_mass_tons_min
    mass_max = config.randomization.passenger_mass_tons_max

    # DOT code for Simulink-like block diagram
    dot = f"""digraph Env0 {{
    rankdir=TB;  // Top to bottom layout like Simulink
    node [shape=box, style=filled, fontsize=10, fontname="Arial"];
    edge [fontsize=9, fontname="Arial"];

    // Main blocks
    Reset [label="Reset\\nRandomize: mass U[{mass_min}-{mass_max}]t\\nSOC U[{config.battery.soc_init_min}-{config.battery.soc_init_max}]\\ntank U[{config.fuel_cell.tank_init_min}-{config.fuel_cell.tank_init_max}]\\nlen U[{config.sim.episode_steps_min}-{config.sim.episode_steps_max}] steps", fillcolor=lightgray];

    Driver [label="Driver Block\\nTraction Demand Generation\\n\\nTarget v from profile (segments: dwell/accel/cruise)\\nPID: a = {config.driver.speed_pid_kp}*e + {config.driver.speed_pid_ki}*int + {config.driver.speed_pid_kd}*d\\nConstraints: accel ±0.20/-0.25 m/s², jerk 0.10 m/s³\\nP_req = loss ({p_loss} kW) + (a / gain) + bias\\nFilter: α={alpha_preq:.2f} (τ={tau_preq}s)\\nRate: 1000 kW/s\\nOutput: p_req_filtered", fillcolor=lightblue];

    Plant [label="Plant Dynamics (Integrator)\\n\\nAux: p_aux = {p_aux_base} kW + N(0,{aux_sigma})\\nDemand: p_dem = p_req + p_aux\\n\\nPower Balance: p_supply = p_fc + p_batt * {eta_dis}\\np_unmet = max(p_dem - p_supply, 0)\\np_delivered = p_supply - p_aux - p_unmet\\n\\nSOC Integrator: ΔSOC = -dis / ({e_batt} * {eta_dis}) * dt_h + chg * {eta_chg} / {e_batt} * dt_h\\nH2 Tank: Δm = p_fc * dt_h / ({eta_fc} * {lh_v}) ; tank -= Δm / {tank_cap} kg\\nKinematics: F_net = F_traction - F_davis - F_grade\\na = F_net / mass; v += a * dt (clip 0-{v_max} m/s)\\ndist += v * dt / 1000 km", fillcolor=lightgreen, shape=rectangle];

    Shield [label="Shield (Constraint Block)\\nSafety Enforcement\\n\\np_fc = clip(fc_frac * {p_fc_max}, 0, {p_fc_max})\\nRamp: |Δp_fc| ≤ {ramp_rate} kW/s\\nIf tank ≤ {tank_hard_min}: p_fc = 0\\n\\nSOC Corridor ({soc_soft_min}-{soc_soft_max}):\\nLow: block dis, force chg if very low\\nHigh: block chg, limit dis to 0.5*{p_batt_max_dis} kW\\nHard: SOC ≤{soc_hard_min}: only chg/0; ≥1.00: only dis/0\\nC-rate: clip p_batt [-{p_batt_max_chg}, {p_batt_max_dis}] kW\\nOutput: p_fc/batt_shielded, violations", fillcolor=lightcoral];

    Agent [label="Agent (EMS Policy)\\nExternal RL\\nInput: obs (12D [-1,1])\\nOutput: action = [fc_frac (0-1), batt_cmd (-1 to 1)]", fillcolor=lightyellow];

    Obs [label="Observation Builder\\n12D Vector (Box -1 to 1)\\n\\nobs[0]: (v / {v_max})*2-1\\nobs[1]: noisy SOC (σ={soc_noise}) *2-1\\nobs[2]: noisy tank (σ={tank_noise}) *2-1\\nobs[3]: time-to-stop /600s *2-1 or phase\\nobs[4]: grade /5% *2-1 (±5% norm)\\nobs[5]: p_req_filt / {p_req_max} kW *2-1\\nobs[6]: trend (Δp_req/dt /1000 kW/s)*2-1\\nobs[7]: last_fc *2-1\\nobs[8]: last_batt (-1 to 1)\\nobs[9-11]: N(0, {noise_std}) noise", fillcolor=lightpink];

    Reward [label="Reward Calculator\\nNegative Costs\\n\\nH2: {c_h2} €/kg * Δm_H2 (if p_fc>0)\\nGrid: {c_grid} €/kWh * p_chg * dt_h * {eta_chg} (chg only)\\nSmooth: {lambda_smooth} * ||action - last||_2\\nDelay: {lambda_delay} * delay_s * dt ({lambda_delay})\\nUnmet: {lambda_unmet} * p_unmet\\nr = -(h2 + grid + smooth + delay) - unmet\\nHigher better (min cost)", fillcolor=lightcyan];

    Term [label="Termination Check\\nterminated: SOC ≤{soc_hard_min} or tank ≤{tank_hard_min}\\ntruncated: step ≥ len\\nIf dwell: v=0", fillcolor=lightgray, shape=diamond];

    // Flows (signals)
    Reset -> Driver [label="init episode"];
    Driver -> Plant [label="p_req_kw (filtered)"];
    Driver -> Obs [label="grade, next_stop, dwell"];
    Plant -> Obs [label="state (SOC, tank, v, ...)"];
    Plant -> Reward [label="p_unmet, Δm_H2, p_chg"];
    Obs -> Agent [label="obs (12D)"];
    Agent -> Shield [label="action [fc_frac, batt_cmd]"];
    Shield -> Plant [label="p_fc/batt_shielded"];
    Shield -> Reward [label="violations"];
    Plant -> Term [label="state update"];
    Term -> Obs [label="next obs (loop)"];
    Term -> Reset [label="end episode (info totals)" labeldistance=2];
    Agent -> Obs [label="last_action (feedback)"];

    // Colors and layout
    {{"rank=same; Obs Agent Shield"}}
    {{"rank=same; Driver Plant"}}
    {{"rank=same; Reward Term"}}
}}"""
    
    return dot

def main():
    conf_path = project_root / "conf.yaml"
    config = Config.from_yaml(conf_path)
    
    dot_content = generate_dot(config)
    
    docs_dir = project_root / "docs"
    docs_dir.mkdir(exist_ok=True)
    
    dot_path = docs_dir / "env0_simulink.dot"
    with open(dot_path, "w") as f:
        f.write(dot_content)
    
    print(f"Generated DOT: {dot_path}")
    
    # Generate PNG if dot available
    png_path = docs_dir / "env0_simulink.png"
    try:
        # Check if dot is available
        check = subprocess.run(['dot', '-V'], capture_output=True)
        if check.returncode == 0:
            subprocess.run([
                'dot', '-Tpng', str(dot_path), '-o', str(png_path)
            ], check=True, capture_output=True)
            print(f"Generated PNG: {png_path}")
        else:
            print("Graphviz (dot) not found. Install with 'sudo apt install graphviz' and run:")
            print(f"  dot -Tpng {dot_path} -o {png_path}")
    except subprocess.CalledProcessError as e:
        print(f"Graphviz error: {e}")
        print(f"DOT file generated: {dot_path}. Run manually: dot -Tpng {dot_path} -o {png_path}")
    
if __name__ == "__main__":
    main()
