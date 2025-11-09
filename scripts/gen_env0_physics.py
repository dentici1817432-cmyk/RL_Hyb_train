import os
import sys
from pathlib import Path
import subprocess

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from rl_hyb_train import Config

def generate_physics_dot(config: Config) -> str:
    """Generate physics simulation block diagram in DOT (Driver-Shield-Plant focus)."""
    # Extract constants
    dt_s = config.sim.dt_seconds
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
    tau_preq = config.driver.p_req_smoothing_tau_s
    p_aux_base = config.plant.p_aux_base_watts / 1000.0
    aux_sigma = config.randomization.aux_bias_rw_sigma_kw
    v_max = config.plant.v_max_mps
    alpha_preq = dt_s / (tau_preq + dt_s)
    p_loss = config.driver.p_loss_watts / 1000.0
    kinematic_gain = config.plant.kinematic_gain_mps_per_watt
    davis_a = 5000.0
    davis_b = 100.0
    davis_c = 5.0
    mass_min = config.randomization.passenger_mass_tons_min
    mass_max = config.randomization.passenger_mass_tons_max
    kp = config.driver.speed_pid_kp
    ki = config.driver.speed_pid_ki
    kd = config.driver.speed_pid_kd
    accel_lim = config.driver.speed_tracking_accel_limit_mps2
    brake_lim = config.driver.speed_tracking_brake_limit_mps2

    # DOT for physics diagram
    dot = f"""digraph PhysicsSim {{
    rankdir=TB;
    node [shape=box, style=filled, fontsize=10, fontname="Arial"];
    edge [fontsize=9, fontname="Arial"];

    // Main blocks
    Driver [label="Driver (Traction Demand)\\nSpeed Profile & PID Controller\\n\\nTarget v from segments (dwell/accel/cruise/climb)\\nPID: a = {kp}*e + {ki}*int + {kd}*d\\nAccel limits: +{accel_lim} / -{brake_lim} m/s²\\nP_req_raw = p_loss ({p_loss} kW) + (a / {kinematic_gain} m/s/W) + bias\\nFilter: α={alpha_preq:.2f} (τ={tau_preq} s)\\nRate limit: {config.driver.p_req_rate_limit_kw_per_s} kW/s\\nOutput: p_req_kw", fillcolor=lightblue];

    Shield [label="Shield (Action Constraints)\\nSafety Enforcement\\n\\nInput: fc_frac (0-1), batt_cmd (-1 to 1)\\np_fc = fc_frac * {p_fc_max} kW; clip [0, {p_fc_max}]\\nRamp: |Δp_fc| ≤ {ramp_rate} kW/s\\nIf tank ≤ {tank_hard_min}: p_fc = 0\\n\\nSOC Corridor ({soc_soft_min}-{soc_soft_max}):\\nLow SOC: block discharge, force charge if <{soc_soft_min}-0.05\\nHigh SOC: block charge, limit dis to 0.5*{p_batt_max_dis} kW\\nC-rate: clip p_batt [-{p_batt_max_chg}, {p_batt_max_dis}] kW\\nOutput: p_fc_kw, p_batt_kw, violations", fillcolor=lightcoral];

    subgraph cluster_Plant ["Plant (Physical Dynamics)"] {{
        label="Plant Subsystem";
        style=filled; color=lightgreen;

        Aux [label="Aux Load Generator\\np_aux = {p_aux_base} kW + N(0, {aux_sigma})\\nRandom walk bias", fillcolor=yellow, shape=ellipse];

        Regen [label="Regen Allocator (if p_req <0 & v >0.01 m/s)\\nregen_total = |p_req|\\nregen_aux = min(total, p_aux)\\npost_aux = max(0, total - aux)\\ncapture = min(chg_cmd, post_aux)\\nfriction = max(0, total - aux - capture)\\np_brake_regen = aux + capture", fillcolor=orange];

        Balance [label="Power Balance Sum\\np_dem = p_req + p_aux\\np_supply = p_fc + (p_batt_dis * {eta_dis})\\np_unmet = max(p_dem - p_supply, 0)\\np_delivered = p_supply - p_aux - p_unmet\\nFC excess = max(0, p_fc - p_dem) for chg", fillcolor=pink, shape=circle];

        SOC [label="SOC Integrator (Coulomb Counting)\\nΔSOC = -p_dis / ({e_batt} * {eta_dis}) * dt_h\\n     + p_chg * {eta_chg} / {e_batt} * dt_h\\n(dt_h = {dt_s}/3600; clip [0,1])", fillcolor=lightyellow, shape=doublecircle];

        H2 [label="H2 Tank Integrator\\nIf p_fc >0:\\nΔm_H2 = p_fc * dt_h / ({eta_fc} * {lh_v}) kg\\ntank -= Δm_H2 / {tank_cap} kg\\n(max(0, tank); LHV=33.3 kWh/kg)", fillcolor=lightyellow, shape=doublecircle];

        Kinematics [label="Kinematics (Davis + Grade)\\nIf stopped (v<0.01 & p_req≤0): v=0\\nElse:\\nF_traction = p_delivered *1000 / v (or max start)\\nF_resist = {davis_a} + {davis_b}*v + {davis_c}*v²\\nF_grade = mass * 9.81 * grade/100 (mass U[{mass_min}-{mass_max}] t)\\nF_net = traction - resist - grade\\na = F_net / (mass*1000)\\nv_new = clip(v + a*dt, 0, {v_max}) m/s\\ndist += v*dt /1000 km\\nIf dwell: v=0", fillcolor=lightgreen];
    }}

    // Physics flows
    Driver -> Aux [label="p_req_kw"];
    Driver -> Kinematics [label="target_v (profile)"];
    Aux -> Balance [label="p_aux"];
    Kinematics -> Driver [label="v_mps (feedback)"];
    Kinematics -> Aux [label="v for bias?"];  // Optional

    // Assume external action to Shield
    Shield -> Balance [label="p_fc_kw, p_batt_kw"];

    // Plant internals
    Aux -> Regen [label="p_aux"];
    Driver -> Regen [label="p_req (for braking)"];
    Kinematics -> Regen [label="v_mps"];
    Regen -> Balance [label="p_brake_regen, friction"];
    Shield -> Regen [label="chg_cmd from p_batt"];
    Balance -> SOC [label="p_batt_dis, p_chg"];
    Balance -> H2 [label="p_fc"];
    Balance -> Kinematics [label="p_delivered"];
    Kinematics -> Balance [label="v for traction calc"];

    // Outputs
    Plant -> Outputs [label="SOC, tank, v, dist, p_unmet"];
    Outputs [label="Physics Outputs\\n(SOC, H2 level, speed, distance, unmet power)", fillcolor=lightgray, shape=plaintext];

    // Shield inputs (external)
    External_Action [label="External Action\\n(fc_frac, batt_cmd)", fillcolor=lightyellow, shape=plaintext];
    External_Action -> Shield [label="action"];
}}"""

    return dot

def main():
    conf_path = project_root / "conf.yaml"
    config = Config.from_yaml(conf_path)
    
    dot_content = generate_physics_dot(config)
    
    docs_dir = project_root / "docs"
    docs_dir.mkdir(exist_ok=True)
    
    dot_path = docs_dir / "env0_physics.dot"
    with open(dot_path, "w") as f:
        f.write(dot_content)
    
    print(f"Generated Physics DOT: {dot_path}")
    
    # Generate PNG if dot available
    png_path = docs_dir / "env0_physics.png"
    try:
        check = subprocess.run(['dot', '-V'], capture_output=True)
        if check.returncode == 0:
            subprocess.run([
                'dot', '-Tpng', str(dot_path), '-o', str(png_path)
            ], check=True, capture_output=True)
            print(f"Generated Physics PNG: {png_path}")
        else:
            print("Graphviz (dot) not found. Install with 'sudo apt install graphviz' and run:")
            print(f"  dot -Tpng {dot_path} -o {png_path}")
    except subprocess.CalledProcessError as e:
        print(f"Graphviz error: {e}")
        print(f"DOT file generated: {dot_path}. Run manually: dot -Tpng {dot_path} -o {png_path}")
    
if __name__ == "__main__":
    main()
