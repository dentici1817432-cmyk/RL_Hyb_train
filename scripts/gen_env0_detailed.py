import os
import sys
from pathlib import Path
import subprocess

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from rl_hyb_train import Config

def generate_detailed_dot(config: Config) -> str:
    """Generate detailed block diagram with subgraphs for EMS and Plant."""
    # Extract constants
    p_fc_max = config.fuel_cell.p_fc_max_kw
    p_batt_max_dis = config.battery.p_batt_max_discharge_kw
    p_batt_max_chg = config.battery.p_batt_max_charge_kw
    e_batt = config.battery.e_batt_kwh
    eta_dis = config.battery.eta_discharge
    eta_chg = config.battery.eta_charge
    eta_fc = config.fuel_cell.eta_fc
    lh_v = config.fuel_cell.h2_lhv_kwh_per_kg
    p_aux_base = config.plant.p_aux_base_watts / 1000.0
    kinematic_gain = config.plant.kinematic_gain_mps_per_watt
    davis_a = 5000.0
    davis_b = 100.0
    davis_c = 5.0
    tau_preq = config.driver.p_req_smoothing_tau_s
    ramp_rate = config.fuel_cell.ramp_kw_per_s
    soc_soft_min = config.shield.soc_soft_min
    soc_soft_max = config.shield.soc_soft_max

    # Detailed DOT with subgraphs
    dot = f"""digraph DetailedEnv0 {{
    rankdir=LR;
    node [shape=box, style=filled, fontsize=10, fontname="Arial"];
    edge [fontsize=9, fontname="Arial"];
    graph [compound=true, newrank=true, splines=ortho, ranksep=1.0, nodesep=0.6, concentrate=true];
    edge [arrowsize=0.8, penwidth=1.3, color="#333333"];
    node [margin="0.08,0.06"];

    Driver [label="Driver (Traction Demand)\\n\\nSpeed Profile: segments (dwell/accel/cruise/climb)\\nPID Controller for v_tracking\\nP_req = loss + (a / {kinematic_gain} m/s/W) + bias\\nLow-pass filter (τ={tau_preq} s, α≈0.25)\\nRate limit: 1000 kW/s\\n\\nOutput: P_req kW", fillcolor=lightblue];

    subgraph cluster_EMS {{
        label="EMS (Energy Management System)";
        style=filled; color=lightyellow;

        Policy [label="Policy (External)\\n\\nInput: P_req, SOC, tank, v\\nOutput: fc_frac (0-1), batt_cmd (-1 to 1)\\n(e.g., RL policy or rule-based)", fillcolor=lightyellow, shape=rectangle];

        subgraph cluster_Shield {{
            label="Shield (Constraints)";
            style=filled; color=lightcoral;

            Ramp [label="FC Ramp Limiter\\n|Δp_fc| ≤ {ramp_rate} kW/s\\np_fc_cmd = fc_frac * {p_fc_max} kW\\nclip p_fc [0, {p_fc_max}] kW\\nIf tank ≤ 0.02: p_fc = 0", fillcolor=lightcoral];

            SOC_Guard [label="SOC Guard (Corridor {soc_soft_min}-{soc_soft_max})\\nLow: if SOC < {soc_soft_min}: block dis, force chg\\nHigh: if SOC > {soc_soft_max}: block chg, limit dis 0.5*{p_batt_max_dis} kW\\nHard min {config.battery.soc_hard_min}: only chg/0\\nHard max 1.00: only dis/0", fillcolor=lightcoral];

            Crate [label="C-rate Limiter\\nclip p_batt [-{p_batt_max_chg}, {p_batt_max_dis}] kW\\nP_chg_max = {p_batt_max_chg} kW\\nP_dis_max = {p_batt_max_dis} kW", fillcolor=lightcoral];

            Tank_Check [label="Tank Check\\nIf tank_level ≤ 0.02: p_fc = 0\\nNo H2: disable FC", fillcolor=lightcoral];
        }}

        // Shield internal flows
        Policy -> Ramp [xlabel="fc_frac"];
        Policy -> SOC_Guard [xlabel="batt_cmd"];
        Policy -> Crate [xlabel="batt_cmd"];
        Policy -> Tank_Check [xlabel="fc_frac"];
        Ramp -> Output_Shield [xlabel="p_fc (ramped)"];
        SOC_Guard -> Output_Shield [xlabel="p_batt (SOC-safe)"];
        Crate -> Output_Shield [xlabel="p_batt (C-rate)"];
        Tank_Check -> Output_Shield [xlabel="p_fc (tank-safe)"];
        Output_Shield [label="Shield Output\\np_fc_kw, p_batt_kw, violations", fillcolor=lightyellow, shape=plaintext];
    }}

    subgraph cluster_Plant {{
        label="Train Model Dynamics (Plant)";
        style=filled; color=lightgreen;

        Aux [label="Aux Load Generator\\np_aux = {p_aux_base} kW + N(0, {config.randomization.aux_bias_rw_sigma_kw})\\nRandom walk bias (hidden)", fillcolor=yellow, shape=ellipse];

        Regen [label="Regen Allocator\\n(if P_req <0 & v >0.01 m/s)\\nregen_total = |P_req|\\nregen_aux = min(total, p_aux)\\npost_aux = max(0, total - aux)\\ncapture = min(chg_cmd, post_aux)\\nfriction = max(0, total - aux - capture)\\np_brake_regen = aux + capture", fillcolor=orange];

        Balance [label="Power Balance\\nDC demand/supply accounting\\n\\np_dem = P_req + p_aux\\n(p_req includes traction sign)\\n\\np_supply = p_fc + (p_batt_dis * {eta_dis})\\n\\np_unmet = max(p_dem - p_supply, 0)\\n\\np_delivered = p_supply - p_aux - p_unmet\\n\\nFC excess → charge (if room)\\nExpose: p_batt_dis, p_chg, p_delivered", fillcolor=pink, shape=circle];

        SOC [label="SOC Integrator\\n(Coulomb Counting)\\nΔSOC = -p_dis / ({e_batt} * {eta_dis}) * dt_h\\n     + p_chg * {eta_chg} / {e_batt} * dt_h\\n(dt_h = {config.sim.dt_seconds}/3600; clip [0,1])", fillcolor=lightyellow, shape=doublecircle];

        H2 [label="H2 Tank Integrator\\nIf p_fc >0:\\nΔm_H2 = p_fc * dt_h / ({eta_fc} * {lh_v}) kg\\ntank -= Δm_H2 / {config.fuel_cell.tank_capacity_kg} kg\\n(max(0, tank); LHV={lh_v} kWh/kg)", fillcolor=lightyellow, shape=doublecircle];

        Kinematics [label="Kinematics (Davis + Grade)\\np_delivered = p_fc + (p_batt * {eta_dis}) - p_aux\\np_unmet = max((P_req + p_aux) - (p_fc + p_batt*{eta_dis}), 0)\\n\\nIf stopped (v<0.01 & P_req≤0): v=0\\nElse:\\nF_traction = p_delivered *1000 / v (or max start)\\nF_resist = {davis_a} + {davis_b}*v + {davis_c}*v²\\nF_grade = mass * 9.81 * grade/100\\n(mass U[{config.randomization.passenger_mass_tons_min}-{config.randomization.passenger_mass_tons_max}] t)\\nF_net = traction - resist - grade\\na = F_net / (mass*1000)\\nv_new = clip(v + a*dt, 0, {config.plant.v_max_mps}) m/s\\ndist += v*dt /1000 km\\nIf dwell: v=0", fillcolor=lightgreen];
        {{ rank=same; SOC; H2; }}
    }}

    Outputs [label="Physics Outputs\\n- SOC (0-1)\\n- H2 tank level (0-1)\\n- Speed (m/s)\\n- Distance (km)\\n- p_unmet (kW)\\n- Violations", fillcolor=lightgray, shape=plaintext];

    // Main flows (forward)
    Driver -> Policy [xlabel="P_req kW\n(target profile)", minlen=2, color="#7f7f7f", style=dashed];
    // Route shielded powers through Power Balance
    Output_Shield -> Balance [xlabel="p_fc_kw, p_batt_kw"];
    Balance -> SOC [xlabel="p_batt_dis, p_chg", color="#1f77b4"];
    Balance -> H2 [xlabel="p_fc", color="#1f77b4"];
    Balance -> Kinematics [xlabel="p_delivered", minlen=2, color="#1f77b4"];
    SOC -> Outputs [xlabel="SOC", color="#7f7f7f"];
    H2 -> Outputs [xlabel="tank", color="#7f7f7f"];
    Kinematics -> Outputs [xlabel="v, dist, p_unmet", color="#7f7f7f"];

    // Feedback loops (dashed, red; non-constraining)
    Kinematics -> Driver [xlabel="speed_mps\n(feedback)", color=red, style=dashed, penwidth=2, constraint=false];
    SOC -> Policy [xlabel="SOC\n(feedback)", color=red, style=dashed, penwidth=2, constraint=false];
    H2 -> Policy [xlabel="tank_level\n(feedback)", color=red, style=dashed, penwidth=2, constraint=false];
    Kinematics -> Policy [xlabel="speed_mps\n(feedback)", color=red, style=dashed, penwidth=2, constraint=false];

    // Plant internal flows (forward)
    Driver -> Kinematics [xlabel="P_req kW\n(requested traction)", minlen=2, color="#7f7f7f", style=dashed];
    // Inputs to Power Balance
    Driver -> Balance [xlabel="P_req kW", color="#7f7f7f", style=dashed];
    Aux -> Balance [xlabel="p_aux", color="#1f77b4"];
    Aux -> Regen [xlabel="p_aux", color="#1f77b4"];
    Driver -> Regen [xlabel="P_req (for braking)", color="#7f7f7f", style=dashed];
    Kinematics -> Regen [xlabel="v_mps", color=red, style=dashed, penwidth=2, constraint=false];
    Output_Shield -> Regen [xlabel="chg_cmd from p_batt", color="#7f7f7f", style=dashed];
    Regen -> Balance [xlabel="p_brake_regen, friction", color="#1f77b4"];

    // Loop back
    Outputs -> Loop [xlabel="use in next step\n(loop)", color=red, style=dashed, penwidth=2, constraint=false];
    Loop [label="Time Step\ndt=1 s", fillcolor=lightgray, shape=circle];
}}"""

    return dot

def main():
    conf_path = project_root / "conf.yaml"
    config = Config.from_yaml(conf_path)
    
    dot_content = generate_detailed_dot(config)
    
    docs_dir = project_root / "docs"
    docs_dir.mkdir(exist_ok=True)
    
    dot_path = docs_dir / "env0_detailed.dot"
    with open(dot_path, "w") as f:
        f.write(dot_content)
    
    print(f"Generated Detailed DOT: {dot_path}")
    
    # Generate PNG if dot available
    png_path = docs_dir / "env0_detailed.png"
    try:
        check = subprocess.run(['dot', '-V'], capture_output=True)
        if check.returncode == 0:
            subprocess.run([
                'dot', '-Tpng', str(dot_path), '-o', str(png_path)
            ], check=True, capture_output=True)
            print(f"Generated Detailed PNG: {png_path}")
        else:
            print("Graphviz (dot) not found. Install with 'sudo apt install graphviz' and run:")
            print(f"  dot -Tpng {dot_path} -o {png_path}")
    except subprocess.CalledProcessError as e:
        print(f"Graphviz error: {e}")
        print(f"DOT file generated: {dot_path}. Run manually: dot -Tpng {dot_path} -o {png_path}")
    
if __name__ == "__main__":
    main()
