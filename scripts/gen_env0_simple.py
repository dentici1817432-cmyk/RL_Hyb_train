import os
import sys
from pathlib import Path
import subprocess

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from rl_hyb_train import Config

def generate_simple_dot(config: Config) -> str:
    """Generate simplified block diagram in DOT: Driver -> EMS -> Plant loop."""
    # Extract key constants
    p_fc_max = config.fuel_cell.p_fc_max_kw
    p_batt_max_dis = config.battery.p_batt_max_discharge_kw
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

    # Simple DOT
    dot = f"""digraph SimpleEnv0 {{
    rankdir=LR;  // Left to right flow
    node [shape=box, style=filled, fontsize=12, fontname="Arial", width=2, height=1];
    edge [fontsize=10, fontname="Arial"];

    Driver [label="Driver (Traction Demand)\\n\\nSpeed Profile: segments (dwell/accel/cruise)\\nPID Controller for v_tracking\\nP_req = loss + (a / {kinematic_gain} m/s/W) + bias\\nLow-pass filter (τ={tau_preq} s, α≈0.25)\\nRate limit: 1000 kW/s\\n\\nOutput: P_req kW", fillcolor=lightblue];

    EMS [label="EMS Policy + Shield\\n(External RL or Rule-based)\\n\\nInput: P_req, SOC, tank\\nPolicy: fc_frac (0-1), batt_cmd (-1 to 1)\\nShield: p_fc = fc_frac * {p_fc_max} kW (ramp 40 kW/s)\\np_batt = batt_cmd * {p_batt_max_dis} kW (C-rate ±)\\nSOC corridor: 0.20-0.90 (block chg/dis)\\nOutput: p_fc_kw, p_batt_kw (shielded)", fillcolor=lightyellow];

    Plant [label="Train Model Dynamics (Plant)\\n\\nAux Load: p_aux = {p_aux_base} kW + noise\\nDemand: p_dem = P_req + p_aux\\nRegen: if P_req<0, alloc aux then batt (η implied)\\nBalance: p_supply = p_fc + p_batt * {eta_dis}\\np_unmet = max(p_dem - p_supply, 0)\\n\\nSOC: ΔSOC = -dis / ({e_batt} * {eta_dis}) * dt + chg * {eta_chg} / {e_batt} * dt\\nH2: Δm = p_fc / ({eta_fc} * {lh_v}); tank -= Δm / cap\\nKinematics: F_net = (p_deliv*1000/v) - ( {davis_a} + {davis_b}v + {davis_c}v² ) - grade*m*g\\nv += (F_net / mass) * dt (clip 0-{config.plant.v_max_mps} m/s)\\ndist += v * dt /1000 km", fillcolor=lightgreen];

    Outputs [label="Simulation Outputs\\n- SOC (0-1)\\n- H2 tank level (0-1)\\n- Speed (m/s)\\n- Distance (km)\\n- p_unmet (kW)\\n- Violations", fillcolor=lightgray, shape=plaintext];

    // Simple flow
    Driver -> EMS [label="P_req kW\\n(target profile)"];
    EMS -> Plant [label="p_fc_kw, p_batt_kw\\n(shielded powers)"];
    Plant -> Driver [label="speed_mps\\n(feedback)"];
    Plant -> Outputs [label="state: SOC, tank, v, dist, unmet"];

    // Loop back
    Outputs -> Loop [label="use in next step\\n(loop)"];
    Loop [label="Time Step\\ndt=1 s", fillcolor=lightgray, shape=circle];
}}"""

    return dot

def main():
    conf_path = project_root / "conf.yaml"
    config = Config.from_yaml(conf_path)
    
    dot_content = generate_simple_dot(config)
    
    docs_dir = project_root / "docs"
    docs_dir.mkdir(exist_ok=True)
    
    dot_path = docs_dir / "env0_simple.dot"
    with open(dot_path, "w") as f:
        f.write(dot_content)
    
    print(f"Generated Simple DOT: {dot_path}")
    
    # Generate PNG if dot available
    png_path = docs_dir / "env0_simple.png"
    try:
        check = subprocess.run(['dot', '-V'], capture_output=True)
        if check.returncode == 0:
            subprocess.run([
                'dot', '-Tpng', str(dot_path), '-o', str(png_path)
            ], check=True, capture_output=True)
            print(f"Generated Simple PNG: {png_path}")
        else:
            print("Graphviz (dot) not found. Install with 'sudo apt install graphviz' and run:")
            print(f"  dot -Tpng {dot_path} -o {png_path}")
    except subprocess.CalledProcessError as e:
        print(f"Graphviz error: {e}")
        print(f"DOT file generated: {dot_path}. Run manually: dot -Tpng {dot_path} -o {png_path}")
    
if __name__ == "__main__":
    main()
