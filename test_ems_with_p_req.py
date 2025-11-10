#!/usr/bin/env env python
"""
Standalone test script for EMS + train dynamics verification.

Tests the EMS logic and train physics with synthetic P_req curves,
bypassing the driver to isolate and verify:
- EMS power allocation logic
- Plant dynamics (speed, SOC, H2)
- Shield constraint enforcement
- Spec.md compliance

Usage:
    python test_ems_with_p_req.py [--scenario SCENARIO_NAME]
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse

from rl_hyb_train.config import Config
from rl_hyb_train.plant import Plant
from rl_hyb_train.powerflow import (
    compute_power_balance,
    resolve_battery_flow,
    compute_regen_flow,
    soc_delta,
    h2_consumption_kg,
)
from rl_hyb_train.shield import Shield
from rl_hyb_train.policies.baseline_ems import BaselineEMS
from rl_hyb_train.plotting import plot_history


# ============================================================================
# SYNTHETIC P_REQ CURVE GENERATION
# ============================================================================

def generate_synthetic_p_req_curves(dt=1.0, duration=600):
    """
    Generate multiple synthetic P_req test scenarios.

    Args:
        dt: Timestep in seconds
        duration: Total duration in seconds

    Returns:
        Dictionary of {scenario_name: (time_array, p_req_array)}
    """
    n_steps = int(duration / dt)
    time = np.arange(n_steps) * dt

    scenarios = {}

    # Scenario 1: Steady cruise (200 kW constant)
    scenarios["steady_cruise"] = (
        time.copy(),
        np.full(n_steps, 200.0)
    )

    # Scenario 2: Step changes (test FC ramp limits and battery support)
    p_req_steps = np.zeros(n_steps)
    p_req_steps[0:100] = 0.0      # Idle
    p_req_steps[100:250] = 400.0  # Jump to 400 kW
    p_req_steps[250:400] = 200.0  # Drop to 200 kW
    p_req_steps[400:550] = 600.0  # Jump to 600 kW (high demand)
    p_req_steps[550:] = 100.0     # Coast
    scenarios["step_changes"] = (time.copy(), p_req_steps)

    # Scenario 3: Linear ramp up and down (test tracking)
    p_req_ramp = np.zeros(n_steps)
    ramp_up_end = n_steps // 3
    ramp_down_start = 2 * n_steps // 3
    p_req_ramp[0:ramp_up_end] = np.linspace(0, 800, ramp_up_end)
    p_req_ramp[ramp_up_end:ramp_down_start] = 800.0
    p_req_ramp[ramp_down_start:] = np.linspace(800, 0, n_steps - ramp_down_start)
    scenarios["ramp_profile"] = (time.copy(), p_req_ramp)

    # Scenario 4: Regenerative braking periods
    p_req_regen = np.zeros(n_steps)
    p_req_regen[0:150] = 300.0    # Acceleration
    p_req_regen[150:250] = -200.0 # Regen braking
    p_req_regen[250:400] = 400.0  # Acceleration
    p_req_regen[400:500] = -150.0 # Regen braking
    p_req_regen[500:] = 100.0     # Coast
    scenarios["regen_profile"] = (time.copy(), p_req_regen)

    # Scenario 5: NIL-like profile (mixed cruise/accel/decel)
    p_req_nil = np.zeros(n_steps)
    # Station 0 -> 1: Accel + cruise
    p_req_nil[0:30] = np.linspace(0, 600, 30)    # Accel from stop
    p_req_nil[30:100] = 300.0                     # Cruise
    p_req_nil[100:120] = np.linspace(300, 0, 20)  # Coast to stop
    # Station 1: Dwell (P_req = 0 or small aux load)
    p_req_nil[120:180] = 0.0
    # Station 1 -> 2: Accel + cruise on grade
    p_req_nil[180:210] = np.linspace(0, 700, 30)  # Accel
    p_req_nil[210:280] = 500.0                     # Cruise uphill
    p_req_nil[280:300] = np.linspace(500, 0, 20)   # Coast to stop
    # Station 2: Dwell
    p_req_nil[300:360] = 0.0
    # Station 2 -> 3: Downhill with regen
    p_req_nil[360:390] = np.linspace(0, 200, 30)   # Modest accel
    p_req_nil[390:450] = -100.0                     # Regen on descent
    p_req_nil[450:470] = np.linspace(-100, 0, 20)  # Coast to stop
    # Station 3: Terminal dwell
    p_req_nil[470:] = 0.0
    scenarios["nil_like_profile"] = (time.copy(), p_req_nil)

    # Scenario 6: Emergency braking (friction brakes needed due to POWER limit)
    p_req_emergency = np.zeros(n_steps)
    # Accelerate to high speed
    p_req_emergency[0:60] = np.linspace(0, 800, 60)   # Strong accel
    p_req_emergency[60:200] = 600.0                    # High-speed cruise
    # Emergency stop: VERY strong braking (exceeds battery charge capacity)
    p_req_emergency[200:240] = np.linspace(600, -1400, 40)  # Rapid decel
    p_req_emergency[240:280] = -1400.0                 # Sustained heavy braking (>1000 kW limit!)
    p_req_emergency[280:300] = np.linspace(-1400, 0, 20)  # Final stop
    # Dwell after emergency stop
    p_req_emergency[300:] = 0.0
    scenarios["emergency_braking"] = (time.copy(), p_req_emergency)

    # Scenario 7: High SOC regen rejection (friction brakes needed due to SOC limit)
    # Simulates descending a long grade with battery nearly full
    p_req_high_soc = np.zeros(n_steps)
    # Accelerate to high speed
    p_req_high_soc[0:60] = np.linspace(0, 600, 60)    # Accel to high speed
    p_req_high_soc[60:120] = 500.0                     # High-speed cruise
    # Long descent: sustained heavy braking (like going downhill)
    # Battery starts at 89.8% SOC - very close to 90% limit!
    p_req_high_soc[120:160] = np.linspace(500, -500, 40)   # Transition to descent braking
    p_req_high_soc[160:350] = -500.0                   # Long sustained braking (190s!)
    p_req_high_soc[350:380] = np.linspace(-500, 0, 30) # Final coast to stop
    # Stopped
    p_req_high_soc[380:] = 0.0
    scenarios["high_soc_regen"] = (time.copy(), p_req_high_soc)

    return scenarios


# ============================================================================
# TEST HARNESS
# ============================================================================

class EMSTestHarness:
    """Standalone test harness for EMS + plant with fixed P_req."""

    def __init__(self, config_path="conf.yaml"):
        """Initialize components from config."""
        self.cfg = Config.from_yaml(config_path)

        # Create RNG for plant
        self.rng = np.random.default_rng(seed=42)

        # Initialize components with proper config objects
        self.plant = Plant(
            plant_config=self.cfg.plant,
            battery_config=self.cfg.battery,
            fc_config=self.cfg.fuel_cell,
            rng=self.rng,
            train_config=self.cfg.train_physics
        )
        self.shield = Shield(
            shield_config=self.cfg.shield,
            battery_config=self.cfg.battery,
            fc_config=self.cfg.fuel_cell
        )
        self.ems = BaselineEMS(self.cfg)

        # Simulation state
        self.dt = self.cfg.scenario.sim.dt_seconds
        self.time = 0.0
        self.step_count = 0

        # History for recording
        self.history = {
            "time": [],
            "p_req_kw": [],
            "p_fc_kw": [],
            "p_batt_kw": [],
            "p_batt_delivered_kw": [],
            "p_unmet_kw": [],
            "p_aux_kw": [],
            "speed_mps": [],
            "soc": [],
            "h2_level": [],
            "fc_frac": [],
            "batt_cmd": [],
            "fc_frac_shielded": [],
            "batt_cmd_shielded": [],
            "accel_mps2": [],
            "position_m": [],
            # Braking power breakdown
            "p_brake_total_kw": [],
            "p_brake_regen_kw": [],
            "p_brake_friction_kw": [],
        }

    def reset(self, soc_init=None):
        """Reset plant and EMS to initial conditions.

        Args:
            soc_init: Optional initial SOC override (0-1). If None, uses config average.
        """
        # Get initial conditions from config
        if soc_init is None:
            soc_init = (self.cfg.battery.soc_init_min + self.cfg.battery.soc_init_max) / 2.0
        tank_init = (self.cfg.fuel_cell.tank_init_min + self.cfg.fuel_cell.tank_init_max) / 2.0
        passenger_mass_tons = (
            self.cfg.randomization.passenger_mass_tons_min +
            self.cfg.randomization.passenger_mass_tons_max
        ) / 2.0
        aux_bias_sigma_kw = self.cfg.randomization.aux_bias_rw_sigma_kw

        # Reset plant with parameters
        self.plant.reset(
            soc_init=soc_init,
            tank_init=tank_init,
            passenger_mass_tons=passenger_mass_tons,
            aux_bias_sigma_kw=aux_bias_sigma_kw
        )
        self.ems.reset()
        self.time = 0.0
        self.step_count = 0

        # Clear history
        for key in self.history:
            self.history[key].clear()

    def step(self, p_req_kw):
        """
        Single simulation step with fixed P_req.

        Args:
            p_req_kw: Power request in kW

        Returns:
            Dictionary of current state
        """
        # Get current state from plant
        state = self.plant.state
        soc = state.soc
        h2_level = state.tank_level
        speed_mps = state.speed_mps

        # Prepare info dict for EMS (mimics what Env0 provides)
        # Calculate actual auxiliary power including bias (matches Plant.step() line 126)
        p_aux_kw = (self.cfg.plant.p_aux_base_watts / 1000.0) + state.aux_bias_kw
        info = {
            "p_req_kw": p_req_kw,
            "soc": soc,
            "h2_level": h2_level,
            "speed_mps": speed_mps,
            "p_aux_kw": p_aux_kw,
            "p_fc_prev_kw": state.p_fc_kw,
        }

        # EMS decides action
        # Build proper observation array (EMS reads P_req from obs, not info!)
        # Observation space from env0_env.py:319-384
        v_max_mps = self.cfg.plant.v_max_mps
        p_req_max_kw = self.cfg.observations.p_req_max_kw

        obs = np.array([
            2.0 * (speed_mps / v_max_mps) - 1.0,          # 0: speed normalized [-1, 1]
            2.0 * soc - 1.0,                               # 1: SOC normalized [-1, 1]
            2.0 * h2_level - 1.0,                          # 2: H2 level normalized [-1, 1]
            0.0,                                            # 3: time_to_next_stop (not used)
            2.0 * (p_req_kw / p_req_max_kw) - 1.0,        # 4: P_req filtered normalized [-1, 1]
            0.0,                                            # 5: P_req trend (not used)
            0.0,                                            # 6: last fc_frac
            0.0,                                            # 7: last batt_cmd
            0.0, 0.0, 0.0, 0.0                             # 8-11: noise channels
        ])

        action = self.ems.compute_action(obs, info)
        fc_frac, batt_cmd = action

        # Apply shield constraints
        shielded_action = self.shield.apply(
            fc_frac=fc_frac,
            batt_cmd=batt_cmd,
            soc=soc,
            tank_level=h2_level
        )

        # Shield returns power commands directly in kW
        p_fc_kw = shielded_action.p_fc_kw
        p_batt_kw = shielded_action.p_batt_kw

        # Calculate shielded fractions for recording
        p_fc_max_kw = self.cfg.fuel_cell.p_fc_max_kw
        p_batt_max_discharge_kw = self.cfg.battery.p_batt_max_discharge_kw
        fc_frac_shielded = p_fc_kw / p_fc_max_kw if p_fc_max_kw > 0 else 0.0
        # For batt_cmd, use max discharge/charge depending on sign
        batt_max = p_batt_max_discharge_kw if p_batt_kw >= 0 else self.cfg.battery.p_batt_max_charge_kw
        batt_cmd_shielded = p_batt_kw / batt_max if batt_max > 0 else 0.0

        # Step plant with power commands
        # Plant.step() returns PlantState dataclass
        # Note: Grade effects are handled by driver via p_req; plant doesn't take grade directly
        new_state = self.plant.step(
            p_fc_kw=p_fc_kw,
            p_batt_kw=p_batt_kw,
            p_req_kw=p_req_kw,
            dt_seconds=self.dt
        )

        # Calculate actual auxiliary power AFTER step (plant updates aux_bias during step)
        # This matches Plant.step() line 126
        p_aux_kw_actual = (self.cfg.plant.p_aux_base_watts / 1000.0) + new_state.aux_bias_kw

        # Calculate acceleration from speed change (plant doesn't expose accel directly in state)
        accel_mps2 = (new_state.speed_mps - speed_mps) / self.dt

        # Record history
        self.history["time"].append(self.time)
        self.history["p_req_kw"].append(p_req_kw)
        self.history["p_fc_kw"].append(p_fc_kw)
        self.history["p_batt_kw"].append(p_batt_kw)
        self.history["p_batt_delivered_kw"].append(new_state.p_batt_delivered_kw)
        self.history["p_unmet_kw"].append(new_state.p_unmet_kw)
        self.history["p_aux_kw"].append(p_aux_kw_actual)
        self.history["speed_mps"].append(new_state.speed_mps)
        self.history["soc"].append(new_state.soc)
        self.history["h2_level"].append(new_state.tank_level)
        self.history["fc_frac"].append(fc_frac)
        self.history["batt_cmd"].append(batt_cmd)
        self.history["fc_frac_shielded"].append(fc_frac_shielded)
        self.history["batt_cmd_shielded"].append(batt_cmd_shielded)
        self.history["accel_mps2"].append(accel_mps2)
        self.history["position_m"].append(new_state.distance_km * 1000.0)  # Convert km to m
        # Brake power breakdown
        self.history["p_brake_total_kw"].append(new_state.p_brake_total_kw)
        self.history["p_brake_regen_kw"].append(new_state.p_brake_regen_kw)
        self.history["p_brake_friction_kw"].append(new_state.p_brake_friction_kw)

        self.time += self.dt
        self.step_count += 1

        return new_state

    def run_scenario(self, time_array, p_req_array, soc_init=None):
        """
        Run full simulation with given P_req curve.

        Args:
            time_array: Time values (s)
            p_req_array: P_req values (kW)
            soc_init: Optional initial SOC override (0-1)
        """
        self.reset(soc_init=soc_init)

        for t, p_req in zip(time_array, p_req_array):
            self.step(p_req)

    def get_history_arrays(self):
        """Convert history to numpy arrays for analysis."""
        return {key: np.array(val) for key, val in self.history.items()}


# ============================================================================
# VISUALIZATION
# ============================================================================

# ============================================================================
# SPEC VERIFICATION
# ============================================================================

def verify_spec_compliance(history, cfg):
    """
    Verify simulation results against spec.md requirements.

    Args:
        history: Dictionary of numpy arrays
        cfg: Configuration object

    Returns:
        Dictionary of verification results
    """
    results = {
        "passed": True,
        "violations": [],
        "summary": {},
    }

    # Extract parameters from config
    soc_min = cfg.shield.soc_soft_min
    soc_max = cfg.shield.soc_soft_max
    fc_ramp_kw_per_s = cfg.fuel_cell.ramp_kw_per_s
    batt_capacity_kwh = cfg.battery.e_batt_kwh
    # Calculate C-rates from power limits
    batt_c_rate_discharge = cfg.battery.p_batt_max_discharge_kw / batt_capacity_kwh
    batt_c_rate_charge = cfg.battery.p_batt_max_charge_kw / batt_capacity_kwh

    dt = cfg.scenario.sim.dt_seconds

    soc = history["soc"]
    p_fc_kw = history["p_fc_kw"]
    p_batt_kw = history["p_batt_kw"]
    p_req_kw = history["p_req_kw"]
    p_aux_kw = history["p_aux_kw"]

    # Check 1: SOC corridor [0.20, 0.90]
    soc_violations = np.sum((soc < soc_min) | (soc > soc_max))
    if soc_violations > 0:
        results["passed"] = False
        results["violations"].append(
            f"SOC corridor violated {soc_violations} times "
            f"(min: {soc.min():.3f}, max: {soc.max():.3f})"
        )
    results["summary"]["soc_min"] = float(soc.min())
    results["summary"]["soc_max"] = float(soc.max())
    results["summary"]["soc_violations"] = int(soc_violations)

    # Check 2: FC ramp rate
    fc_ramp_rates = np.abs(np.diff(p_fc_kw)) / dt
    fc_ramp_max = fc_ramp_rates.max()
    fc_ramp_violations = np.sum(fc_ramp_rates > fc_ramp_kw_per_s * 1.01)  # 1% tolerance
    if fc_ramp_violations > 0:
        results["passed"] = False
        results["violations"].append(
            f"FC ramp rate exceeded {fc_ramp_violations} times "
            f"(max: {fc_ramp_max:.2f} kW/s, limit: {fc_ramp_kw_per_s:.2f} kW/s)"
        )
    results["summary"]["fc_ramp_max_kw_per_s"] = float(fc_ramp_max)
    results["summary"]["fc_ramp_violations"] = int(fc_ramp_violations)

    # Check 3: Battery C-rates
    p_batt_max_discharge = batt_c_rate_discharge * batt_capacity_kwh
    p_batt_max_charge = batt_c_rate_charge * batt_capacity_kwh

    discharge_violations = np.sum(p_batt_kw > p_batt_max_discharge * 1.01)
    charge_violations = np.sum(p_batt_kw < -p_batt_max_charge * 1.01)

    if discharge_violations > 0:
        results["passed"] = False
        results["violations"].append(
            f"Battery discharge C-rate exceeded {discharge_violations} times "
            f"(max: {p_batt_kw.max():.2f} kW, limit: {p_batt_max_discharge:.2f} kW)"
        )
    if charge_violations > 0:
        results["passed"] = False
        results["violations"].append(
            f"Battery charge C-rate exceeded {charge_violations} times "
            f"(min: {p_batt_kw.min():.2f} kW, limit: {-p_batt_max_charge:.2f} kW)"
        )

    results["summary"]["batt_power_max_kw"] = float(p_batt_kw.max())
    results["summary"]["batt_power_min_kw"] = float(p_batt_kw.min())
    results["summary"]["batt_discharge_violations"] = int(discharge_violations)
    results["summary"]["batt_charge_violations"] = int(charge_violations)

    # Check 4: Power balance (P_fc + P_batt_delivered + P_unmet = P_req + P_aux)
    # Note: p_batt_delivered accounts for battery discharge/charge efficiency
    # p_unmet represents demand that couldn't be met by supply
    p_batt_delivered_kw = history["p_batt_delivered_kw"]
    p_unmet_kw = history["p_unmet_kw"]
    p_supplied = p_fc_kw + p_batt_delivered_kw + p_unmet_kw
    p_demand = p_req_kw + p_aux_kw
    power_balance_error = np.abs(p_supplied - p_demand)
    power_balance_max_error = power_balance_error.max()
    power_balance_mean_error = power_balance_error.mean()

    # Allow tolerance for numerical errors and transients (10 kW)
    # Mean error should be small even if there are occasional spikes during transients
    tolerance_kw = 10.0
    power_balance_violations = np.sum(power_balance_error > tolerance_kw)

    if power_balance_violations > 0:
        results["violations"].append(
            f"Power balance error exceeded tolerance {power_balance_violations} times "
            f"(max error: {power_balance_max_error:.2f} kW, tolerance: {tolerance_kw:.2f} kW)"
        )

    results["summary"]["power_balance_max_error_kw"] = float(power_balance_max_error)
    results["summary"]["power_balance_mean_error_kw"] = float(power_balance_mean_error)
    results["summary"]["power_balance_violations"] = int(power_balance_violations)

    # Check 5: H2 consumption (integral of FC power)
    h2_consumed_kg = (soc[0] - soc[-1]) * batt_capacity_kwh  # Energy from battery
    fc_energy_kwh = np.trapezoid(p_fc_kw, dx=dt) / 3600.0
    h2_lhv_kwh_per_kg = 33.33  # Lower heating value of H2
    h2_fc_efficiency = 0.55  # Typical FC efficiency
    h2_expected_kg = fc_energy_kwh / (h2_lhv_kwh_per_kg * h2_fc_efficiency)

    results["summary"]["h2_expected_kg"] = float(h2_expected_kg)
    results["summary"]["fc_energy_kwh"] = float(fc_energy_kwh)

    # Energy metrics
    total_energy_delivered_kwh = np.trapezoid(p_req_kw, dx=dt) / 3600.0
    results["summary"]["total_energy_delivered_kwh"] = float(total_energy_delivered_kwh)

    return results


def print_verification_report(results):
    """Print formatted verification report."""
    print("\n" + "=" * 70)
    print("SPEC VERIFICATION REPORT")
    print("=" * 70)

    if results["passed"]:
        print("\n✓ ALL CHECKS PASSED")
    else:
        print("\n✗ VIOLATIONS DETECTED:")
        for violation in results["violations"]:
            print(f"  - {violation}")

    print("\n" + "-" * 70)
    print("SUMMARY STATISTICS:")
    print("-" * 70)
    for key, value in results["summary"].items():
        print(f"  {key:.<50s} {value:.4f}")

    print("=" * 70 + "\n")


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Test EMS with synthetic P_req curves")
    parser.add_argument(
        "--scenario",
        type=str,
        default="step_changes",
        choices=["steady_cruise", "step_changes", "ramp_profile", "regen_profile", "nil_like_profile", "emergency_braking", "high_soc_regen", "all"],
        help="Test scenario to run"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="conf.yaml",
        help="Path to config file"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=600,
        help="Simulation duration in seconds"
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip plotting"
    )

    args = parser.parse_args()

    # Generate synthetic curves
    print("Generating synthetic P_req curves...")
    scenarios = generate_synthetic_p_req_curves(dt=1.0, duration=args.duration)

    # Initialize test harness
    print(f"Initializing test harness with config: {args.config}")
    harness = EMSTestHarness(config_path=args.config)

    # Select scenarios to run
    if args.scenario == "all":
        scenario_names = list(scenarios.keys())
    else:
        scenario_names = [args.scenario]

    # Run scenarios
    for scenario_name in scenario_names:
        print(f"\n{'=' * 70}")
        print(f"Running scenario: {scenario_name}")
        print(f"{'=' * 70}")

        time_array, p_req_array = scenarios[scenario_name]

        # Set high SOC for scenarios demonstrating friction brakes
        if scenario_name == "emergency_braking":
            soc_init = 0.85  # High SOC to show friction brakes from power limit
            print("  Setting high initial SOC (85%) to demonstrate friction brake activation")
        elif scenario_name == "high_soc_regen":
            soc_init = 0.898  # Very high SOC near 90% limit to show regen rejection during descent
            print("  Setting very high initial SOC (89.8%) to demonstrate regen rejection during long descent")
        else:
            soc_init = None  # Use default from config

        # Run simulation
        harness.run_scenario(time_array, p_req_array, soc_init=soc_init)

        # Get results
        history = harness.get_history_arrays()

        # Verify spec compliance
        results = verify_spec_compliance(history, harness.cfg)
        print_verification_report(results)

        # Plot results
        if not args.no_plot:
            fig = plot_history(history, scenario_name)
            plt.savefig(f"test_ems_{scenario_name}.png", dpi=150, bbox_inches="tight")
            print(f"Plot saved: test_ems_{scenario_name}.png")

    if not args.no_plot:
        print("\nShowing plots...")
        plt.show()


if __name__ == "__main__":
    main()
