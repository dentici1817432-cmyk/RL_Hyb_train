"""Scenario sweeps for EMS policies on the RL Hybrid Train environment."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

from rl_hyb_train import Config
from rl_hyb_train.env0_env import Env0
from rl_hyb_train.policies import (
    BaselineEMS,
    BalancedEMS,
    BalancedEMSConfig,
    ScenarioAwareEMS,
    ScenarioEMSConfig,
    RLEMS,
    MPCEms,
    MPCConfig,
)
from rl_hyb_train.powerflow import compute_power_balance
from rl_hyb_train.speed_profile import generate_feasible_profile, Cruise


# ---------------------------------------------------------------------------
# Scenario builders
# ---------------------------------------------------------------------------

def create_full_regime_profile() -> List[Dict[str, float]]:
    """Single timeline that exercises every power-flow regime."""
    return [
        {"duration_s": 120, "p_req_kw": 0.0},     # station dwell / grid charge
        {"duration_s": 320, "p_req_kw": 110.0},   # moderate climb (FC-only traction)
        {"duration_s": 180, "p_req_kw": 150.0},   # heavier pull, battery assist
        {"duration_s": 200, "p_req_kw": 70.0},    # cruise, FC surplus -> charge batt
        {"duration_s": 260, "p_req_kw": 210.0},   # high-power sprint (FC+Batt discharge)
        {"duration_s": 160, "p_req_kw": 140.0},   # plateau
        {"duration_s": 220, "p_req_kw": 70.0},    # pre-regenerative descent
        {"duration_s": 200, "p_req_kw": -90.0},   # gentle regen (Batt charge from regen)
        {"duration_s": 120, "p_req_kw": -130.0},  # deeper regen
        {"duration_s": 60, "p_req_kw": -170.0},   # strong braking → regen clipping
        {"duration_s": 120, "p_req_kw": 0.0},     # mid-route dwell / idle
        {"duration_s": 260, "p_req_kw": 100.0},   # accelerate out of station
        {"duration_s": 200, "p_req_kw": 200.0},   # second sprint
        {"duration_s": 120, "p_req_kw": 210.0},   # sustained high demand
        {"duration_s": 200, "p_req_kw": -80.0},   # downhill regen
        {"duration_s": 150, "p_req_kw": -120.0},  # final braking
        {"duration_s": 150, "p_req_kw": 0.0},     # terminal dwell
    ]


def create_regen_test_profile() -> List[Dict[str, float]]:
    """Legacy long-form regen script."""
    return [
        {"duration_s": 100, "p_req_kw": 0.0},
        {"duration_s": 150, "p_req_kw": 160.0},
        {"duration_s": 200, "p_req_kw": 200.0},
        {"duration_s": 220, "p_req_kw": 120.0},
        {"duration_s": 120, "p_req_kw": 60.0},
        {"duration_s": 150, "p_req_kw": -60.0},
        {"duration_s": 150, "p_req_kw": -110.0},
        {"duration_s": 150, "p_req_kw": 0.0},
        {"duration_s": 200, "p_req_kw": 180.0},
        {"duration_s": 140, "p_req_kw": 90.0},
        {"duration_s": 80, "p_req_kw": 0.0},
        {"duration_s": 220, "p_req_kw": -150.0},
        {"duration_s": 180, "p_req_kw": -190.0},
        {"duration_s": 120, "p_req_kw": -90.0},
        {"duration_s": 200, "p_req_kw": 0.0},
        {"duration_s": 200, "p_req_kw": 110.0},
        {"duration_s": 280, "p_req_kw": 80.0},
        {"duration_s": 140, "p_req_kw": 40.0},
        {"duration_s": 150, "p_req_kw": -80.0},
        {"duration_s": 150, "p_req_kw": -120.0},
        {"duration_s": 80, "p_req_kw": -60.0},
        {"duration_s": 200, "p_req_kw": 0.0},
    ]


def create_stop_go_profile(repeats: int = 4) -> List[Dict[str, float]]:
    profile: List[Dict[str, float]] = []
    for _ in range(repeats):
        profile.extend(
            [
                {"duration_s": 90, "p_req_kw": 0.0},
                {"duration_s": 150, "p_req_kw": 160.0},
                {"duration_s": 100, "p_req_kw": 220.0},
                {"duration_s": 80, "p_req_kw": 140.0},
                {"duration_s": 70, "p_req_kw": 40.0},
                {"duration_s": 120, "p_req_kw": -90.0},
                {"duration_s": 60, "p_req_kw": -140.0},
            ]
        )
    profile.append({"duration_s": 180, "p_req_kw": 0.0})
    return profile


def create_flat_cruise_profile() -> List[Dict[str, float]]:
    return [
        {"duration_s": 150, "p_req_kw": 0.0},
        {"duration_s": 450, "p_req_kw": 90.0},
        {"duration_s": 320, "p_req_kw": 150.0},
        {"duration_s": 260, "p_req_kw": 110.0},
        {"duration_s": 200, "p_req_kw": 70.0},
        {"duration_s": 150, "p_req_kw": -30.0},
        {"duration_s": 160, "p_req_kw": -60.0},
        {"duration_s": 280, "p_req_kw": 150.0},
        {"duration_s": 320, "p_req_kw": 100.0},
        {"duration_s": 200, "p_req_kw": 0.0},
    ]


def create_pulse_stress_profile() -> List[Dict[str, float]]:
    profile: List[Dict[str, float]] = [{"duration_s": 120, "p_req_kw": 0.0}]
    for _ in range(8):
        profile.extend(
            [
                {"duration_s": 30, "p_req_kw": 200.0},
                {"duration_s": 30, "p_req_kw": -120.0},
                {"duration_s": 20, "p_req_kw": 160.0},
                {"duration_s": 20, "p_req_kw": -80.0},
            ]
        )
    profile.extend(
        [
            {"duration_s": 200, "p_req_kw": 180.0},
            {"duration_s": 200, "p_req_kw": -150.0},
            {"duration_s": 200, "p_req_kw": 0.0},
        ]
    )
    return profile


def create_regen_showcase_profile() -> List[Dict[str, float]]:
    """Long downhill section to force sustained regeneration."""
    return [
        {"duration_s": 180, "p_req_kw": 0.0},
        {"duration_s": 240, "p_req_kw": 160.0},
        {"duration_s": 240, "p_req_kw": 200.0},
        {"duration_s": 120, "p_req_kw": 80.0},
        {"duration_s": 180, "p_req_kw": -60.0},
        {"duration_s": 240, "p_req_kw": -110.0},
        {"duration_s": 180, "p_req_kw": -160.0},
        {"duration_s": 120, "p_req_kw": -200.0},
        {"duration_s": 300, "p_req_kw": -140.0},
        {"duration_s": 120, "p_req_kw": -60.0},
        {"duration_s": 180, "p_req_kw": 40.0},
        {"duration_s": 120, "p_req_kw": 0.0},
    ]


SCENARIOS: Dict[str, Callable[[], List[Dict[str, float]]]] = {
    "full_regime": create_full_regime_profile,
    "regen_stress": create_regen_test_profile,
    "stop_and_go": create_stop_go_profile,
    "flat_cruise": create_flat_cruise_profile,
    "pulse_stress": create_pulse_stress_profile,
    "regen_showcase": create_regen_showcase_profile,
}


# ---------------------------------------------------------------------------
# Regime tracking
# ---------------------------------------------------------------------------


@dataclass
class RegimeDefinition:
    name: str
    category: str
    description: str
    predicate: Callable[[Dict[str, float], Sequence[float]], bool]
    min_duration: int = 5


class RegimeTracker:
    """Track contiguous windows that satisfy regime predicates."""

    def __init__(self, definitions: List[RegimeDefinition]):
        self.definitions = definitions
        self.active: Dict[str, Dict[str, Any] | None] = {d.name: None for d in definitions}
        self.events: List[Dict[str, Any]] = []

    def update(self, step: int, info: Dict[str, float], action: Sequence[float]) -> None:
        p_req = float(info.get("p_req_kw", 0.0))
        fc_frac = float(action[0]) if len(action) > 0 else 0.0
        batt_cmd = float(action[1]) if len(action) > 1 else 0.0
        soc = float(info.get("soc", 0.0))

        for definition in self.definitions:
            is_active = definition.predicate(info, action)
            slot = self.active[definition.name]
            if is_active:
                if slot is None:
                    slot = {
                        "definition": definition,
                        "name": definition.name,
                        "category": definition.category,
                        "description": definition.description,
                        "start": step,
                        "end": step,
                        "min_p_req": p_req,
                        "max_p_req": p_req,
                        "min_fc_frac": fc_frac,
                        "max_fc_frac": fc_frac,
                        "min_batt_cmd": batt_cmd,
                        "max_batt_cmd": batt_cmd,
                        "start_soc": soc,
                        "end_soc": soc,
                    }
                    self.active[definition.name] = slot
                else:
                    slot["end"] = step
                    slot["min_p_req"] = min(slot["min_p_req"], p_req)
                    slot["max_p_req"] = max(slot["max_p_req"], p_req)
                    slot["min_fc_frac"] = min(slot["min_fc_frac"], fc_frac)
                    slot["max_fc_frac"] = max(slot["max_fc_frac"], fc_frac)
                    slot["min_batt_cmd"] = min(slot["min_batt_cmd"], batt_cmd)
                    slot["max_batt_cmd"] = max(slot["max_batt_cmd"], batt_cmd)
                    slot["end_soc"] = soc
            elif slot is not None:
                self._finalize_event(definition, slot)
                self.active[definition.name] = None

    def finalize(self) -> None:
        for definition in self.definitions:
            slot = self.active.get(definition.name)
            if slot is not None:
                self._finalize_event(definition, slot)
                self.active[definition.name] = None

    def _finalize_event(self, definition: RegimeDefinition, event: Dict[str, Any]) -> None:
        duration = event["end"] - event["start"] + 1
        if duration >= definition.min_duration:
            event["duration"] = duration
            self.events.append(event)


def build_regime_tracker(
    config: Config,
    profile: Optional[List[Dict[str, float]]],
    power_stats: Optional[Dict[str, float]] = None,
) -> RegimeTracker:
    pos_peak = max((seg["p_req_kw"] for seg in profile if seg["p_req_kw"] > 0.0), default=0.0) if profile else 0.0
    neg_peak = min((seg["p_req_kw"] for seg in profile if seg["p_req_kw"] < 0.0), default=0.0) if profile else 0.0

    if power_stats:
        pos_peak = max(pos_peak, power_stats.get("max_pull_kw", 0.0))
        neg_peak = min(neg_peak, power_stats.get("max_regen_kw", 0.0))

    heavy_threshold = 0.75 * pos_peak if pos_peak > 0.0 else 0.25 * (
        config.fuel_cell.p_fc_max_kw + 0.2 * config.battery.p_batt_max_discharge_kw
    )
    regen_threshold = 0.75 * neg_peak if neg_peak < 0.0 else -0.25 * (
        config.battery.p_batt_max_charge_kw + 0.25 * config.fuel_cell.p_fc_max_kw
    )
    idle_threshold = max(5.0, 0.05 * max(pos_peak, config.fuel_cell.p_fc_max_kw))

    definitions = [
        RegimeDefinition(
            name="Heavy traction request",
            category="P_req regimes",
            description=f"P_req ≥ {heavy_threshold:.0f} kW (≈75% of scenario peak)",
            predicate=lambda info, action, thr=heavy_threshold: float(info.get("p_req_kw", 0.0)) >= thr,
            min_duration=8,
        ),
        RegimeDefinition(
            name="Deep regenerative braking",
            category="P_req regimes",
            description=f"P_req ≤ {regen_threshold:.0f} kW (strong regen segment)",
            predicate=lambda info, action, thr=regen_threshold: float(info.get("p_req_kw", 0.0)) <= thr,
            min_duration=5,
        ),
        RegimeDefinition(
            name="Station dwell / idle",
            category="P_req regimes",
            description="Demand near zero while train speed is low",
            predicate=lambda info, action, thr=idle_threshold: abs(float(info.get("p_req_kw", 0.0))) <= thr
            and float(info.get("speed_mps", 0.0)) < 0.5,
            min_duration=10,
        ),
        RegimeDefinition(
            name="FC saturated",
            category="EMS actions",
            description="EMS requests maximum FC output",
            predicate=lambda info, action: len(action) > 0 and float(action[0]) >= 0.95,
            min_duration=4,
        ),
        RegimeDefinition(
            name="Battery boost",
            category="EMS actions",
            description="Battery discharge command ≥90% of limit",
            predicate=lambda info, action: len(action) > 1 and float(action[1]) >= 0.9,
            min_duration=4,
        ),
        RegimeDefinition(
            name="Battery soak / regen",
            category="EMS actions",
            description="Battery charge command ≤-90% of limit",
            predicate=lambda info, action: len(action) > 1 and float(action[1]) <= -0.9,
            min_duration=4,
        ),
        RegimeDefinition(
            name="Idle trickle charge",
            category="EMS actions",
            description="Low demand but EMS charges battery to build SOC headroom",
            predicate=lambda info, action, thr=idle_threshold: abs(float(info.get("p_req_kw", 0.0))) <= thr
            and len(action) > 1
            and float(action[1]) < -0.2,
            min_duration=6,
        ),
    ]
    return RegimeTracker(definitions)


def report_regime_summary(regime_tracker: RegimeTracker) -> None:
    print("\nRegime highlights:")
    if not regime_tracker.events:
        print("  (no interesting regimes detected)")
        return

    events_by_category: Dict[str, List[Dict[str, Any]]] = {}
    for event in sorted(regime_tracker.events, key=lambda evt: evt["start"]):
        events_by_category.setdefault(event["category"], []).append(event)

    category_order = ["P_req regimes", "EMS actions"]
    for category in category_order:
        events = events_by_category.get(category, [])
        if not events:
            continue
        print(f"\n  {category}:")
        for event in events:
            print(
                f"    - {event['name']}: steps {event['start']}–{event['end']} ({event['duration']} s); "
                f"P_req {event['min_p_req']:.1f}→{event['max_p_req']:.1f} kW; "
                f"fc_frac {event['min_fc_frac']:.2f}–{event['max_fc_frac']:.2f}; "
                f"batt_cmd {event['min_batt_cmd']:.2f}–{event['max_batt_cmd']:.2f}; "
                f"SOC {event['start_soc']:.3f}→{event['end_soc']:.3f}"
            )
            description = event.get("description")
            if description:
                print(f"        {description}")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def summarize_profile(name: str, profile: List[Dict[str, float]]) -> None:
    total_duration = sum(seg["duration_s"] for seg in profile)
    regen_segments = [seg for seg in profile if seg["p_req_kw"] < 0]
    max_pull = max(seg["p_req_kw"] for seg in profile)
    min_regen = min((seg["p_req_kw"] for seg in regen_segments), default=0.0)

    print(f"\nScenario: {name}")
    print("-" * 60)
    print(f"  Total duration: {total_duration} s ({total_duration / 60:.1f} min)")
    print(f"  Segments: {len(profile)}")
    print(f"  Regen segments: {len(regen_segments)}")
    print(f"  Max traction request: {max_pull:.1f} kW")
    if regen_segments:
        print(f"  Max regen power: {abs(min_regen):.1f} kW")
    print()


def summarize_speed_profile(name: str, profile: List[Dict[str, float]], dwell_threshold_mps: float = 0.5) -> None:
    if not profile:
        print(f"\nScenario: {name} (config speed profile)")
        print("-" * 60)
        print("  No manual speed profile is defined in config.")
        return

    total_duration = sum(seg.get("duration_s", 0.0) for seg in profile)
    dwell_segments = [seg for seg in profile if seg.get("speed_mps", 0.0) <= dwell_threshold_mps]
    max_speed = max(seg.get("speed_mps", 0.0) for seg in profile)
    min_speed = min(seg.get("speed_mps", 0.0) for seg in profile)

    print(f"\nScenario: {name} (config speed profile)")
    print("-" * 60)
    print(f"  Total duration: {total_duration:.0f} s ({total_duration / 60:.1f} min)")
    print(f"  Segments: {len(profile)}")
    print(f"  Stops/dwells: {len(dwell_segments)}")
    print(f"  Speed range: {min_speed:.1f}–{max_speed:.1f} m/s")


def _expand_preq_profile_vector(profile: List[Dict[str, float]], dt: float) -> List[float]:
    vector: List[float] = []
    if not profile:
        return vector
    for seg in profile:
        duration = max(1, int(round(float(seg.get("duration_s", 0.0)) / dt)))
        p_req = float(seg.get("p_req_kw", 0.0))
        vector.extend([p_req] * duration)
    return vector


def estimate_speed_profile_power_bounds(config: Config) -> Dict[str, float]:
    k_gain = max(config.plant.kinematic_gain_mps_per_watt, 1e-8)
    base_loss_kw = config.driver.p_loss_watts / 1000.0
    accel_limit = config.driver.speed_tracking_accel_limit_mps2
    brake_limit = config.driver.speed_tracking_brake_limit_mps2
    accel_power = accel_limit / (k_gain * 1000.0)
    brake_power = brake_limit / (k_gain * 1000.0)
    bias_values = [float(seg.get("p_bias_kw", 0.0)) for seg in config.driver.manual_speed_profile or []]
    bias_max = max(bias_values) if bias_values else 0.0
    bias_min = min(bias_values) if bias_values else 0.0
    max_pull = base_loss_kw + accel_power + bias_max
    max_regen = base_loss_kw - brake_power + bias_min
    return {"max_pull_kw": max_pull, "max_regen_kw": max_regen}


def build_ems(
    policy: str,
    config: Config,
    profile: List[Dict[str, float]],
    rl_model: Optional[Path],
) -> Any:
    policy_cfg = config.policy

    if policy == "baseline":
        return BaselineEMS(config)

    if policy == "balanced":
        tuning = BalancedEMSConfig(**policy_cfg.balanced) if policy_cfg.balanced else None
        return BalancedEMS(config, tuning=tuning)

    if policy == "scenario":
        scenario_tuning = (
            ScenarioEMSConfig(**policy_cfg.scenario) if policy_cfg.scenario else None
        )
        return ScenarioAwareEMS(config, profile, tuning=scenario_tuning)

    if policy == "rl":
        if rl_model is None:
            raise ValueError("--rl-model must be provided when policy=rl")
        return RLEMS(rl_model)

    if policy == "mpc":
        mpc_tuning = MPCConfig(**policy_cfg.mpc) if policy_cfg.mpc else None
        return MPCEms(config, tuning=mpc_tuning)

    raise ValueError(f"Unknown policy {policy}")


def run_policy_episode(
    policy: str,
    config_path: Path,
    profile_name: str,
    profile: List[Dict[str, float]],
    driver_mode: str,
    *,
    stats_interval: int,
    render_interval: int,
    live_render: bool,
    final_render: bool,
    seed: int = 42,
    rl_model: Optional[Path] = None,
    auto_speed: bool = False,
) -> Dict[str, float]:
    config = Config.from_yaml(config_path)
    power_stats: Optional[Dict[str, float]] = None
    resolved_policy = config.policy.default if policy == "auto" else policy

    if driver_mode == "scenario_profile":
        config.driver.manual_p_req_profile = profile
        config.driver.manual_speed_profile = []
        config.driver.manual_loop = False
        summarize_profile(profile_name, profile)
    elif driver_mode == "config_speed":
        # Optionally synthesize a feasible speed profile
        if auto_speed:
            auto_profile = generate_feasible_profile(
                config,
                dwells=[{"duration_s": 120, "label": "Station A dwell"}],
                cruises=[
                    Cruise(speed_mps=12.0, duration_s=900, label="Leg A1"),
                    Cruise(speed_mps=18.0, duration_s=900, label="Leg A2"),
                    Cruise(speed_mps=22.0, duration_s=1200, label="Suburban"),
                    Cruise(speed_mps=16.0, duration_s=720, label="Approach"),
                    Cruise(speed_mps=0.0, duration_s=180, label="Station B dwell"),
                    Cruise(speed_mps=12.0, duration_s=720, label="Leg B1"),
                    Cruise(speed_mps=18.0, duration_s=900, label="Leg B2"),
                    Cruise(speed_mps=24.0, duration_s=1200, label="Express"),
                    Cruise(speed_mps=0.0, duration_s=180, label="Terminal dwell"),
                ],
                start_speed_mps=0.0,
                ramp_margin_kw=60.0,
                min_ramp_time_s=90,
            )
            config.driver.manual_speed_profile = auto_profile
            config.driver.manual_p_req_profile = []
            config.driver.manual_loop = False
        summarize_speed_profile(profile_name, config.driver.manual_speed_profile)
        power_stats = estimate_speed_profile_power_bounds(config)
        config.driver.manual_loop = False
    else:
        raise ValueError(f"Unknown driver mode: {driver_mode}")

    # Always capture buffers so we can emit a final frame even if live rendering is off.
    config.renderer.enabled = True

    env = Env0(config, seed=seed)
    ems = build_ems(resolved_policy, config, profile, rl_model=rl_model)
    if hasattr(ems, "reset"):
        ems.reset()

    print(f"Loaded config from {config_path}")
    print(f"Policy: {resolved_policy}")
    print(f"FC max power: {config.fuel_cell.p_fc_max_kw} kW")
    print(f"Battery max discharge: {config.battery.p_batt_max_discharge_kw} kW")
    print(f"Battery max charge: {config.battery.p_batt_max_charge_kw} kW\n")

    obs, info = env.reset()
    print("Initial state:")
    print(f"  SOC: {info['soc']:.3f}")
    print(f"  Tank: {info['tank_level']:.3f}")
    print(f"  Speed: {info['speed_mps']:.1f} m/s\n")

    totals = {
        "reward": 0.0,
        "cost_h2": 0.0,
        "cost_grid": 0.0,
        "unmet_sum": 0.0,
        "max_unmet": 0.0,
        "regen_steps": 0,
        "regen_power": 0.0,
        "max_regen": 0.0,
        "regen_charge_kwh": 0.0,
        "fc_charge_kwh": 0.0,
        "max_balance_error_kw": 0.0,
    }
    step_count = 0

    stats_interval = max(1, stats_interval)
    render_interval = max(1, render_interval)

    regime_tracker = build_regime_tracker(
        config,
        profile if driver_mode == "scenario_profile" else None,
        power_stats=power_stats,
    )

    try:
        for step in range(8000):
            # Prefer RL policy that consumes observations, fallback to info-based baseline
            if hasattr(ems, "act"):
                action = ems.act(obs, info)
            else:
                # If using MPC with a scripted P_req scenario, feed a forecast of future P_req
                if isinstance(ems, MPCEms) and driver_mode == "scenario_profile":
                    horizon = ems.tuning.horizon_s if hasattr(ems, "tuning") else 30
                    vec = _expand_preq_profile_vector(profile, config.sim.dt_seconds)
                    step_idx = int(info.get("step", 0))
                    forecast_slice = vec[step_idx: step_idx + horizon]
                    if forecast_slice:
                        ems.set_forecast(forecast_slice)

                action = ems.act_from_info(info)
            action_tuple = (float(action[0]), float(action[1])) if len(action) >= 2 else (float(action[0]), 0.0)
            obs, reward, terminated, truncated, info = env.step(action)

            step_count += 1
            totals["reward"] += reward
            totals["cost_h2"] += info.get("cost_h2_eur", 0.0)
            totals["cost_grid"] += info.get("cost_grid_eur", 0.0)

            unmet = info.get("p_unmet_kw", 0.0)
            totals["unmet_sum"] += unmet
            totals["max_unmet"] = max(totals["max_unmet"], unmet)

            p_req = info.get("p_req_kw", 0.0)
            if "p_aux_kw" in info:
                balance = compute_power_balance(
                    p_req_kw=p_req,
                    p_aux_kw=info["p_aux_kw"],
                    p_fc_kw=info.get("p_fc_kw", 0.0),
                    p_batt_delivered_kw=info.get("p_batt_delivered_kw", max(info.get("p_batt_kw", 0.0), 0.0)),
                )
                diff = abs(balance.p_unmet_kw - info.get("p_unmet_kw", 0.0))
                totals["max_balance_error_kw"] = max(totals["max_balance_error_kw"], diff)

            if p_req < 0.0:
                totals["regen_steps"] += 1
                totals["regen_power"] += abs(p_req)
                totals["max_regen"] = max(totals["max_regen"], abs(p_req))

            p_regen_chg = info.get("p_batt_charge_regen_kw", 0.0)
            p_fc_chg = info.get("p_batt_charge_fc_kw", 0.0)
            if p_regen_chg > 0.0:
                totals["regen_charge_kwh"] += p_regen_chg * config.sim.dt_seconds / 3600.0
            if p_fc_chg > 0.0:
                totals["fc_charge_kwh"] += p_fc_chg * config.sim.dt_seconds / 3600.0

            regime_tracker.update(step, info, action_tuple)

            if step % stats_interval == 0:
                regen_indicator = " [REGEN]" if p_req < 0 else ""
                p_brake_total = info.get("p_brake_total_kw", 0.0)
                p_brake_friction = info.get("p_brake_friction_kw", 0.0)
                print(
                    f"Step {step:4d}: "
                    f"SOC={info['soc']:.3f} "
                    f"Tank={info['tank_level']:.3f} "
                    f"P_req={p_req:7.1f} kW{regen_indicator} "
                    f"P_FC={info['p_fc_kw']:6.1f} kW "
                    f"P_batt={info['p_batt_kw']:7.1f} kW "
                    f"P_unmet={unmet:6.2f} kW "
                    f"P_brk={p_brake_total:6.1f} kW "
                    f"P_brk_mech={p_brake_friction:6.1f} kW"
                )

            if live_render and step % render_interval == 0:
                env.render(mode="human")

            if terminated or truncated:
                print(f"\nEpisode ended at step {step}: terminated={terminated}, truncated={truncated}")
                break
    finally:
        if final_render and env._renderer is not None:
            env._render_step_counter = env.config.renderer.render_every
            env.render(mode="human")
        env.close()

    regime_tracker.finalize()

    avg_unmet = totals["unmet_sum"] / max(1, step_count)
    distance = info.get("distance_km", 0.0)

    print("\n" + "=" * 60)
    print(f"Episode Summary - {profile_name} ({policy})")
    print("=" * 60)
    print(f"Steps: {step_count}")
    print(f"Distance: {distance:.3f} km")
    print(f"Final SOC: {info['soc']:.3f}")
    print(f"Final tank level: {info['tank_level']:.3f}")
    print(f"Total reward: {totals['reward']:.4f}")
    print(f"Total H2 cost: {totals['cost_h2']:.4f} €")
    print(f"Total grid cost: {totals['cost_grid']:.4f} €")
    print(f"Average unmet demand: {avg_unmet:.2f} kW")
    print(f"Max unmet demand: {totals['max_unmet']:.2f} kW")
    print(f"Constraint violations: {info.get('constraint_violations', 0)}")
    print("\nRegen & Charging Statistics:")
    print(f"Regen steps: {totals['regen_steps']} ({100.0 * totals['regen_steps'] / max(1, step_count):.1f}% of episode)")
    if totals["regen_steps"] > 0:
        print(f"Average regen power: {totals['regen_power'] / totals['regen_steps']:.2f} kW")
        print(f"Max regen power: {totals['max_regen']:.2f} kW")
    print(f"  Battery charge from regen: {totals['regen_charge_kwh']:.2f} kWh")
    print(f"  Battery charge from FC: {totals['fc_charge_kwh']:.2f} kWh")
    print("\nDiagnostics:")
    print(f"Max DC-bus mismatch: {totals['max_balance_error_kw']:.3e} kW")

    if distance > 0:
        cost_per_km = (totals["cost_h2"] + totals["cost_grid"]) / distance
        print("\nPerformance Metrics:")
        print(f"Cost per km: {cost_per_km:.4f} €/km")
        print(f"H2 cost per km: {totals['cost_h2'] / distance:.4f} €/km")
        print(f"Grid cost per km: {totals['cost_grid'] / distance:.4f} €/km")

    report_regime_summary(regime_tracker)

    print("\nScenario run completed.\n")

    return {
        "steps": step_count,
        "distance_km": distance,
        "total_reward": totals["reward"],
        "total_cost_h2": totals["cost_h2"],
        "total_cost_grid": totals["cost_grid"],
        "max_unmet_kw": totals["max_unmet"],
        "avg_unmet_kw": avg_unmet,
        "constraint_violations": info.get("constraint_violations", 0),
        "final_soc": info["soc"],
        "final_tank": info["tank_level"],
        "policy_used": resolved_policy,
    }


def test_baseline_ems() -> Dict[str, Dict[str, Dict[str, float]]]:
    parser = argparse.ArgumentParser(description="EMS scenario sweep")
    parser.add_argument(
        "--scenarios",
        nargs="*",
        choices=SCENARIOS.keys(),
        default=["full_regime"],
        help="Scenario names to run (default: full_regime only)",
    )
    parser.add_argument(
        "--policy",
        choices=["auto", "baseline", "balanced", "scenario", "mpc", "rl", "both"],
        default="scenario",
        help="Which EMS policy to run ('auto' uses config.policy.default)",
    )
    parser.add_argument("--rl-model", type=Path, default=None, help="Path to a trained SB3 model (when policy=rl)")
    parser.add_argument("--render", action="store_true", help="Enable live rendering (slower)")
    parser.add_argument("--render-interval", type=int, default=10, help="Render every N steps when live rendering")
    parser.add_argument("--stats-interval", type=int, default=50, help="Print stats every N steps")
    parser.add_argument("--skip-final-render", action="store_true", help="Skip saving the final frame")
    parser.add_argument("--seed", type=int, default=42, help="Environment seed")
    parser.add_argument("--auto-speed", action="store_true", help="Generate a feasible speed profile and use it (driver_mode=config_speed)")
    parser.add_argument(
        "--driver-mode",
        choices=["scenario_profile", "config_speed"],
        default="scenario_profile",
        help="Use scripted P_req profile (default) or keep config-defined speed-driven driver",
    )
    args = parser.parse_args()

    selected_scenarios = args.scenarios or ["full_regime"]
    policies = ["baseline", "scenario"] if args.policy == "both" else [args.policy]

    config_path = Path(__file__).parent / "conf.yaml"

    aggregated: Dict[str, Dict[str, Dict[str, float]]] = {}
    for scenario_name in selected_scenarios:
        profile = SCENARIOS[scenario_name]()
        aggregated[scenario_name] = {}
        for policy in policies:
            aggregated[scenario_name][policy] = run_policy_episode(
                policy,
                config_path,
                scenario_name,
                profile,
                args.driver_mode,
                stats_interval=args.stats_interval,
                render_interval=args.render_interval,
                live_render=args.render,
                final_render=not args.skip_final_render,
                seed=args.seed,
                rl_model=args.rl_model,
                auto_speed=args.auto_speed,
            )

    print("\n" + "#" * 86)
    print("Scenario Comparison Summary")
    print("#" * 86)
    header = f"{'Scenario':<14} {'Policy':<10} {'Steps':>6} {'Dist(km)':>9} {'€/km':>8} {'Max Unmet':>10} {'Violations':>12}"
    print(header)
    print("-" * len(header))
    for scenario_name in selected_scenarios:
        for policy in policies:
            metrics = aggregated[scenario_name][policy]
            policy_label = metrics.get("policy_used", policy)
            distance = metrics["distance_km"]
            if distance > 0:
                cost_per_km = (metrics["total_cost_h2"] + metrics["total_cost_grid"]) / distance
            else:
                cost_per_km = 0.0
            print(
                f"{scenario_name:<14} "
                f"{policy_label:<10} "
                f"{metrics['steps']:6d} "
                f"{distance:9.2f} "
                f"{cost_per_km:8.3f} "
                f"{metrics['max_unmet_kw']:10.2f} "
                f"{int(metrics['constraint_violations']):12d}"
            )

    print("\nMulti-scenario sweep complete.")
    return aggregated


if __name__ == "__main__":
    test_baseline_ems()
