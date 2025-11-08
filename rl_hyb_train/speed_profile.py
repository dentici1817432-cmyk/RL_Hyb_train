"""Utilities to synthesize feasible speed profiles for the driver.

The generator respects power supply limits, driver accel/brake limits,
and produces continuous piecewise-constant targets with linear ramps.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Dict, Optional

from .config import Config


@dataclass
class Cruise:
    speed_mps: float
    duration_s: int
    label: str = "cruise"
    p_bias_kw: float = 0.0
    grade_percent: float = 0.0


def _feasible_accel_from_power(
    config: Config,
    p_bias_kw: float = 0.0,
    *,
    margin_kw: float = 10.0,
) -> float:
    """Return a conservative accel limit (m/s^2) based on power caps.

    a = k_gain * 1000 * (P_cap - losses - bias)
    """
    k_gain = max(config.plant.kinematic_gain_mps_per_watt, 1e-8)
    p_cap_kw = config.fuel_cell.p_fc_max_kw + config.battery.p_batt_max_discharge_kw
    p_losses_kw = config.driver.p_loss_watts / 1000.0
    net_kw = max(0.0, p_cap_kw - p_losses_kw - p_bias_kw - margin_kw)
    a_power = k_gain * 1000.0 * net_kw
    return max(0.0, min(a_power, config.driver.speed_tracking_accel_limit_mps2))


def _feasible_brake_from_power(
    config: Config,
    p_bias_kw: float = 0.0,
    *,
    margin_kw: float = 10.0,
) -> float:
    """Conservative braking limit based on regen + losses (m/s^2)."""
    k_gain = max(config.plant.kinematic_gain_mps_per_watt, 1e-8)
    p_regen_cap_kw = config.battery.p_batt_max_charge_kw
    p_losses_kw = config.driver.p_loss_watts / 1000.0
    net_kw = max(0.0, p_regen_cap_kw + p_losses_kw - abs(p_bias_kw) - margin_kw)
    a_power = k_gain * 1000.0 * net_kw
    return max(0.0, min(a_power, config.driver.speed_tracking_brake_limit_mps2))


def generate_feasible_profile(
    config: Config,
    *,
    dwells: List[Dict[str, float]],
    cruises: List[Cruise],
    start_speed_mps: float = 0.0,
    ramp_margin_kw: float = 10.0,
    min_ramp_time_s: int = 0,
    ramp_step_s: int = 5,
) -> List[Dict[str, float]]:
    """Build a feasible speed profile list for config.driver.manual_speed_profile.

    - Ramps use feasible accel/brake computed from power caps and driver limits
    - Target speeds are clipped to v_max
    - Output is a list of dicts matching the YAML schema
    """
    v_max = float(config.plant.v_max_mps)
    profile: List[Dict[str, float]] = []
    v_curr = float(max(0.0, min(start_speed_mps, v_max)))

    # Initial dwell if provided
    for dwell in dwells:
        dur = int(max(0, dwell.get("duration_s", 0)))
        if dur > 0:
            profile.append({"duration_s": dur, "speed_mps": 0.0, "label": dwell.get("label", "dwell"), "hold": True})

        # After a dwell we are stopped
        v_curr = 0.0

    for seg in cruises:
        v_target = float(max(0.0, min(seg.speed_mps, v_max)))
        p_bias = float(seg.p_bias_kw)

        if v_target > v_curr:
            a = _feasible_accel_from_power(config, p_bias_kw=p_bias, margin_kw=ramp_margin_kw)
            t_ramp = int(round((v_target - v_curr) / max(a, 1e-6)))
            if min_ramp_time_s > 0:
                t_ramp = max(t_ramp, int(min_ramp_time_s))
            if t_ramp > 0:
                n_step = max(1, int(max(1, t_ramp) // max(1, ramp_step_s)))
                dv = (v_target - v_curr) / n_step
                dur_each = max(1, int(round(t_ramp / n_step)))
                v_tmp = v_curr
                for _ in range(n_step):
                    v_tmp = min(v_target, v_tmp + dv)
                    profile.append({"duration_s": int(dur_each), "speed_mps": float(v_tmp), "label": "ramp_up"})
                elapsed = n_step * dur_each
                if elapsed < t_ramp:
                    profile.append({"duration_s": int(t_ramp - elapsed), "speed_mps": float(v_target), "label": "ramp_up"})
        elif v_target < v_curr:
            a_b = _feasible_brake_from_power(config, p_bias_kw=p_bias, margin_kw=ramp_margin_kw)
            t_ramp = int(round((v_curr - v_target) / max(a_b, 1e-6)))
            if min_ramp_time_s > 0:
                t_ramp = max(t_ramp, int(min_ramp_time_s))
            if t_ramp > 0:
                n_step = max(1, int(max(1, t_ramp) // max(1, ramp_step_s)))
                dv = (v_target - v_curr) / n_step
                dur_each = max(1, int(round(t_ramp / n_step)))
                v_tmp = v_curr
                for _ in range(n_step):
                    v_tmp = max(v_target, v_tmp + dv)
                    profile.append({"duration_s": int(dur_each), "speed_mps": float(v_tmp), "label": "ramp_down"})
                elapsed = n_step * dur_each
                if elapsed < t_ramp:
                    profile.append({"duration_s": int(t_ramp - elapsed), "speed_mps": float(v_target), "label": "ramp_down"})

        # Cruise
        if seg.duration_s > 0:
            entry = {
                "duration_s": int(seg.duration_s),
                "speed_mps": v_target,
                "label": seg.label,
            }
            if abs(seg.p_bias_kw) > 0:
                entry["p_bias_kw"] = float(seg.p_bias_kw)
            if abs(seg.grade_percent) > 0:
                entry["grade_percent"] = float(seg.grade_percent)
            profile.append(entry)
        v_curr = v_target

    return profile
