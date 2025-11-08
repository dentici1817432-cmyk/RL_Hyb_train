"""Scenario-aware EMS that exploits foreknowledge of the manual P_req profile."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from ..config import Config


@dataclass
class ScenarioEMSConfig:
    """Tunable heuristics for the scenario-aware EMS."""

    heavy_threshold_kw: float = 170.0
    regen_threshold_kw: float = -70.0
    lookahead_high_s: int = 60
    lookahead_regen_s: int = 45
    idle_grid_charge_kw: float = 40.0
    target_soc_low: float = 0.55
    target_soc_mid: float = 0.62
    target_soc_high: float = 0.70


class ScenarioAwareEMS:
    """
    Rule-based EMS that knows the full manual P_req timeline and
    schedules FC/Battery use to exercise every power-flow regime.
    """

    def __init__(
        self,
        config: Config,
        profile: Optional[List[Dict[str, float]]] = None,
        tuning: ScenarioEMSConfig | None = None,
    ):
        self.config = config
        self.dt = config.sim.dt_seconds
        self.p_fc_max = config.fuel_cell.p_fc_max_kw
        self.fc_ramp = config.fuel_cell.ramp_kw_per_s
        self.p_batt_dis_max = config.battery.p_batt_max_discharge_kw
        self.p_batt_chg_max = config.battery.p_batt_max_charge_kw
        self.eta_discharge = config.battery.eta_discharge
        self.eta_charge = config.battery.eta_charge
        self.p_aux_base = config.plant.p_aux_base_watts / 1000.0

        self.tuning = tuning or ScenarioEMSConfig()
        if tuning is None:
            # Adapt SOC targets to the configured corridor so discharge isn't blocked near mid-SOC.
            corridor_min = float(config.shield.soc_soft_min)
            corridor_max = float(config.shield.soc_soft_max)
            margin = max((corridor_max - corridor_min) * 0.1, 0.02)
            target_low = min(max(corridor_min + margin, 0.05), 0.9)
            target_high = max(min(corridor_max - margin, 0.95), target_low + 0.05)
            target_mid = (target_low + target_high) * 0.5
            self.tuning.target_soc_low = target_low
            self.tuning.target_soc_mid = target_mid
            self.tuning.target_soc_high = target_high
        self.fc_ref_kw = 0.0

        self.profile_vector = self._expand_profile(profile) if profile else np.zeros(1, dtype=float)
        self.profile_len = len(self.profile_vector)

    # ------------------------------------------------------------------
    def reset(self):
        self.fc_ref_kw = 0.0

    def act(self, obs: Optional[np.ndarray] = None, info: Optional[Dict[str, float]] = None) -> np.ndarray:
        if info is None:
            raise ValueError("ScenarioAwareEMS requires env info to act")
        return self._act(info)

    # ------------------------------------------------------------------
    def act_from_info(self, info: Dict[str, float]) -> np.ndarray:
        return self._act(info)

    # ------------------------------------------------------------------
    def _expand_profile(self, profile: List[Dict[str, float]]) -> np.ndarray:
        vector: List[float] = []
        if not profile:
            return np.zeros(1, dtype=float)

        for segment in profile:
            duration = max(1, int(round(segment.get("duration_s", 0.0) / self.dt)))
            p_req = float(segment.get("p_req_kw", 0.0))
            vector.extend([p_req] * duration)
        return np.asarray(vector, dtype=float)

    # ------------------------------------------------------------------
    def _future_window_stats(self, step: int, window: int) -> tuple[float, float]:
        if self.profile_len == 0:
            return 0.0, 0.0
        start = min(max(step, 0), self.profile_len - 1)
        end = min(self.profile_len, start + window)
        if start >= end:
            return 0.0, 0.0
        window_slice = self.profile_vector[start:end]
        return float(np.max(window_slice)), float(np.min(window_slice))

    # ------------------------------------------------------------------
    def _act(self, info: Dict[str, float]) -> np.ndarray:
        p_req = float(info.get("p_req_kw", 0.0))
        soc = float(info.get("soc", 0.65))
        speed = float(info.get("speed_mps", 0.0))
        at_stop = bool(info.get("at_stop", False))
        step = int(info.get("step", 0))
        p_aux = float(info.get("p_aux_kw", self.p_aux_base))

        future_max, future_min = self._future_window_stats(step, self.tuning.lookahead_high_s)
        _, future_regen_min = self._future_window_stats(step, self.tuning.lookahead_regen_s)

        # Base FC target tracks demand but uses foresight for upcoming heavy pulls / regen.
        p_dem = p_req + p_aux
        target_fc = np.clip(p_dem, 0.0, self.p_fc_max)

        heavy_ahead = future_max >= self.tuning.heavy_threshold_kw
        regen_ahead = future_min <= self.tuning.regen_threshold_kw or future_regen_min <= self.tuning.regen_threshold_kw

        if heavy_ahead and soc < self.tuning.target_soc_high:
            target_fc = min(self.p_fc_max, max(target_fc, future_max + p_aux * 0.5))
        if regen_ahead and soc > self.tuning.target_soc_low:
            target_fc = min(target_fc, self.p_fc_max * 0.35)

        is_regen_now = p_req <= self.tuning.regen_threshold_kw and speed > 0.5
        is_idle = abs(p_req) < 5.0 and (at_stop or speed < 0.5)

        if is_idle:
            target_fc = max(p_aux, 0.0)

        # Ramp FC reference (faster ramp-down during braking)
        ramp_down = self.fc_ramp * self.dt * (1.5 if (p_req <= self.tuning.regen_threshold_kw and speed > 0.5) else 1.0)
        delta = np.clip(target_fc - self.fc_ref_kw, -ramp_down, self.fc_ramp * self.dt)
        self.fc_ref_kw = np.clip(self.fc_ref_kw + delta, 0.0, self.p_fc_max)

        batt_cmd_kw = 0.0

        if is_idle:
            # keep FC covering auxiliaries, optionally trickle-charge battery
            if soc < self.tuning.target_soc_mid:
                batt_cmd_kw = -min(self.tuning.idle_grid_charge_kw, self.p_batt_chg_max)
            else:
                batt_cmd_kw = 0.0

        elif is_regen_now:
            regen_power = min(abs(p_req), self.p_batt_chg_max)
            if soc > self.tuning.target_soc_high:
                # reduce intake to leave headroom
                headroom = max(0.05, (self.config.shield.soc_soft_max - soc) / 0.2)
                regen_power *= np.clip(headroom, 0.1, 1.0)

            # Default: clamp FC off during braking to prioritize regen and avoid
            # forcing friction brakes due to charge-rate saturation.
            target_fc = 0.0

            # Allow a small FC trickle only if SOC is well below the low target
            # and there is headroom after regen. Cap to 20% of FC max.
            p_regen_post_aux = float(info.get("p_regen_post_aux_kw", 0.0)) if info is not None else 0.0
            charge_headroom = max(0.0, self.p_batt_chg_max - max(0.0, p_regen_post_aux))
            if soc < (self.tuning.target_soc_low - 0.02) and charge_headroom > 1e-3:
                target_fc = min(self.p_fc_max * 0.2, charge_headroom)

            batt_cmd_kw = -regen_power

        else:
            residual = p_dem - self.fc_ref_kw
            if residual >= 1e-3:
                # Battery discharges to cover residual traction; account for discharge efficiency.
                batt_cmd_kw = residual / max(self.eta_discharge, 1e-3)
            else:
                # Excess FC power charges the battery.
                batt_cmd_kw = residual

        # SOC corridor management
        if soc <= self.tuning.target_soc_low:
            batt_cmd_kw = min(batt_cmd_kw, 0.0)  # encourage charging, block discharge
        elif soc >= self.config.shield.soc_soft_max:
            batt_cmd_kw = max(batt_cmd_kw, 0.0)  # block further charging

        # Clamp to hardware limits
        if batt_cmd_kw >= 0.0:
            batt_cmd_kw = min(batt_cmd_kw, self.p_batt_dis_max)
        else:
            batt_cmd_kw = max(batt_cmd_kw, -self.p_batt_chg_max)

        action = self._powers_to_action(self.fc_ref_kw, batt_cmd_kw)
        return action

    # ------------------------------------------------------------------
    def _powers_to_action(self, p_fc_kw: float, p_batt_kw: float) -> np.ndarray:
        if self.p_fc_max > 0:
            fc_frac = np.clip(p_fc_kw / self.p_fc_max, 0.0, 1.0)
        else:
            fc_frac = 0.0

        if p_batt_kw >= 0:
            batt_cmd = np.clip(p_batt_kw / self.p_batt_dis_max, 0.0, 1.0) if self.p_batt_dis_max > 0 else 0.0
        else:
            batt_cmd = np.clip(p_batt_kw / self.p_batt_chg_max, -1.0, 0.0) if self.p_batt_chg_max > 0 else 0.0

        return np.array([fc_frac, batt_cmd], dtype=np.float32)
