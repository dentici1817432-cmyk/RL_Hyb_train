"""Rule-based EMS that follows the scripted power demand with smooth FC and battery decisions."""
import numpy as np
from typing import Dict, Any, Optional

from .config import Config


class BaselineEMS:
    """Simple EMS that tracks manual P_req with an FC follower and battery support."""

    def __init__(self, config: Config):
        self.config = config

        self.dt = config.sim.dt_seconds
        self.p_fc_max = config.fuel_cell.p_fc_max_kw
        self.fc_ramp_kw_per_s = config.fuel_cell.ramp_kw_per_s
        self.p_batt_dis_max = config.battery.p_batt_max_discharge_kw
        self.p_batt_chg_max = config.battery.p_batt_max_charge_kw
        self.p_aux_base = config.plant.p_aux_base_watts / 1000.0

        self.fc_idle_threshold_kw = 30.0  # shut down FC below this demand
        self.fc_target_kw = 0.0

    # ------------------------------------------------------------------
    def reset(self):
        self.fc_target_kw = 0.0

    def act(self, obs: Optional[np.ndarray] = None, info: Optional[Dict[str, Any]] = None) -> np.ndarray:
        """Return EMS action [fc_frac, batt_cmd]."""
        if info is None and obs is None:
            raise ValueError("BaselineEMS requires info or observation input")

        if info is not None:
            p_req_kw = info.get("p_req_kw", 0.0)
            soc = float(info.get("soc", 0.65))
            speed_mps = float(info.get("speed_mps", 0.0))
            p_unmet_prev = float(info.get("p_unmet_kw", 0.0))
            p_aux_kw = float(info.get("p_aux_kw", self.p_aux_base))
        else:
            # Fallback to observation estimates (less accurate)
            p_req_kw = float(((obs[4] + 1.0) / 2.0) * self.config.observations.p_req_max_kw)
            soc = float(np.clip((obs[1] + 1.0) / 2.0, 0.0, 1.0))
            speed_mps = 0.0
            p_unmet_prev = 0.0
            p_aux_kw = self.p_aux_base

        total_demand_kw = p_req_kw + p_aux_kw

        # Desired FC power follows the positive part of the demand
        fc_desired_kw = np.clip(total_demand_kw, 0.0, self.p_fc_max)
        if fc_desired_kw < self.fc_idle_threshold_kw and self.fc_target_kw < self.fc_idle_threshold_kw:
            fc_desired_kw = 0.0

        # Apply ramp limits
        delta_max = self.fc_ramp_kw_per_s * self.dt
        if fc_desired_kw > self.fc_target_kw:
            self.fc_target_kw = min(fc_desired_kw, self.fc_target_kw + delta_max)
        else:
            self.fc_target_kw = max(fc_desired_kw, self.fc_target_kw - delta_max)

        fc_power_kw = self.fc_target_kw

        # Battery covers the residual demand
        batt_power_kw = total_demand_kw - fc_power_kw

        # Incorporate unmet from previous step by pushing battery harder
        if p_unmet_prev > 1e-3:
            batt_power_kw += min(p_unmet_prev, self.p_batt_dis_max)

        # Battery limits (positive = discharge, negative = charge)
        if batt_power_kw > self.p_batt_dis_max:
            batt_power_kw = self.p_batt_dis_max
        if batt_power_kw < -self.p_batt_chg_max:
            batt_power_kw = -self.p_batt_chg_max

        # Simple SOC protection: if SOC is very high, avoid charging; if very low, avoid discharging
        soc_low_band = self.config.shield.soc_soft_min
        soc_high_band = self.config.shield.soc_soft_max
        if soc <= soc_low_band:
            batt_power_kw = max(batt_power_kw, 0.0)
        elif soc >= soc_high_band:
            batt_power_kw = min(batt_power_kw, 0.0)

        # Map powers to normalized actions
        if self.p_fc_max > 0:
            fc_frac = np.clip(fc_power_kw / self.p_fc_max, 0.0, 1.0)
        else:
            fc_frac = 0.0

        if batt_power_kw >= 0:
            batt_cmd = np.clip(batt_power_kw / self.p_batt_dis_max, 0.0, 1.0)
        else:
            batt_cmd = np.clip(batt_power_kw / self.p_batt_chg_max, -1.0, 0.0)

        return np.array([fc_frac, batt_cmd], dtype=np.float32)

    def act_from_info(self, info: Dict[str, Any]) -> np.ndarray:
        return self.act(info=info)

