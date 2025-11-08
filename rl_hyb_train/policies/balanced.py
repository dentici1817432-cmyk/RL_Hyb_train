"""Rule-based EMS that balances FC charging and battery usage."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Any

import numpy as np

from ..config import Config


@dataclass
class BalancedEMSConfig:
    target_soc: float = 0.80
    soc_high_margin: float = 0.05
    soc_full: float = 0.995
    charge_boost_kw: float = 40.0
    regen_clip_soc: float = 0.99


class BalancedEMS:
    """
    Simple supervisory policy:
      - Run FC hard to charge battery up to target SOC (80%)
      - Use regen to fill battery to 100% to avoid friction braking
      - When SOC > target, back off the FC and rely on battery until SOC returns to target
    """

    def __init__(self, config: Config, tuning: Optional[BalancedEMSConfig] = None):
        self.config = config
        self.tuning = tuning or BalancedEMSConfig()

        self.dt = config.sim.dt_seconds
        self.p_fc_max = config.fuel_cell.p_fc_max_kw
        self.fc_ramp = config.fuel_cell.ramp_kw_per_s
        self.p_batt_dis_max = config.battery.p_batt_max_discharge_kw
        self.p_batt_chg_max = config.battery.p_batt_max_charge_kw
        self.eta_dis = config.battery.eta_discharge

        self.p_aux_base = config.plant.p_aux_base_watts / 1000.0
        self.fc_ref_kw = 0.0

    def reset(self):
        self.fc_ref_kw = 0.0

    def act(self, obs: Optional[np.ndarray] = None, info: Optional[Dict[str, Any]] = None) -> np.ndarray:
        if info is None:
            raise ValueError("BalancedEMS requires env info to act")
        return self._act(info)

    def _act(self, info: Dict[str, Any]) -> np.ndarray:
        p_req = float(info.get("p_req_kw", 0.0))
        soc = float(info.get("soc", 0.65))
        speed = float(info.get("speed_mps", 0.0))
        p_aux = float(info.get("p_aux_kw", self.p_aux_base))

        p_dem = p_req + p_aux

        target_soc = self.tuning.target_soc
        soc_high = target_soc + self.tuning.soc_high_margin

        # --- Fuel cell target ---
        fc_target = 0.0
        if p_req >= 0:
            if soc < target_soc:
                fc_target = np.clip(p_dem + self.tuning.charge_boost_kw, 0.0, self.p_fc_max)
            elif soc > soc_high:
                fc_target = max(p_aux, 0.0)
            else:
                fc_target = np.clip(max(p_dem, 0.0), 0.0, self.p_fc_max)
        else:
            # Braking: keep FC low to leave room for regen, only cover auxiliaries if needed
            fc_target = 0.0 if soc > self.tuning.regen_clip_soc else max(0.0, min(p_aux, self.p_fc_max * 0.2))

        # Ramp FC reference
        ramp_delta = self.fc_ramp * self.dt
        self.fc_ref_kw = np.clip(fc_target, self.fc_ref_kw - ramp_delta, self.fc_ref_kw + ramp_delta)

        # --- Battery command ---
        residual = p_dem - self.fc_ref_kw
        batt_kw = residual

        if p_req < 0:
            # Regenerative braking: request charge up to limits
            regen_power = abs(p_req)
            batt_kw = -min(self.p_batt_chg_max, regen_power)
            if soc >= self.tuning.soc_full:
                batt_kw = 0.0
        else:
            if soc > soc_high:
                # Encourage discharge to pull SOC back down
                batt_kw = min(self.p_batt_dis_max, residual + self.tuning.charge_boost_kw * 0.5)
            elif soc < target_soc:
                # Allow discharge to meet demand but bias towards charging when possible
                batt_kw = residual
                if residual < 0.0:
                    batt_kw = max(residual, -self.p_batt_chg_max)
                else:
                    batt_kw = min(residual, self.p_batt_dis_max)
            else:
                batt_kw = residual

        # Clamp to hardware limits
        batt_kw = np.clip(batt_kw, -self.p_batt_chg_max, self.p_batt_dis_max)

        # Map to normalized action
        fc_frac = np.clip(self.fc_ref_kw / self.p_fc_max, 0.0, 1.0) if self.p_fc_max > 0 else 0.0
        if batt_kw >= 0:
            batt_cmd = np.clip(batt_kw / self.p_batt_dis_max, 0.0, 1.0) if self.p_batt_dis_max > 0 else 0.0
        else:
            batt_cmd = np.clip(batt_kw / self.p_batt_chg_max, -1.0, 0.0) if self.p_batt_chg_max > 0 else 0.0

        return np.array([fc_frac, batt_cmd], dtype=np.float32)
