"""Simple receding-horizon MPC EMS.

Minimizes operational cost over a short horizon using a coarse grid search over
fuel-cell (FC) setpoints and charging/discharging residual handled by the
battery subject to limits and SOC corridor. No external solver needed.

This controller does not require a future P_req profile; by default it assumes
current demand persists over the horizon (zero-order hold). If a forecast
vector is provided via `set_forecast()`, it will use that instead.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Dict, Any

import numpy as np

from ..config import Config


@dataclass
class MPCConfig:
    horizon_s: int = 30
    n_grid: int = 6  # number of FC grid levels in [0, p_fc_max]
    beam_width: int = 3  # number of hypotheses kept per step
    lambda_smooth: float = 0.01
    lambda_unmet: float = 1e-6
    soc_soft_min_margin: float = 0.02
    soc_soft_max_margin: float = 0.02
    # SOC guidance (target corridor center). Penalize deficit vs. target to
    # avoid ending trips too low on SOC (encourages FC use when needed).
    soc_target: float = 0.55
    lambda_soc_deficit: float = 2000.0


class MPCEms:
    def __init__(self, config: Config, tuning: Optional[MPCConfig] = None):
        self.config = config
        self.dt = config.sim.dt_seconds
        self.t = 0

        self.p_fc_max = config.fuel_cell.p_fc_max_kw
        self.fc_ramp = config.fuel_cell.ramp_kw_per_s
        self.p_dis_max = config.battery.p_batt_max_discharge_kw
        self.p_chg_max = config.battery.p_batt_max_charge_kw
        self.eta_chg = config.battery.eta_charge
        self.eta_dis = config.battery.eta_discharge

        self.c_h2 = config.costs.c_h2_eur_per_kg
        self.eta_fc = config.fuel_cell.eta_fc
        self.h2_lhv = config.fuel_cell.h2_lhv_kwh_per_kg

        self.c_grid = config.costs.c_grid_eur_per_kwh

        self.soc_min = config.shield.soc_soft_min
        self.soc_max = config.shield.soc_soft_max

        self.tuning = tuning or MPCConfig()

        self.p_fc_prev = 0.0
        self._forecast: Optional[np.ndarray] = None

    # Optional: provide a forecast of p_req over horizon (length >= horizon)
    def set_forecast(self, p_req_vector: List[float]) -> None:
        self._forecast = np.asarray(p_req_vector, dtype=float)

    def _h2_cost(self, p_fc_kw: float) -> float:
        # energy from FC per step (kWh) = p * dt_h, H2 mass = E_fc / (eta_fc * LHV)
        dt_h = self.dt / 3600.0
        if p_fc_kw <= 0:
            return 0.0
        m_kg = (p_fc_kw * dt_h) / (self.eta_fc * self.h2_lhv)
        return self.c_h2 * m_kg

    def _grid_cost(self, p_batt_kw: float) -> float:
        # Grid cost applies only when charging from grid (we approximate any net charge as grid energy)
        if p_batt_kw >= 0:
            return 0.0
        dt_h = self.dt / 3600.0
        e_kwh = abs(p_batt_kw) * dt_h
        return self.c_grid * e_kwh

    def _simulate(self, p_dem_seq: np.ndarray, soc0: float, p_fc0: float) -> tuple[float, float, float]:
        """Beam search over FC sequences with ramp constraint.

        Returns: (best_p_fc0, best_p_batt0, best_cost)
        """
        H = min(self.tuning.horizon_s, len(p_dem_seq))
        if H <= 0:
            return p_fc0, 0.0, 0.0

        fc_levels = np.linspace(0.0, self.p_fc_max, num=max(2, self.tuning.n_grid))
        beam_w = max(1, int(self.tuning.beam_width))

        # Beam state: (cum_cost, p_fc_prev, soc, first_fc, first_batt)
        beams = [(0.0, float(p_fc0), float(soc0), None, None)]

        dt_h = self.dt / 3600.0

        for k in range(H):
            p_dem = float(p_dem_seq[k])
            new_beams: list[tuple[float, float, float, Optional[float], Optional[float]]] = []
            for cost_so_far, p_fc_prev, soc_prev, first_fc, first_batt in beams:
                delta = self.fc_ramp * self.dt
                lo = max(0.0, p_fc_prev - delta)
                hi = min(self.p_fc_max, p_fc_prev + delta)
                for p_fc in fc_levels:
                    if p_fc < lo or p_fc > hi:
                        continue
                    residual = p_dem - p_fc
                    p_batt = float(np.clip(residual, -self.p_chg_max, self.p_dis_max))

                    soc = soc_prev
                    if p_batt >= 0:
                        soc -= (p_batt / (self.config.battery.e_batt_kwh * self.eta_dis)) * dt_h
                    else:
                        soc += (abs(p_batt) * self.eta_chg / self.config.battery.e_batt_kwh) * dt_h

                    corridor_pen = 0.0
                    if soc < self.soc_min:
                        corridor_pen += 1000.0 * (self.soc_min - soc)
                        soc = self.soc_min
                    if soc > self.soc_max:
                        corridor_pen += 1000.0 * (soc - self.soc_max)
                        soc = self.soc_max

                    unmet = max(0.0, residual - p_batt)
                    soc_deficit = max(0.0, self.tuning.soc_target - soc)

                    step_cost = (
                        self._h2_cost(p_fc)
                        + self._grid_cost(p_batt)
                        + self.tuning.lambda_unmet * unmet
                        + self.tuning.lambda_smooth * abs(p_fc - p_fc_prev)
                        + corridor_pen
                        + self.tuning.lambda_soc_deficit * soc_deficit
                    )

                    nf = first_fc if first_fc is not None else p_fc
                    nb = first_batt if first_batt is not None else p_batt
                    new_beams.append((cost_so_far + step_cost, p_fc, soc, nf, nb))

            # Keep top-K beams
            new_beams.sort(key=lambda x: x[0])
            beams = new_beams[:beam_w]

        best_cost, _, _, best_first_fc, best_first_batt = beams[0]
        return best_first_fc or p_fc0, best_first_batt or 0.0, best_cost

    def act(self, obs: Optional[np.ndarray] = None, info: Optional[Dict[str, Any]] = None) -> np.ndarray:
        if info is None:
            raise ValueError("MPCEms requires env info to act")
        return self.act_from_info(info)

    def act_from_info(self, info: Dict[str, Any]) -> np.ndarray:
        # Build demand forecast (zero-order hold)
        p_req = float(info.get("p_req_kw", 0.0))
        p_aux = float(info.get("p_aux_kw", self.config.plant.p_aux_base_watts / 1000.0))
        p_dem0 = p_req + p_aux
        H = self.tuning.horizon_s
        if self._forecast is not None and len(self._forecast) >= H:
            p_dem_seq = np.asarray(self._forecast[:H]) + p_aux
        else:
            p_dem_seq = np.full(H, p_dem0, dtype=float)

        soc = float(info.get("soc", 0.65))

        p_fc0 = float(info.get("p_fc_kw", self.p_fc_prev))
        p_fc0 = float(np.clip(p_fc0, 0.0, self.p_fc_max))

        p_fc_cmd, p_batt_cmd_kw, _ = self._simulate(p_dem_seq, soc, p_fc0)

        # Map to normalized actions
        self.p_fc_prev = p_fc_cmd
        if self.p_fc_max > 0:
            fc_frac = np.clip(p_fc_cmd / self.p_fc_max, 0.0, 1.0)
        else:
            fc_frac = 0.0
        if p_batt_cmd_kw >= 0:
            batt_cmd = np.clip(p_batt_cmd_kw / self.p_dis_max, 0.0, 1.0)
        else:
            batt_cmd = np.clip(p_batt_cmd_kw / self.p_chg_max, -1.0, 0.0)

        return np.array([fc_frac, batt_cmd], dtype=np.float32)
