"""Offline optimal EMS via finite-horizon optimization (scaffold).

This module provides a policy that, given a known P_req time series,
computes an optimal schedule for FC and battery flows to minimize cost
under linear plant/shield constraints.

Notes
-----
- This is a scaffold: it tries to use cvxpy if available; otherwise it
  raises a clear error explaining how to enable it. The formulation is
  an LP with L1 smoothness, so any LP solver supported by cvxpy works.
- The action mapping follows the environment conventions:
  fc_frac in [0,1], batt_cmd in [-1,1] (discharge positive, charge negative).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..config import Config


@dataclass
class OfflinePlan:
    p_fc_kw: np.ndarray
    p_batt_dis_kw: np.ndarray
    p_batt_chg_kw: np.ndarray
    soc: np.ndarray
    tank: np.ndarray
    u_unmet_kw: np.ndarray
    u_curt_kw: np.ndarray


def _expand_preq_profile_vector(profile: List[Dict[str, float]], dt: float) -> np.ndarray:
    """Expand a piecewise-constant p_req profile into a per-step vector."""
    vec: List[float] = []
    if not profile:
        return np.asarray(vec, dtype=float)
    for seg in profile:
        duration = max(1, int(round(float(seg.get("duration_s", 0.0)) / dt)))
        p_req = float(seg.get("p_req_kw", 0.0))
        vec.extend([p_req] * duration)
    return np.asarray(vec, dtype=float)


def plan_offline_schedule(
    config: Config,
    p_req_series: Sequence[float] | List[Dict[str, float]],
    *,
    init_soc: float,
    init_tank: float,
    dt_seconds: float,
) -> OfflinePlan:
    """Solve the offline optimal schedule (LP) for a fixed P_req series.

    Parameters
    ----------
    config : Config
        Full environment configuration.
    p_req_series : Sequence[float] | List[Dict[str, float]]
        Either a per-step vector of P_req (kW) or a list of segments
        with keys {"duration_s", "p_req_kw"}.
    init_soc : float
        Initial SOC in [0,1].
    init_tank : float
        Initial normalized tank level in [0,1].
    dt_seconds : float
        Step duration in seconds.

    Returns
    -------
    OfflinePlan
        Optimal schedules and state trajectories.
    """
    try:
        import cvxpy as cp  # type: ignore
    except Exception as e:  # pragma: no cover - import guard
        raise RuntimeError(
            "cvxpy is required for plan_offline_schedule but is not installed.\n"
            "Install with `uv add cvxpy` or `pip install cvxpy`, then rerun."
        ) from e

    # Prepare P_req vector
    if isinstance(p_req_series, list) and p_req_series and isinstance(p_req_series[0], dict):
        p_req = _expand_preq_profile_vector(p_req_series, dt_seconds)
    else:
        p_req = np.asarray(p_req_series, dtype=float)
    T = int(p_req.shape[0])
    if T <= 0:
        raise ValueError("p_req_series is empty")

    # Shorthands
    batt = config.battery
    fc = config.fuel_cell
    plant = config.plant
    costs = config.costs
    rwd = config.reward_weights
    shield = config.shield
    dt_h = dt_seconds / 3600.0
    p_aux_kw = plant.p_aux_base_watts / 1000.0
    p_loss_kw = config.driver.p_loss_watts / 1000.0

    # Decision variables
    p_fc = cp.Variable(T, nonneg=True)
    p_dis = cp.Variable(T, nonneg=True)
    p_chg = cp.Variable(T, nonneg=True)
    u_unmet = cp.Variable(T, nonneg=True)
    u_curt = cp.Variable(T, nonneg=True)

    # States
    soc = cp.Variable(T + 1)
    tank = cp.Variable(T + 1)

    # Smoothness (L1 via split variables)
    d_fc_pos = cp.Variable(T - 1, nonneg=True)
    d_fc_neg = cp.Variable(T - 1, nonneg=True)
    d_b_pos = cp.Variable(T - 1, nonneg=True)
    d_b_neg = cp.Variable(T - 1, nonneg=True)

    cons = []
    # Initial conditions
    cons += [soc[0] == init_soc]
    cons += [tank[0] == init_tank]

    # Box constraints
    cons += [p_fc <= fc.p_fc_max_kw]
    cons += [p_dis <= batt.p_batt_max_discharge_kw]
    cons += [p_chg <= batt.p_batt_max_charge_kw]

    # SOC/tank corridors: respect soft band within hard limits
    soc_min = max(batt.soc_hard_min, config.shield.soc_soft_min)
    soc_max = min(batt.soc_hard_max, config.shield.soc_soft_max)
    cons += [soc >= soc_min, soc <= soc_max]
    cons += [tank >= fc.tank_hard_min, tank <= 1.0]

    # Dynamics
    for t in range(T):
        # Battery SOC update
        soc_next = soc[t] + dt_h * (
            batt.eta_charge * p_chg[t] / batt.e_batt_kwh
            - p_dis[t] / (cp.maximum(batt.eta_discharge, 1e-6) * batt.e_batt_kwh)
        )
        cons += [soc[t + 1] == soc_next]

        # H2 tank update (normalized)
        delta_m_h2 = (p_fc[t] * dt_h) / (fc.eta_fc * fc.h2_lhv_kwh_per_kg)
        cons += [
            tank[t + 1]
            == tank[t] - delta_m_h2 / cp.maximum(fc.tank_capacity_kg, 1e-6)
        ]

        # Demand vs supply constraints
        is_regen = p_req[t] < 0
        if not is_regen:
            # Traction: FC + batt supply + unmet = traction + aux
            cons += [p_fc[t] + batt.eta_discharge * p_dis[t] + u_unmet[t] == p_req[t] + p_aux_kw]
            cons += [p_chg[t] == 0]
            cons += [u_curt[t] == 0]
        else:
            # Regen magnitude
            R_t = float(-p_req[t])
            regen_aux = min(R_t, p_aux_kw)
            R_post = R_t - regen_aux
            # Bus balance at braking: supply must at least cover aux; unmet allowed
            cons += [p_fc[t] + batt.eta_discharge * p_dis[t] + u_unmet[t] == p_aux_kw]
            # Charging limited by sources: regen post-aux + FC excess
            # FC excess lower bound: p_fc - (net demand excluding batt and unmet)
            # During braking, net demand to cover aux is p_aux_kw
            fc_excess = cp.maximum(p_fc[t] - p_aux_kw, 0)
            cons += [p_chg[t] <= R_post + fc_excess]
            # Curtailment captures unmet regen (post-aux)
            cons += [u_curt[t] >= R_post - p_chg[t]]

        # Optional generic loss can be interpreted as extra aux that must be overcome.
        # We incorporate p_loss as an additional constant load on the kinematic side;
        # it doesn’t enter the DC balance directly in this linear scaffold.

    # FC ramp limits
    r_limit = fc.ramp_kw_per_s * dt_seconds
    if r_limit > 0 and T > 1:
        cons += [p_fc[1:] - p_fc[:-1] <= d_fc_pos]
        cons += [p_fc[:-1] - p_fc[1:] <= d_fc_neg]
        cons += [d_fc_pos <= r_limit]
        cons += [d_fc_neg <= r_limit]

    # Battery smoothness (optional, use L1 on signed command change)
    if T > 1:
        # Signed battery command b_t = p_dis - p_chg (scaled only in objective)
        cons += [p_dis[1:] - p_chg[1:] - (p_dis[:-1] - p_chg[:-1]) <= d_b_pos]
        cons += [-(p_dis[1:] - p_chg[1:] - (p_dis[:-1] - p_chg[:-1])) <= d_b_neg]

    # Objective
    c_h2 = costs.c_h2_eur_per_kg
    c_grid = costs.c_grid_eur_per_kwh
    lam_unmet = getattr(rwd, "lambda_unmet", 1e-6)
    lam_curt = getattr(rwd, "lambda_curt", 0.0)
    lam_smooth = getattr(rwd, "lambda_smooth", 0.0)

    h2_kg = cp.sum(p_fc) * dt_h / (fc.eta_fc * fc.h2_lhv_kwh_per_kg)
    grid_kwh = cp.sum(p_chg) * dt_h

    obj = (
        c_h2 * h2_kg
        + c_grid * grid_kwh
        + lam_unmet * cp.sum(u_unmet)
        + lam_curt * cp.sum(u_curt)
    )
    if T > 1 and lam_smooth > 0.0:
        obj += lam_smooth * (cp.sum(d_fc_pos + d_fc_neg + d_b_pos + d_b_neg))

    prob = cp.Problem(cp.Minimize(obj), cons)
    prob.solve(solver=cp.ECOS, verbose=False)

    if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        raise RuntimeError(f"Offline optimization failed with status: {prob.status}")

    p_fc_v = np.asarray(p_fc.value, dtype=float)
    p_dis_v = np.asarray(p_dis.value, dtype=float)
    p_chg_v = np.asarray(p_chg.value, dtype=float)
    soc_v = np.asarray(soc.value, dtype=float)
    tank_v = np.asarray(tank.value, dtype=float)
    u_unmet_v = np.asarray(u_unmet.value, dtype=float)
    u_curt_v = np.asarray(u_curt.value, dtype=float)

    return OfflinePlan(
        p_fc_kw=p_fc_v,
        p_batt_dis_kw=p_dis_v,
        p_batt_chg_kw=p_chg_v,
        soc=soc_v,
        tank=tank_v,
        u_unmet_kw=u_unmet_v,
        u_curt_kw=u_curt_v,
    )


class OfflineOptimalEMS:
    """Policy that replays an offline optimal schedule as actions.

    Usage
    -----
    - Initialize with a per-step P_req vector or a piecewise profile.
    - On reset(), compute the optimal plan using the initial info dict
      (SOC, tank). Then at each act(), emit the planned action.
    """

    def __init__(self, config: Config, profile_or_vector: Sequence[float] | List[Dict[str, float]]):
        self.config = config
        self._profile_or_vector = profile_or_vector
        self._t = 0
        self._plan: Optional[OfflinePlan] = None

    def reset(self) -> None:
        self._t = 0
        self._plan = None

    def _ensure_plan(self, info: Dict[str, float]) -> None:
        if self._plan is not None:
            return
        init_soc = float(info.get("soc", self.config.battery.soc_init))
        init_tank = float(info.get("tank_level", 1.0))
        dt_s = float(self.config.scenario.sim.dt_seconds)
        self._plan = plan_offline_schedule(
            self.config,
            self._profile_or_vector,
            init_soc=init_soc,
            init_tank=init_tank,
            dt_seconds=dt_s,
        )

    def act(self, obs: Optional[np.ndarray] = None, info: Optional[Dict[str, float]] = None) -> np.ndarray:  # type: ignore[override]
        if info is None:
            raise ValueError("OfflineOptimalEMS requires env info to act (contains SOC/tank and step)")
        self._ensure_plan(info)
        assert self._plan is not None
        t = min(self._t, len(self._plan.p_fc_kw) - 1)
        p_fc = float(self._plan.p_fc_kw[t])
        p_dis = float(self._plan.p_batt_dis_kw[t])
        p_chg = float(self._plan.p_batt_chg_kw[t])

        # Map to action space: fc_frac in [0,1]; batt_cmd in [-1,1]
        fc_frac = 0.0 if self.config.fuel_cell.p_fc_max_kw <= 0 else p_fc / self.config.fuel_cell.p_fc_max_kw
        batt_pos = 0.0 if self.config.battery.p_batt_max_discharge_kw <= 0 else p_dis / self.config.battery.p_batt_max_discharge_kw
        batt_neg = 0.0 if self.config.battery.p_batt_max_charge_kw <= 0 else p_chg / self.config.battery.p_batt_max_charge_kw
        batt_cmd = np.clip(batt_pos - batt_neg, -1.0, 1.0)

        self._t = t + 1
        return np.asarray([np.clip(fc_frac, 0.0, 1.0), batt_cmd], dtype=np.float32)


__all__ = ["OfflineOptimalEMS", "plan_offline_schedule", "OfflinePlan"]
