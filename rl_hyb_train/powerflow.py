"""Reusable helpers for DC-bus power balance and energy bookkeeping."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class BatteryFlow:
    """Battery command resolved into discharge/charge components."""

    command_kw: float
    discharge_kw: float
    charge_kw: float
    delivered_kw: float


@dataclass
class RegenFlow:
    """Breakdown of regenerative braking power."""

    total_kw: float = 0.0
    aux_kw: float = 0.0
    post_aux_kw: float = 0.0
    captured_kw: float = 0.0
    friction_kw: float = 0.0


@dataclass
class ChargeAllocation:
    """Battery charging accepted from different sources."""

    actual_kw: float = 0.0
    from_regen_kw: float = 0.0
    from_fc_kw: float = 0.0


@dataclass
class PowerBalance:
    """Simple DC-bus balance summary."""

    p_dem_kw: float
    p_supply_kw: float
    p_unmet_kw: float
    p_delivered_kw: float


def resolve_battery_flow(p_batt_kw: float, eta_discharge: float) -> BatteryFlow:
    """Split a signed battery command into discharge/charge components."""
    discharge_kw = max(p_batt_kw, 0.0)
    charge_kw = max(-p_batt_kw, 0.0)
    delivered_kw = discharge_kw * eta_discharge
    return BatteryFlow(
        command_kw=p_batt_kw,
        discharge_kw=discharge_kw,
        charge_kw=charge_kw,
        delivered_kw=delivered_kw,
    )


def compute_regen_flow(
    p_req_kw: float,
    speed_mps: float,
    p_aux_kw: float,
    charge_cmd_kw: float,
    speed_threshold: float = 0.01,
) -> RegenFlow:
    """
    Determine how much regenerative braking is available and captured.

    Args:
        p_req_kw: Requested traction power (negative = braking).
        speed_mps: Current train speed (regen only while moving).
        p_aux_kw: Auxiliary loads that consume regen first.
        charge_cmd_kw: Desired battery charging power (kW, >=0).
        speed_threshold: Minimum speed for regen eligibility.
    """
    if p_req_kw >= 0.0 or speed_mps <= speed_threshold:
        total = abs(p_req_kw) if p_req_kw < 0.0 else 0.0
        return RegenFlow(total_kw=total, aux_kw=0.0, post_aux_kw=0.0, captured_kw=0.0, friction_kw=total)

    regen_total = abs(p_req_kw)
    regen_aux = min(regen_total, max(p_aux_kw, 0.0))
    post_aux = max(0.0, regen_total - regen_aux)
    capture = min(max(charge_cmd_kw, 0.0), post_aux)
    friction = max(0.0, regen_total - regen_aux - capture)
    return RegenFlow(
        total_kw=regen_total,
        aux_kw=regen_aux,
        post_aux_kw=post_aux,
        captured_kw=capture,
        friction_kw=friction,
    )


def fc_excess_power(p_fc_kw: float, p_dem_kw: float) -> float:
    """Compute FC power that can be diverted to charge the battery."""
    if p_fc_kw <= 0.0:
        return 0.0
    if p_dem_kw <= 0.0:
        return p_fc_kw
    return max(0.0, p_fc_kw - p_dem_kw)


def allocate_battery_charging(
    charge_cmd_kw: float,
    regen_flow: RegenFlow,
    fc_excess_kw: float,
) -> ChargeAllocation:
    """Clamp battery charging to available sources (regen + FC excess)."""
    if charge_cmd_kw <= 0.0:
        return ChargeAllocation()

    regen_supply = min(charge_cmd_kw, regen_flow.post_aux_kw)
    remaining = charge_cmd_kw - regen_supply
    fc_supply = min(max(fc_excess_kw, 0.0), remaining)
    actual = regen_supply + fc_supply
    return ChargeAllocation(actual_kw=actual, from_regen_kw=regen_supply, from_fc_kw=fc_supply)


def compute_power_balance(
    p_req_kw: float,
    p_aux_kw: float,
    p_fc_kw: float,
    p_batt_delivered_kw: float,
) -> PowerBalance:
    """Return the DC-bus balance for the given demand/supply pair."""
    p_dem_kw = p_req_kw + p_aux_kw
    p_supply_kw = p_fc_kw + p_batt_delivered_kw
    residual = p_dem_kw - p_supply_kw
    p_unmet_kw = max(residual, 0.0)
    p_delivered_kw = p_supply_kw - p_aux_kw - p_unmet_kw
    return PowerBalance(
        p_dem_kw=p_dem_kw,
        p_supply_kw=p_supply_kw,
        p_unmet_kw=p_unmet_kw,
        p_delivered_kw=p_delivered_kw,
    )


def soc_delta(
    discharge_kw: float,
    charge_kw: float,
    eta_discharge: float,
    eta_charge: float,
    e_batt_kwh: float,
    dt_hours: float,
) -> float:
    """Return SOC delta given battery power components."""
    if e_batt_kwh <= 0.0:
        return 0.0
    delta_discharge = -discharge_kw / (e_batt_kwh * max(eta_discharge, 1e-6)) * dt_hours
    delta_charge = charge_kw * eta_charge / e_batt_kwh * dt_hours
    return delta_discharge + delta_charge


def h2_consumption_kg(p_fc_kw: float, eta_fc: float, h2_lhv_kwh_per_kg: float, dt_hours: float) -> float:
    """Return hydrogen mass consumed during the step."""
    if p_fc_kw <= 0.0 or eta_fc <= 0.0 or h2_lhv_kwh_per_kg <= 0.0:
        return 0.0
    return (p_fc_kw * dt_hours) / (eta_fc * h2_lhv_kwh_per_kg)
