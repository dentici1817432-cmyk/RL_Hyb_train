"""Reusable plotting utilities for EMS/plant histories."""
from __future__ import annotations

from typing import Dict

import numpy as np
import matplotlib.pyplot as plt


def plot_history(history: Dict[str, np.ndarray], scenario_name: str):
    """
    Plot power split, stores, and braking/action data for a simulation history.

    Args:
        history: Dict of numpy arrays (from EMSTestHarness.get_history_arrays()).
        scenario_name: Label for the figure title.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f"EMS + Train Dynamics Test: {scenario_name}", fontsize=14, fontweight="bold")

    time = history["time"]

    # --- Power split (stacked) -------------------------------------------------
    ax = axes[0, 0]
    p_batt = history["p_batt_kw"]
    p_fc = history["p_fc_kw"]
    p_batt_discharge = np.maximum(p_batt, 0.0)
    p_batt_charge = np.minimum(p_batt, 0.0)

    ax.fill_between(time, 0, p_fc, alpha=0.6, color="blue", label="P_FC")
    ax.fill_between(time, p_fc, p_fc + p_batt_discharge, alpha=0.6, color="red", label="P_Batt (discharge)")
    if np.any(p_batt_charge < 0):
        ax.fill_between(time, 0, p_batt_charge, alpha=0.6, color="orange", label="P_Batt (charge)")

    p_total_demand = history["p_req_kw"] + history["p_aux_kw"]
    ax.plot(time, history["p_req_kw"], "k--", linewidth=2, label="P_req (traction)", alpha=0.8)
    ax.plot(time, p_total_demand, "k-", linewidth=2, label="P_total (req+aux)", alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (kW)")
    ax.set_title("Power Split (Stacked)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="k", linestyle="-", linewidth=0.5)

    # --- Demand vs supply ------------------------------------------------------
    ax = axes[0, 1]
    p_req = history["p_req_kw"]
    p_aux = history["p_aux_kw"]
    ax.fill_between(time, 0, p_req, alpha=0.5, color="purple", label="P_req (traction)")
    ax.fill_between(time, p_req, p_req + p_aux, alpha=0.5, color="green", label="P_aux")

    p_total_supplied = history["p_fc_kw"] + history["p_batt_kw"]
    ax.plot(time, p_total_supplied, "m-", linewidth=2, label="Total Supplied (FC+Batt)", alpha=0.9)

    p_error = p_total_supplied - (p_req + p_aux)
    ax2 = ax.twinx()
    ax2.plot(time, p_error, "r:", linewidth=1, label="Supply - Demand", alpha=0.5)
    ax2.set_ylabel("Error (kW)", color="r")
    ax2.tick_params(axis="y", labelcolor="r")
    ax2.axhline(y=0, color="r", linestyle="--", linewidth=0.5, alpha=0.3)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (kW)")
    ax.set_title("Power Balance: Demand vs Supply")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- SOC -------------------------------------------------------------------
    ax = axes[1, 0]
    ax.plot(time, history["soc"] * 100, "b-", linewidth=2)
    ax.axhline(y=20, color="r", linestyle="--", alpha=0.5, label="SOC limits")
    ax.axhline(y=90, color="r", linestyle="--", alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("SOC (%)")
    ax.set_title("Battery State of Charge")
    ax.set_ylim([0, 100])
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- H2 level --------------------------------------------------------------
    ax = axes[1, 1]
    ax.plot(time, history["h2_level"] * 100, "g-", linewidth=2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("H2 Level (%)")
    ax.set_title("Hydrogen Tank Level")
    ax.set_ylim([0, 100])
    ax.grid(True, alpha=0.3)

    # --- Speed -----------------------------------------------------------------
    ax = axes[2, 0]
    ax.plot(time, history["speed_mps"], "purple", linewidth=2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Train Speed")
    ax.grid(True, alpha=0.3)

    # --- Braking or EMS actions ------------------------------------------------
    ax = axes[2, 1]
    p_brake_total = history["p_brake_total_kw"]
    has_braking = np.any(np.abs(p_brake_total) > 10.0)

    if has_braking:
        p_brake_regen = history["p_brake_regen_kw"]
        p_brake_friction = history["p_brake_friction_kw"]
        ax.fill_between(time, 0, p_brake_regen, alpha=0.6, color="green", label="Regen (electrical)")
        ax.fill_between(time, p_brake_regen, p_brake_total, alpha=0.6, color="brown", label="Friction (mechanical)")
        ax.plot(time, p_brake_total, "k-", linewidth=2, label="Total brake demand", alpha=0.8)
        ax.set_ylabel("Braking Power (kW)")
        ax.set_title("Brake Power Split (Regen vs Friction)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color="k", linestyle="-", linewidth=0.5)
    else:
        ax.plot(time, history["fc_frac"], "b--", alpha=0.5, label="FC frac (raw)")
        ax.plot(time, history["fc_frac_shielded"], "b-", linewidth=2, label="FC frac (shielded)")
        ax.plot(time, history["batt_cmd"], "r--", alpha=0.5, label="Batt cmd (raw)")
        ax.plot(time, history["batt_cmd_shielded"], "r-", linewidth=2, label="Batt cmd (shielded)")
        ax.set_ylabel("Action Value")
        ax.set_title("EMS Actions (Before/After Shield)")
        ax.legend()
        ax.grid(True, alpha=0.3)

    ax.set_xlabel("Time (s)")
    plt.tight_layout()
    return fig

