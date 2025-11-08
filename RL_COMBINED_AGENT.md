# RL Combined Control — Start→Stop Baseline (Draft)

This document proposes a single RL agent that jointly controls traction/braking and the energy subsystem (FC/battery). The initial scope is a simple start→stop route with a fixed time budget; we will iterate and expand to richer scenarios later.

## Goal

- Train one policy that:
  - Chooses traction/brake power and EMS split each step.
  - Arrives by the episode deadline with speed ≈ 0 at the terminal.
  - Minimizes lifecycle energy cost subject to safety/physics enforced by the shield/plant.

## Scenario (v1)

- Route: Station A dwell → accelerate → cruise → brake → Station B dwell.
- Flat track; fixed passenger mass; fixed initial SOC/tank (no randomization for v1).
- Episode horizon `T`: fixed (e.g., 1,800 s). Target distance `D_target` chosen to be feasible.
- Time step: 1 s.

## Action Space (combined control)

- `action = [u_trac, fc_frac, batt_cmd]`
  - `u_trac ∈ [-1, 1]` → traction request `P_req_agent ∈ [-P_brake_max, +P_pull_max]` with per‑step slew rate.
  - `fc_frac ∈ [0, 1]` → `P_fc = fc_frac * P_fc_max`.
  - `batt_cmd ∈ [-1, 1]` → discharge/charge map (positive = discharge, negative = charge) within battery limits.
- Shield still enforces: FC ramp, SOC corridor, C‑rates, tank minimum.

## Observation Space (minimal, v1)

Start from existing 10‑D vector and adapt for combined control:
- speed/v_max
- noisy SOC, noisy tank
- phase/progress: `t/T` (0→1)
- filtered traction proxy: filtered `P_req_agent` normalized
- last action (3 channels in combined mode)
- nuisance noise (3 channels)
- Optional (later v2): time remaining, distance remaining, schedule slack.

## Rewards

Per‑step cost (negative):

```
r_cost = -(c_h2*Δm_H2 + c_grid*ΔE_charge + λ_unmet*P_unmet + λ_smooth*||a_t - a_{t-1}||)
```

Progress shaping (potential‑based):

```
r_prog = κ_dist * (distance_t - distance_{t-1})
```

Schedule tracking (optional for v1, helpful later):

```
d_nom(t) = (t/T) * D_target
r_sched = -λ_sched * max(0, d_nom(t) - distance_t)  # lateness only
```

Terminal penalties/bonus at `t = T`:

```
s_short  = max(0, D_target - distance_T)
v_resid  = max(0, speed_T - v_stop_threshold)
s_over   = max(0, distance_T - D_target)  # optional overshoot penalty
r_term   = -λ_term_pos*s_short - λ_term_speed*v_resid - λ_overshoot*s_over + b_on_time*1[s_short=0 ∧ v_resid=0]
```

Total:

```
r_t = r_cost + r_prog (+ r_sched) ; and at t=T add r_term
```

Suggested initial scales (tunable):
- `λ_unmet`: 1e−4 … 1e−3 per kW
- `κ_dist`: 1e−3 per meter‑equivalent (so typical progress yields ~0.01–0.1 r/s)
- `λ_smooth`: 1e−2 (L2/L1 on action deltas)
- `λ_sched`: 1e−3 … 1e−2 per km lag (optional)
- `λ_term_pos`: 10 … 100 per km shortfall (dominates failure to arrive)
- `λ_term_speed`: 1 … 10 per m/s residual
- `λ_overshoot`: 0.5 … 2 (if overshoot is undesirable)
- `v_stop_threshold`: 0.2 … 0.5 m/s
- `b_on_time`: +1.0

## Config Knobs to Add (proposal)

- `scenario.control_mode: "combined" | "split"` (default: `split` for backward compatibility)
- `scenario.route`:
  - `d_target_km`
  - `v_stop_threshold_mps`
- `scenario.traction` (combined mode only):
  - `p_pull_max_kw`, `p_brake_max_kw`
  - `rate_limit_kw_per_s`, `action_smoothing_tau_s`
- `scenario.reward_weights` additions:
  - `kappa_distance`, `lambda_schedule`, `lambda_terminal_pos`, `lambda_terminal_speed`, `lambda_overshoot`

## Feasibility Guardrails

- Choose `P_pull_max`, `P_brake_max`, and rate limits so `D_target` is achievable within `T`.
- Keep auxiliaries and generic losses realistic but not prohibitive.
- Start with fixed passenger mass, SOC/tank; add randomization after baseline.

## Training Plan (v1)

- Algorithm: SAC (preferred) or PPO.
- Fixed scenario (no randomization), deterministic seed.
- Disable live rendering; log KPIs; save final frame for periodic checks.
- 1–3M steps to get a baseline policy.

KPIs:
- On‑time arrival rate (distance_T ≥ D_target and speed_T ≤ v_stop_threshold).
- €/km, H2 vs grid split.
- Unmet demand (avg/max), constraint violations.
- Final SOC/tank sanity.

## Milestones

1) Wire combined mode action/obs/reward/terminal (behind config flag); run smoke test.
2) Tune weights to achieve on‑time arrival in the fixed scenario.
3) Add light randomization (initial SOC/tank, passenger mass, aux bias) and retune.
4) Add schedule tracking; later add grades/stops; expand to multi‑stop routes.

## Open Questions

- Do we penalize overshoot distance or allow it if stopped by T?
- Should progress shaping be distance‑only, or include negative shaping when braking late?
- Do we need observation of “time remaining” and “distance remaining” in v1 or only in v2?

