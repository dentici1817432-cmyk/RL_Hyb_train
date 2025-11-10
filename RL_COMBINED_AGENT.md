# RL EMS Agent — Iteration Plan (Living Doc)

This plan outlines iterative development for an RL policy that performs Energy Management (EMS) to accommodate an exogenous driver/ATO power request `P_req`. We keep reward shaping light so strategies emerge from physics, preview, and constraints. A separate appendix (below) retains an exploratory “combined control” track.

## Objectives

- Learn a reactive–anticipatory mapping g(obs) → [fc_frac, batt_cmd] that:
  - Minimizes OPEX proxy (H₂ + grid) with light regularization.
  - Respects shielded constraints without frequent projections (ramp, SOC corridor, C‑rate).
  - Maintains SOC headroom for forecast errors and emergencies (robustness).
  - Generalizes across route variations and driver deviations from the schedule.

## Observations (v1 → v3)

- v1 (minimal, existing): speed, noisy SOC/tank, filtered P_req, last action, noise.
- v2 (preview): add time‑to‑stop, grade/segment indicator, P_req trend.
- v3 (forecast split): expose `P_req_forecast` vs realized `P_req` + uncertainty cue.
  - Keep option for recurrent policy (LSTM) or short frame stack.

## Actions

- `action = [fc_frac ∈ [0,1], batt_cmd ∈ [-1,1]]`, shield enforces feasibility.

## Reward (minimal)

Per step with dt = 1 s:
- Economic cost: `c_h2·Δm_H2 + c_grid·max(-p_batt,0)·dt_h`
- Feasibility only via power accommodation (no distance/speed shaping):
  - Light tracking term on DC bus: `r_track = -λ_track·(P_unmet + P_waste)/P_scale`
    - `P_dem = P_req (+ aux, with traction/regen efficiencies)`
    - `P_sup = P_fc + P_batt_dis`
    - Penalize only unmet and true waste (do NOT penalize captured regen)
  - Optional: delay proxy `λ_delay·1[behind]·dt` (keep small or omit initially)
- Light regularization: `λ_smooth·||a_t − a_{t−1}||`

Notes
- Do not reward distance or speed directly. Strategy should emerge from costs + accommodating the driver’s power.
- Keep `λ_track` small (e.g., 1e−6…1e−5 with `P_scale≈p_fc_max_kw`) so economics dominate.
- Battery‑life terms (throughput/C‑rate) remain zero initially; we track them for selection, not training.

Leave out battery‑life terms initially. For the “battery‑life” variant, add tiny `λ_cycle` and optional `λ_crate` in a separate config.

## Robustness & Divergence Modeling

- Treat `P_req` as a forecast with error; expose forecast vs realized (v3) or include trend/history.
- Inject deviations from forecast: dwell jitter, additive/multiplicative demand noise, emergency brakes, surprise accelerations; occasional adhesion dips and envelope clamps.
- Train with domain randomization over mass, aux bias, timetable jitter, emergency rate.

Policy behaviors to expect
- Pre‑ramp FC guided by forecast; battery absorbs forecast error.
- Maintain SOC headroom proportional to uncertainty and proximity to stops to maximize regen capture during unexpected brakes.

## Metrics to Track (not in reward)

- €/km proxy, kg H₂/100 km, kWh charge/100 km.
- Unmet, clipped regen/waste, shield flags, FC action jerk.
- Battery stress: kWh throughput, max C‑rate, SOC std, rainflow cycles (offline).
- Tracking diagnostics: `P_unmet`, `P_waste`, captured regen vs friction.

## Phased Roadmap

1) Baseline EMS RL (clean schedule)
   - Use v1 observations, minimal reward.
   - Deterministic route; no emergencies; verify stability and cost KPIs.

2) Add preview + trend
   - Upgrade to v2 observations. Expect anticipatory FC pre‑ramp and SOC headroom before stops.

3) Forecast vs actual split
   - v3 observations; inject forecast error + rare emergencies. Target low unmet and low clipped regen under surprises.

4) Two policy profiles
   - OPEX‑min model (current reward weights).
   - Battery‑life model with small `λ_cycle`, `λ_crate` (selection by config or run tag).

5) Lifetime evaluation (offline)
   - Add evaluator to compute EFC, rainflow damage, projected life; select checkpoints on Pareto (€/km vs life).

6) Stress & generalization suite
   - Holdout scenarios: long downhill after high SOC, back‑to‑back brakes, adhesion dips.

## Intermediate Goals (Curriculum with Gates)

We progress through five capabilities. Each stage adds signals/randomization and has clear evaluation gates before advancing.

1) Accommodate Power (Feasibility)
- Setup: v1 observations, deterministic route, no emergencies.
- Reward: economics + small λ_track + small λ_smooth.
- Success criteria (gate):
  - P95 unmet ≤ 0.5% of peak demand; mean unmet ≤ 0.1%.
  - Stable power split (no chattering); minimal shield projections.

2) Cost Efficiency (Economics)
- Setup: same as (1); tune costs only.
- Reward: economics primary (H₂ + grid), λ_track small, λ_smooth small.
- Success criteria:
  - €/km proxy within X% of tuned baseline EMS; kg H₂/100km reduced vs naive.
  - No regressions on unmet gate from (1).

3) Anticipation of P_req Regimes
- Setup: add v2 preview channels (time‑to‑stop, grade/segment, trend).
- Randomization: mild timetable jitter and mass/aux drift.
- Reward: unchanged (minimal).
- Success criteria:
  - Lower unmet during rising‑demand segments; earlier FC pre‑ramp.
  - Higher regen capture near stops; reduced clipped regen vs (2).

4) Headroom for Emergencies (Robustness)
- Setup: v3 forecast vs actual (or keep v2 + trend/history); inject emergency brakes/accelerations (Poisson), adhesion dips.
- Reward: unchanged; λ_track remains small.
- Success criteria:
  - Under emergencies, unmet stays below threshold; clipped regen reduced by Y%.
  - SOC distribution shifts to maintain headroom before high‑uncertainty windows.

5) Battery Lifetime Maximization (Selection)
- Setup: continue training with minimal reward OR add tiny λ_cycle/λ_crate in a separate run.
- Logging: throughput, C‑rate, rainflow cycles; evaluate life offline.
- Success criteria:
  - Pareto improvement: maintain (2)–(4) cost/robustness gates while reducing EFC/cycle damage.
  - Select checkpoint by Pareto frontier (€/km vs life proxy).

### Experiment Timeline (suggested)
- Weeks 1–2: Goals (1) and (2) to convergence; establish baselines.
- Weeks 3–4: Goal (3) preview; ablations on which preview helps most.
- Weeks 5–6: Goal (4) robustness; stress test suite + CVaR analysis of unmet.
- Weeks 7–8: Goal (5) lifetime selection; finalize Pareto trade‑offs.

## Implementation TODOs

- [ ] Confirm obs include filtered `P_req` and last action (v1).
- [ ] Add v2 channels: `time_to_stop_norm`, grade/segment indicator, `dP_req_dt` trend (config‑gated).
- [ ] Add optional v3: `P_req_forecast` vs realized, plus an uncertainty cue.
- [ ] Keep reward minimal in `env0_env._compute_reward` (economics + small `λ_track` + small `λ_smooth`).
- [ ] Add `lambda_track` and `p_scale_kw` to config; compute `(P_unmet + P_waste)/P_scale` and include in reward.
- [ ] Surface `P_waste` in plant/state using regen friction/post‑aux signals; ensure it’s logged.
- [ ] Extend logger/info with: battery throughput (kWh), C‑rate peaks, clipped regen, `P_waste`, `P_unmet`, shield flags.
- [ ] Add `scripts/eval_lifetime.py` for rainflow cycles, EFC, and simple damage; output €/km vs life Pareto.
- [ ] Provide SB3 PPO/LSTM training script with two presets (opex_min, batt_life).

## Immediate Next Steps (High‑Value)

1) Implement λ_track + P_waste
- Add config keys: `scenario.reward_weights.lambda_track`, `scenario.reward_weights.p_scale_kw`.
- In reward, add `r_track = -lambda_track * (P_unmet + P_waste) / p_scale_kw` (exclude captured regen).

2) Add v2 observation channels
- `time_to_stop_norm`, `grade/segment`, `dP_req_dt` under `scenario.observations.*` with toggles.
- Update normalization and obs space accordingly.

3) Logging for lifetime and tracking
- Log per‑step: `|p_batt_kw|*dt_h`, `|p_batt_kw|/P_max`, SOC sample, `P_unmet`, `P_waste`, clipped regen.
- Aggregate: EFC, max C‑rate, SOC std in episode summary.

4) Lifetime evaluator
- New script `scripts/eval_lifetime.py`: compute rainflow on SOC trace, EFC, simple cycle damage; plot €/km vs life and select Pareto checkpoints.


---

