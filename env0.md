# Env0 — Supervisory EMS (POMDP) for Hybrid FC–Battery Train

A lightweight, partially-observable Gymnasium environment to train a **supervisory Energy Management System (EMS)** that splits traction demand between **Fuel Cell (FC)** and **Battery**. It mirrors the structure and assumptions of the project PDFs while remaining fast for iteration. Degradation/REPLEX and detailed physics are deferred to later envs.

---

## 1) Interface

- **Observation:** `Box(low=-1, high=1, shape=(10,), dtype=float32)`
- **Action:** `Box(low=[0,-1], high=[1,1], dtype=float32)` → `[fc_frac, batt_cmd]`
- **Step time:** 1 s; **Episode length:** 1,200–1,800 steps (20–30 min)

### Observation layout (normalized)
1. speed (m/s)
2. SOC estimate (noisy)
3. H₂ tank level proxy (noisy, normalized)
4. timetable phase or distance-to-next-stop proxy
5. requested traction power `P_req` (kW), lightly low-pass filtered
6. last action: fc_frac
7. last action: batt_cmd
8–10. nuisance noise channels (white noise)

> No direct exposure of passenger mass or auxiliary bias → **POMDP**.

### Action layout
- `fc_frac ∈ [0,1]` → `P_fc = fc_frac * P_fc_max`
- `batt_cmd ∈ [-1,1]` → maps to `P_batt ∈ [-P_batt_max_chg, +P_batt_max_dis]` (charge − / discharge +)

---

## 2) Roles

- **Driver/ATO (exogenous):** Emits requested traction/regen power `P_req` following a reference speed profile with 2–3 stops, grade segments, and mild randomness. When a `manual_speed_profile` is provided, the driver uses a jerk-limited PID tracker and can interpret the profile as continuous ramps (`speed_profile_interpolation: linear`) so velocity transitions stay smooth; segments can opt to `hold: true` for pure dwell phases.
- **EMS agent (learned):** Chooses FC setpoint fraction and battery charge/discharge command.
- **Shield (hard safety):** Enforces SOC corridor, C-rate caps, and FC ramp/min-up/down heuristics.

---

## 3) Plant & dynamics (fast, simplified)

All quantities use **Δt = 1 s**.

**Hidden per episode**
- Passenger mass `m` ~ Uniform[180 t, 260 t] (constant within an episode; hidden).
- Auxiliary bias `b_t` follows a random walk around 0 kW.

**Constants (defaults, configurable)**
- Base auxiliaries: **200 kW**
- `P_fc_max`: 400 kW
- `P_batt_max_dis`: 600 kW
- `P_batt_max_chg`: 300 kW
- Battery energy `E_batt`: 300 kWh
- Tank capacity (H₂ mass) `M_tank`: 50 kg (toy capacity for 20–30 min sims)
- Efficiencies: `η_fc = 0.50`, `η_chg = 0.95`, `η_dis = 0.94`
- H₂ LHV: **33.3 kWh/kg**
- FC ramp limit: `r_fc = 40 kW/s`
- SOC corridor: `[0.20, 0.90]`
- Speed cap: `v_max = 50 m/s`
- Kinematic gain: `k_v = 2.5e-6 (m/s)/W` (tune so flats reach ~30–40 m/s)

**Per-step computation**
1) **Demand:**  
   `P_aux = 200 kW + b_t`  
   `P_dem = P_req + P_aux`

2) **Actions → pre-shield supply:**  
   `P_fc_cmd = fc_frac * P_fc_max`  
   `P_batt_cmd ∈ [-P_batt_max_chg, +P_batt_max_dis]`

3) **Shield (apply hard constraints):**
   - FC ramp: `|P_fc(t) - P_fc(t-1)| ≤ r_fc`
   - SOC corridor rules:
     - if `SOC < 0.20` → restrict discharge, bias to charge
     - if `SOC > 0.90` → restrict charge, clip regen
   - C-rate caps via `P_batt_max_(chg/dis)`

4) **Power balance & unmet demand:**  
   Let `P_batt_plus = max(P_batt, 0)`  
   `residual = P_dem - (P_fc + P_batt_plus)`  
   `P_unmet = max(residual, 0)`  
   If `residual < 0`, excess attempts to **charge** the battery (bounded by `P_batt_max_chg` and SOC constraints).

5) **SOC update (coulomb counting):**  
   `SOC_{t+1} = SOC_t
      - (max(P_batt,0) / (E_batt * η_dis)) * Δt_hours
      + (max(-P_batt,0) * η_chg / E_batt) * Δt_hours`  
   Clamp to `[0,1]`.

6) **H₂ consumption:**  
   `Δm_H2 = (P_fc / (η_fc * LHV_H2)) * Δt_hours`  
   `tank_{t+1} = tank_t - Δm_H2 / M_tank`

7) **Kinematics (toy):**  
   `P_delivered = P_fc + max(P_batt,0) - P_aux`  
   `v_{t+1} = clip(v_t + k_v * (P_delivered - P_loss) * 1.0, 0, v_max)`  
   `P_loss` can be a small constant (e.g., 200 kW) to emulate baseline drag.  
   Stops/dwells are imposed by the driver schedule (speed forced to 0 during dwell).

**Stochastic drifts**
- `b_t` RW: `b_{t+1} = b_t + ε`, `ε ~ N(0, σ_b^2)`  
- Dwell time jitter ±20% around nominal.

---

## 4) Reward (cost-aligned, minimal shaping)

Per step:
r = -(
c_h2 * Δm_H2

c_grid * ΔE_charge_kWh

λ_smooth * ||a_t - a_{t-1}||

λ_delay * Δt * 1[behind_schedule]
) - λ_unmet * P_unmet

- `ΔE_charge_kWh` counts only **net charging** energy (battery sink from grid).
- Keep shaping light: the goal is OPEX + schedule adherence.
- Typical costs: `c_h2 = 6 €/kg`, `c_grid = 0.18 €/kWh`.

---

## 5) Episode logic

- **Duration:** 1,200–1,800 steps.
- **Stops:** 2–3; dwell time ±20%.
- **Termination:** `SOC ≤ 0.15` or `tank ≤ 0.02` or time-up.

---

## 6) Randomization & curriculum

Per episode:
- Passenger mass `m` ~ U[180 t, 260 t]
- Aux bias RW std `σ_b` (start small, then increase)
- Initial SOC ~ U[0.55, 0.80]; initial tank ~ U[0.70, 1.00]
- Timetable jitter ±5–10%

Curriculum:
1) narrow mass/low drift/no delays  
2) full ranges/moderate drift/some delays  
3) higher drift + occasional regen caps

---

## 7) Baselines & KPIs

**Baselines**
- Rule-based EMS: FC near nominal; battery covers peaks; FC charges only at low SOC; idle at stations.

**KPIs**
- €/km (OPEX proxy), kg H₂/100 km, kWh charged/100 km
- Delay seconds/trip (95th percentile)
- Constraint violations (target: 0)
- FC action jerk (smoothness)
- Battery energy throughput
- End-of-trip SOC and tank levels

---

## 8) Configuration

See `config.yaml` for all knobs:
- `sim`: horizons, dt, seeds
- `driver`: stop layout, grades, jitter
- `plant`: constants (loss term, kinematic gain, caps)
- `battery`: capacities and efficiencies
- `fuel_cell`: power/efficiency/ramp
- `shield`: SOC corridor, C-rates, ramp
- `costs`: `c_h2`, `c_grid`
- `reward_weights`: `λ_smooth`, `λ_delay`, `λ_unmet`
- `randomization`: ranges for mass, SOC/tank init, aux drift
- `logging`: which traces to record

---

## 9) Quickstart (implementation guidance)

1) Parse `config.yaml`.
2) Build Gymnasium env with the interface above.
3) Implement the **shield** as a post-processor on actions.
4) Add a scripted **driver** that outputs `P_req` from a simple speed profile with stops and grades.
5) Verify:
   - With random actions + shield → **zero hard violations**.
   - With the rule-based baseline → plausible SOC/tank trajectories.
6) Train with a **recurrent PPO/SAC** implementation (PufferLib or similar).
