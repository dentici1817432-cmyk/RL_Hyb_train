# Hybrid FC–Battery Train — Env0 (Driver-in-the-Loop EMS) Technical Specification

This document lists **all systems and technical specs** needed to implement **Env0**: a Gymnasium-compatible simulation where the **driver/ATO dictates motion** (power request from speed control) and the **Energy Management System (EMS)** allocates power between **Fuel Cell (FC)** and **Battery**. The goal is to minimize **operational cost (€/km proxy)** while satisfying **safety constraints**.

---

## 1) System Architecture

**Blocks & Ownership**
- **Driver/ATO (exogenous):** Generates requested **wheel power** \(P_req(t)\) from timetable, speed limits, and route grade.
- **EMS Agent (learned):** Chooses **FC setpoint** and **Battery charge/discharge**. Cannot throttle/brake or alter the driver’s request.
- **Safety Shield (hard constraints):** Projects EMS actions into the feasible set (SOC corridor, C-rate caps, FC ramp, regen acceptance, adhesion).
- **Plant:**
  - **Train Dynamics:** Point-mass model (Davis resistances) + grade.
  - **Traction/Braking Envelopes:** Speed-dependent max traction/regen.
  - **Driveline & Aux:** DC bus, converter efficiencies, **Aux load ≈ 200 kW** + bias drift.
  - **Battery Subsystem:** SOC dynamics with η, power caps, corridor.
  - **Fuel Cell Subsystem:** Power setpoint with ramp; H₂ usage from efficiency & LHV.
  - **H₂ Storage:** Simple mass balance (tank level fraction).

**Data Flow per step (Δt = 1 s):**
1) Driver → \(P_req\) (kW) and dwell flag.
2) Plant computes DC-bus **demand** \(P_dem\) from \(P_req\) and auxiliaries.
3) EMS proposes actions \((fc\_frac, batt\_cmd)\).
4) Shield adjusts actions to **feasible** \((P_{FC}, P_{bat})\).
5) Plant updates **stores** (SOC, H₂), **speed/position**, and logs **unmet/wasted** power.

---

## 2) Gymnasium Environment

- **Observation space:** `Box(low=-1, high=1, shape=(12,), dtype=float32)`
  1. Speed \(v\) (m/s)
  2. Time/Distance to next stop (normalized)
  3. Grade preview / segment indicator (normalized)
  4. Requested power \(P_req\) (kW, LPF)
  5. Requested power trend (kW/s)
  6. Battery SOC estimate (noisy)
  7. Tank level (normalized proxy)
  8. Last FC fraction
  9. Last battery cmd
  10–12. Nuisance noise channels

- **Action space:** `Box(low=[0,-1], high=[1,1], dtype=float32)` → `[fc_frac, batt_cmd]`
  - `fc_frac` → \(P_{FC} = fc\_frac \cdot P^{max}_{FC}\)
  - `batt_cmd` → \(P_{bat} \in [-P^{max}_{chg}, +P^{max}_{dis}]\) (−=charge)

- **Episode:** 20–30 minutes (1,200–1,800 steps). Terminate on **SOC ≤ 0.15**, **tank ≤ 0.02**, or time-up.

---

## 3) Driver/ATO Specification

**3) MPC Driver (most precise, still lightweight)**

Receding-horizon speed planning using Model Predictive Control (MPC), which optimizes a short-horizon cost at each time step to generate smooth, energy-efficient, and punctual trajectories:

At each step, MPC solves:

\[
\min_{a_{t:t+H-1}} \sum_{k=t}^{t+H-1}
  \left[
    \alpha_1 (v_k - v_k^\star)^2
    + \alpha_2\,(\mathrm{power\_req}_k)^2
    + \alpha_3\,(\mathrm{brake\_req}_k)^2
  \right]
\]

**Subject to:**
- **Dynamics:**
  \[
  m\,\dot{v} =
      \frac{P_{\text{wheel}}}{\max(v,\,\varepsilon)}
      - (A+Bv+C v^2)
      - mg\sin\theta
  \]
  where \(m\) is vehicle mass, \(A,B,C\) are Davis params, \(\theta\) is grade, and \(\varepsilon\) avoids division by zero.

- **Speed limits:** 
  \[
  v_k \le v_{\max}(s_k)
  \]
  Also subject to **stop constraints**, **comfort (jerk/acc limits)**, and possible **adhesion** constraints.

- **Terminal constraint:**  
  The final time in the planning horizon must align with the scheduled arrival time (or maintain non-negative slack).

- **Input conversion:**  
  The planned acceleration \(a_k\) (or jerk/velocity profile) is unrolled; for each step, \(P_{\text{req}}\) is derived as
  \[
  P_{\text{req},k} = (m a_k + A + B v_k + C v_k^2 + m g \sin\theta) \cdot v_k
  \]

The MPC procedure re-solves this optimization every control step (typically each second), delivering:
- **Smooth and efficient speed trajectories**, favoring limited braking (high regen potential)
- **On-time arrivals** (by terminal constraint)
- **Tunable tradeoff** between timeliness, smoothness, and energy via \(\alpha_1, \alpha_2, \alpha_3\)

MPC achieves "nicest" trajectories — i.e., maximally smooth (good for regenerative recovery) and can make explicit tradeoffs (e.g., penalize braking or deviation from target). Such a driver is especially useful as a strong baseline or for ablation.




**Inputs:** Timetable, speed limits vs position, route grade profile, adhesion coefficient \(\mu\).

**Procedure per step:**
1. Compute **target speed** \(v^\star\) from timetable and limits.
2. Longitudinal control (e.g., PID) → requested wheel **force** \(\hat F\).
3. Convert to **requested wheel power**: \(P_{req} = \hat F \cdot v\) (kW).
4. Clamp by **adhesion**: \(|\hat F| \le \mu m g\).
5. Clamp by **traction/regen envelopes** (speed dependent).
6. **Dwell**: force \(P_{req} \approx 0\) for dwell duration (± jitter).
7. Add small randomness to emulate human variability.

**Config (examples):**
- Stops per episode: 2–3; dwell mean 60 s; jitter ±20%.
- Adhesion \(\mu\): sample per episode (e.g., 0.20–0.35).
- Grade profile: piecewise constant segments (e.g., +2%, 0%, −1%).
- Limits: speed caps per segment.

---

## 4) Plant & Dynamics

### 4.1 Driveline and DC Bus
- **Auxiliaries:** \(P_{aux} = 200\,\text{kW} + b_t\) (bias \(b_t\) = random walk).
- **Driveline efficiency:** traction \(\eta_{drv,tr}\) (e.g., 0.85), regen \(\eta_{drv,rg}\) (e.g., 0.75).
- **DC demand:**
  \[
  P_{dem} =
  \begin{cases}
  \dfrac{P_{req}}{\eta_{drv,tr}} + P_{aux}, & P_{req}\ge 0\\[4pt]
  \eta_{drv,rg}\,P_{req} + P_{aux}, & P_{req}< 0
  \end{cases}
  \]
  (Regen \(P_{req}<0\) is negative.)

### 4.2 Battery Subsystem
- **Power limits:** \(P^{max}_{dis}\), \(P^{max}_{chg}\) (asymmetric to reflect charge bottlenecks).
- **Energy:** \(E_{batt}\) (kWh).
- **Efficiencies:** \(\eta_{dis}\), \(\eta_{chg}\).
- **SOC update (coulomb counting):**
  \[
  SOC_{t+1} = SOC_t
  - \frac{\max(P_{bat},0)}{E_{batt}\,\eta_{dis}}\Delta t_h
  + \frac{\max(-P_{bat},0)\,\eta_{chg}}{E_{batt}}\Delta t_h
  \]
- **Corridor:** soft [0.20, 0.90]; **hard trip** at 0.15.

### 4.3 Fuel Cell Subsystem
- **Max power:** \(P^{max}_{FC}\).
- **Ramp:** \(|\Delta P_{FC}| \le r_{FC}\) kW/s; optional min-up/min-down.
- **Efficiency (Env0):** fixed \(\eta_{FC}\).
- **H₂ usage:** \(\Delta m_{H2} = \dfrac{P_{FC}}{\eta_{FC}\,LHV_{H2}}\Delta t_h\), with \(LHV_{H2}=33.3\,\text{kWh/kg}\).

### 4.4 H₂ Storage
- **Tank mass capacity:** \(M_{tank}\) (kg).
- **Level:** \(tank_{t+1} = tank_t - \Delta m_{H2}\).
- **Trip if:** tank fraction ≤ 0.02.

### 4.5 Traction / Regen Envelopes
- **Max traction power:** \(P^{max}_{trac}(v)\) (curve or constant).
- **Max regen power:** \(P^{max}_{regen}(v)\) (≤ traction).
- **Adhesion-limited force:** \(|F| \le \mu m g\).
- **Regen acceptance:** limited further by SOC high and battery charge cap.

### 4.6 Train Dynamics (Point-Mass)
- **Davis resistances:** \(F_r = A + Bv + Cv^2\).
- **Grade:** \(\theta(s)\); component \(m g \sin\theta \approx m g \cdot \text{grade}\).
- **Wheel power delivered:** \(P_{wheel} = P_{req} - P_{unmet}\).
- **Force from power:** \(F_{wheel} = \frac{P_{wheel}}{\max(v,\epsilon)}\) (sign-aware).
- **State update:** \(m\dot v = F_{wheel} - F_r - m g \sin\theta\); integrate \(v, s\).

---

## 5) Safety Shield (Hard Constraints)

- **FC ramp:** enforce \(|\Delta P_{FC}| \le r_{FC}\).
- **SOC corridor:** project \(P_{bat}\) to avoid discharging below 0.20 or charging above 0.90.
- **C-rate caps:** optionally via current limit; practically enforce via \(P^{max}_{dis/chg}\).
- **Regen clipping:** if \(P_{req}<0\) and SOC high / charge cap reached → clip accepted regen; track **clipped_regen**.
- **DC power feasibility:** If \(P_{FC} + P_{bat}^+ < P_{dem}\) → define **unmet**. If \(P_{FC}+P_{bat}^+ > P_{dem}\) and \(P_{bat}^-\) is saturated or SOC high → **wasted**.
- **Adhesion/Envelope:** ensure driver’s request is clamped before EMS (the driver block), but expose flags when caps bind.

---

## 6) Reward and Costing

**Per-step reward (Δt = 1 s):**
\[
r = -\Big(
c_{H2}\,\Delta m_{H2}
+ c_{grid}\,\Delta E_{chg}
+ \lambda_{smooth}\,\|a_t-a_{t-1}\|
+ \lambda_{unmet}\,\frac{P_{unmet}}{P_{scale}}
+ \lambda_{waste}\,\frac{P_{waste}}{P_{scale}}
+ \lambda_{delay}\,\Delta t\cdot \mathbf{1}_{behind}
\Big)
\]

**Notes**
- \(\Delta E_{chg}\) counts **net charging** energy only.
- **Delay** arises from dynamics (if \(P_{unmet}\) slows the train vs the timetable).
- \(P_{scale}\) ≈ \(P^{max}_{FC}\) (for numerical stability).
- **Objective:** minimize OPEX proxy while respecting constraints (violations avoided by shield).

---

## 7) Observations (Normalization & Noise)

- Normalize each channel to roughly \([-1,1]\) using fixed ranges.
- Add small Gaussian noise to **SOC** and **tank** to reflect estimation error.
- Include **history handling** via a recurrent policy (recommended) or stack last actions.

---

## 8) Randomization & Curriculum

**Per-episode random draws:**
- Passenger mass \(m\) (e.g., 180–260 t).
- Aux bias RW std (start small; increase with curriculum).
- Adhesion \(\mu\) (weather).
- Initial SOC (0.55–0.80), Tank level (0.70–1.00).
- Route pick (flat/rolling/mild-mountain), dwell jitter.

**Curriculum phases:**
1. Easy: narrow mass, high adhesion, low aux drift, minimal regen.
2. Medium: full mass range, moderate \(\mu\) variation, regen enabled.
3. Robust: larger drifts, aggressive regen caps, schedule tightness.

---

## 9) Renderer (Recommended)

**Panels:**
- **Speed vs Target** with stop windows & limits (delay overlay).
- **Power Flow:** \(P_{req}\), \(P_{FC}\), \(P_{bat}^{+/-}\), **unmet**, **clipped regen**, **wasted**.
- **Stores:** SOC (with corridor band), Tank level.
- **Costs/Reward:** per-second cost breakdown; cumulative €/km proxy.
- **Flags:** SOC_LOW/HIGH, FC_RAMP, C_RATE, REGEN_CLIP, ADHESION_CAP.

Modes: `human`, `rgb_array`, `ansi`. Refresh every N steps.

---

## 10) Configuration (Keys & Defaults)

```yaml
sim:
  dt_s: 1
  steps_min: 1200
  steps_max: 1800
  seed: 123

route:
  profiles: [flat, rolling, mild_mountain]
  grade_segments_percent: [0.02, 0.00, -0.01]
  segment_duration_s: 600
  speed_limits_mps: [40, 35, 30]

stops:
  count: 3
  dwell_mean_s: 60
  dwell_jitter_frac: 0.2

driver:
  pid: {kp: 5.0, ki: 0.0, kd: 1.5}
  adhesion_mu: {min: 0.2, max: 0.35}
  traction_power_max_kw: 1000.0
  regen_power_max_kw: 400.0
  req_lpf_tau_s: 3.0
  trend_window_s: 3

driveline:
  eta_traction: 0.85
  eta_regen: 0.75
  aux_base_kw: 200.0
  aux_bias_rw_sigma_kw: 0.1

train:
  mass_tons_min: 180.0
  mass_tons_max: 260.0
  davis_A_N: 5000.0
  davis_B_N_per_mps: 100.0
  davis_C_N_per_mps2: 5.0

battery:
  energy_kwh: 300.0
  p_discharge_max_kw: 600.0
  p_charge_max_kw: 300.0
  eta_dis: 0.94
  eta_chg: 0.95
  soc_init_min: 0.55
  soc_init_max: 0.80
  soc_soft_min: 0.20
  soc_soft_max: 0.90
  soc_hard_min: 0.15

fuel_cell:
  p_max_kw: 400.0
  eta_fc: 0.50
  ramp_kw_per_s: 40.0
  h2_lhv_kwh_per_kg: 33.3
  tank_capacity_kg: 50.0
  tank_init_min: 0.70
  tank_init_max: 1.00
  tank_hard_min_frac: 0.02

shield:
  enforce_fc_ramp: true
  enforce_soc_corridor: true
  enforce_crate_caps: true
  enable_regen_clipping: true

costs:
  h2_eur_per_kg: 6.0
  grid_eur_per_kwh: 0.18

reward:
  lambda_smooth: 0.01
  lambda_unmet: 1.0e-6
  lambda_waste: 1.0e-6
  lambda_delay: 0.5
  p_scale_kw: 400.0

observations:
  include_last_action: true
  include_noise_channels: 3
  normalize_to_unit_box: true

renderer:
  enabled: true
  mode: human
  render_every: 5
  rolling_window_s: 180
