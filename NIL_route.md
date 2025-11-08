# NIL Route (North Italy Line) — Technical Sheet

> Extracted from the project deliverable and the Energies paper. Use this as a canonical route file for Env0 driver-in-the-loop simulations.

---

## 1) Overview

- **Name:** North Italy Line (NIL), “Piemonte short mountain line”  
- **Length:** **16.5 km** one-way.  
- **Stations:** **7** (including terminals).  
- **Dwell policy:** **120 s** at intermediate stations; **300 s** at terminal.  
- **Speed limits:** **90 km/h** from 0 to **7.81 km**; **60 km/h** from **7.81–16.5 km**.  
- **Average gradient:** **9‰**; **Max gradient:** **23.8‰**. 

### Station positions (meters from origin)

`0, 3470, 5290, 7810, 10200, 13820, 16500` (origin → terminal). :contentReference[oaicite:1]{index=1}

---

## 2) Geometry & Limits (distance-based)

| Segment | s_from (m) | s_to (m) | v_max (km/h) | Notes |
|---|---:|---:|---:|---|
| S1 | 0 | 7 810 | 90 | Uphill/variable grade portions present. |
| S2 | 7 810 | 16 500 | 60 | Includes descents enabling regen. |

- Elevation/grade vary by distance; overall **short mountainous** profile (see elevation–distance figure in the paper). :contentReference[oaicite:2]{index=2}

---

## 3) Stops & Dwell

| Stop # | Position (m) | Dwell (s) |
|---:|---:|---:|
| 1 | 0 | 300 (terminal) |
| 2 | 3 470 | 120 |
| 3 | 5 290 | 120 |
| 4 | 7 810 | 120 |
| 5 | 10 200 | 120 |
| 6 | 13 820 | 120 |
| 7 | 16 500 | 300 (terminal) |

Dwell assumptions match the PDF simulation setup. :contentReference[oaicite:3]{index=3}

---

## 4) Reference kinematic & resistance notes (for driver/plant)

- The deliverable validates kinematics against the above speed limits; **acc ≤ 0.78 m/s²**, **dec ∈ [0.7, 0.8] m/s²**. Curves obey limits along NIL.   
- Resistance modeling uses **Davis** equation; grade resistance follows the route’s slope profile. :contentReference[oaicite:5]{index=5}

> For Env0, you can use your project’s standard Davis coefficients and comfort limits; the PDFs provide the framework and show typical compliance behavior over NIL.

---

## 5) Powertrain context from PDFs (for consistency)

- **Auxiliaries:** assume **~200 kW** baseline in the energy balance. :contentReference[oaicite:6]{index=6}  
- **Motor nominal mechanical power (for clipping/plots):** **1435 kW** (used in kinematic/power figures). :contentReference[oaicite:7]{index=7}

(These values are not route properties per se, but they are the context used in the NIL simulations.)

---

## 6) YAML — Route Spec v1 (ready to load)

```yaml
version: 1
metadata:
  id: "NIL-16k5"
  name: "North Italy Line (Piemonte, short mountain)"
  source: ["Deliverable_PNRR_2_2_3_Def.pdf", "energies-18-05457-v2.pdf"]
  notes: "Short mountainous route; 7 stations; 90/60 km/h segments."
track:
  length_m: 16500
  davis:
    # Use your project defaults or tuned values; PDFs validate with Davis modeling.
    A_N: 5000
    B_N_per_mps: 100
    C_N_per_mps2: 5
segments:
  - { s_from_m: 0,     s_to_m: 7810,  grade_percent: null, v_max_mps: 25.0 }   # 90 km/h
  - { s_from_m: 7810,  s_to_m: 16500, grade_percent: null, v_max_mps: 16.67 } # 60 km/h
stops:
  - { name: "Origin",    s_m: 0,     dwell_seconds_mean: 300, dwell_jitter_frac: 0.0 }
  - { name: "S2",        s_m: 3470,  dwell_seconds_mean: 120, dwell_jitter_frac: 0.2 }
  - { name: "S3",        s_m: 5290,  dwell_seconds_mean: 120, dwell_jitter_frac: 0.2 }
  - { name: "S4",        s_m: 7810,  dwell_seconds_mean: 120, dwell_jitter_frac: 0.2 }
  - { name: "S5",        s_m: 10200, dwell_seconds_mean: 120, dwell_jitter_frac: 0.2 }
  - { name: "S6",        s_m: 13820, dwell_seconds_mean: 120, dwell_jitter_frac: 0.2 }
  - { name: "Terminal",  s_m: 16500, dwell_seconds_mean: 300, dwell_jitter_frac: 0.0 }
timetable:
  schedule_type: "end_to_end"
  # Choose based on your sim pacing; PDFs don’t give a single fixed number here.
  total_time_s:  # e.g. 1500
  slack_policy: { min_buffer_s: 20, distribute: "proportional_to_length" }
envelopes:
  # Provide your platform’s traction/regen vs speed curves here.
  traction_power_max_kw_by_speed: [[0, 600], [20, 1000], [40, 1000]]
  regen_power_max_kw_by_speed:    [[0, 100], [20,  400], [40,  400]]
  accel_limit_mps2: 0.78
  decel_limit_mps2: 0.8
  jerk_limit_mps3: 0.6
adhesion:
  default_mu: 0.30
  variability: { per_episode_uniform: [0.20, 0.35] }
auxiliaries:
  base_power_kw: 200
  overrides: []
randomization:
  dwell_jitter_enable: true
route_stats:
  stations: 7
  avg_gradient_per_mille: 9.0
  max_gradient_per_mille: 23.8
