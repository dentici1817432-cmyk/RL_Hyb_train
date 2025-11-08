# Baseline EMS (from the PDFs) — Rule-Based Power Split for FC–Battery Train

This file distills the **rule-based EMS** described in the two PDFs into a concrete, buildable baseline.
It reacts to the **driver/ATO power request** and allocates power between **Fuel Cell (FC)** and **Battery**,
respecting SOC and device limits.

---

## 0) Shared assumptions

- **Driver/ATO** computes traction/regen request; EMS **does not** change the motion plan.
- **Auxiliaries:** add ~**200 kW** to the DC bus demand. :contentReference[oaicite:0]{index=0}  
- **SOC corridor:** battery considered **usable above 20%**; regen accepted **only if SOC < 90%** (otherwise energy is **dumped**). :contentReference[oaicite:1]{index=1}  
- **FC operating bands (sizing context):** a nominal band around \(P_{\mathrm{nom,FC}}\) (deliverable uses **90–105%**; Energies uses **95–105%**), with a **max/boost** up to **110–120%** for exceptional charging/demand. 

---

## 1) Two canonical baselines (pick one)

We provide **Baseline A (Deliverable)** and **Baseline B (Energies)** exactly as written in the PDFs.  
Main difference: **whether/when the FC is used to *charge* the battery**.

### Baseline A — “FC at nominal, charges when possible” (Deliverable)

**Intent:** Keep FC at **nominal power** during motion; battery covers peaks; **FC also charges battery** whenever demand is below nominal (subject to SOC). At stations, FC goes to **aux-only idle**, **unless SOC is low**, in which case FC remains at nominal to recharge. 

**Logic (step t)**  
Inputs: \(P_{\text{req}}\) (wheel), \(P_{\text{aux}}\), SOC, device states.

1) **Sanity:** If FC or tank **offline** → **battery supplies all** until **SOC ≤ 20%**, then mission fails/stop. :contentReference[oaicite:4]{index=4}  
2) **Station dwell?**  
   - If **dwell** and SOC **> low_S** (cfg): set \(P_{FC}=P_{\text{aux}}\) (idle). :contentReference[oaicite:5]{index=5}  
   - If **dwell** and SOC **≤ low_S**: keep \(P_{FC}=P_{\mathrm{nom}}\) to **recharge**. (The deliverable notes this is energetically sub-optimal but ensures SOC headroom.) :contentReference[oaicite:6]{index=6}
3) **In motion:**  
   - Set \(P_{FC}=P_{\mathrm{nom}}\) (within nominal band limits/ramp). :contentReference[oaicite:7]{index=7}  
   - **If** \(P_{\text{dem}} > P_{FC}\): battery **discharges** the delta (if SOC>20%). :contentReference[oaicite:8]{index=8}  
   - **Else** (\(P_{\text{dem}} < P_{FC}\)): use FC **surplus** to **charge** battery (until SOC→90% or charge cap hit). :contentReference[oaicite:9]{index=9}
4) **Regen:** accept only if SOC<90%; otherwise **dump** (log clipped). :contentReference[oaicite:10]{index=10}
5) **Ramps/limits:** enforce FC ramp & battery C-rate/power caps.

> Notes from text: FC **two fixed levels** (nominal vs **aux-only standby**), avoiding FC ON/OFF; SOC thresholds configurable (example in deliverable shows **50%** as a station-recharge trigger in a large-battery case). :contentReference[oaicite:11]{index=11}

---

### Baseline B — “FC near nominal band; *no charging* except SOC<30%” (Energies)

**Intent:** Keep FC within **95–105%** of nominal when demand is inside band; otherwise **idle** and let battery work. **Never charge** battery from FC **except** when SOC is **below 30%**, where FC goes up to **110%** to recharge when possible. :contentReference[oaicite:12]{index=12}

**Logic (step t)**  
1) If \(P_{\text{dem}}\) **≥** \(P_{\text{upper,FC}} = 1.05\,P_{\mathrm{nom}}\): set \(P_{FC}=P_{\text{upper,FC}}\); battery supplies the rest (if SOC>20%). :contentReference[oaicite:13]{index=13}  
2) If \(P_{\text{lower,FC}} \le P_{\text{dem}} < P_{\text{upper,FC}}\): **battery OFF**; \(P_{FC}\) alone supplies load in band (hysteresis to avoid toggling). :contentReference[oaicite:14]{index=14}  
3) If \(P_{\text{dem}} < P_{\text{lower,FC}} = 0.95\,P_{\mathrm{nom}}\): set \(P_{FC}=P_{\text{idling,FC}}\); **battery covers** the motor/aux load **if SOC allows**. **No FC charging** in this regime. :contentReference[oaicite:15]{index=15}  
4) **Low SOC exception:** if **SOC < 30%**, force \(P_{FC}=\min(1.10\,P_{\mathrm{nom}}, P_{FC}^{\max})\) and allow **FC→battery charging** when demand permits. :contentReference[oaicite:16]{index=16}  
5) **Regen:** battery charges from regen only (SOC<90%); otherwise **dump**. :contentReference[oaicite:17]{index=17}  
6) **Ramps/limits:** enforce FC ramp, battery caps.

---

## 2) Build parameters (pull straight from the PDFs)

| Parameter | Value / Rule | Source |
|---|---|---|
| **SOC_min (usable)** | 20% (battery offline below this) | :contentReference[oaicite:18]{index=18} |
| **SOC_regen_cap** | 90% (dump above) | :contentReference[oaicite:19]{index=19} |
| **FC band (deliverable sizing)** | 90–105% of \(P_{\mathrm{nom}}\); **max** 120% | :contentReference[oaicite:20]{index=20} |
| **FC band (Energies EMS)** | 95–105% of \(P_{\mathrm{nom}}\); **max** 110% | :contentReference[oaicite:21]{index=21} |
| **FC at stations (Deliverable)** | Idle at **aux-only**; **stay at nominal** if SOC below a set “low” threshold (example 50% in a case study) | :contentReference[oaicite:22]{index=22} |
| **Charging from FC** | **Allowed broadly** in Deliverable; **forbidden** except when **SOC<30%** in Energies |  |
| **Aux load** | ~200 kW baseline (added to demand) | :contentReference[oaicite:24]{index=24} |

---

## 3) Pseudocode (drop-in)

### Common prelude
```text
observe P_req, P_aux≈200kW, SOC, tank, device states
P_dem = bus_demand(P_req, P_aux, driveline_eta)

if FC_offline or tank_empty:
    use battery until SOC<=0.20 → stop   # safety/mission end

Baseline A:
if dwell:
    if SOC <= SOC_low_station:      # e.g., 0.50 in deliverable example
        P_FC = P_nom                # recharge at station
    else:
        P_FC = P_aux                # aux-only idle
else:
    P_FC = P_nom (ramp-limited within 90–105% band)

if P_dem > P_FC:
    P_batt = +min(P_dem - P_FC, P_batt_dis_max) if SOC>0.20 else 0
else:
    # surplus FC → charge battery (until SOC<0.90 & charge cap)
    P_batt = -min(P_FC - P_dem, P_batt_chg_max) if SOC<0.90 else 0

apply regen rule: if P_req<0 and SOC>=0.90 → dump/clipped_regen
