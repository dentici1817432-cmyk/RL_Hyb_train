# EMS Logic Review

## Overview
This document reviews the Energy Management System (EMS) logic flow and identifies potential issues affecting speed tracking.

## EMS Flow Architecture

```
Driver → P_req (traction power request)
   ↓
EMS/Agent → Action [fc_frac, batt_cmd]
   ↓
Shield → ShieldedAction [p_fc_kw, p_batt_kw] (constrained)
   ↓
Plant → Power Balance & Speed Update
```

---

## Component Analysis

### 1. Driver (`driver.py`)
**Role:** Generates requested traction power `P_req` (kW)

**Current Status:** 
- ✅ Reference mode implemented (smooth P_req without PID/jitter)
- ⚠️ Normal mode uses PID control which can cause oscillations

**Key Output:** `P_req` (kW) - can be positive (traction) or negative (regen)

---

### 2. Baseline EMS (`rl_hyb_train/policies/baseline.py`)

**Strategy:** Battery-only mode (FC disabled in current config)

**Logic Flow:**

```python
# Step 1: Calculate total demand
p_demand_estimate = p_req_kw + p_aux_base  # Traction + Aux

# Step 2: FC is disabled (0 kW)
fc_frac = 0.0
p_fc_available = 0.0

# Step 3: Battery must meet all demand
p_battery_needed = p_demand_estimate

# Step 4: Convert to battery command
if p_battery_needed > 0:
    batt_cmd = p_battery_needed / p_batt_max_dis  # [0, 1]
elif p_battery_needed < 0:  # Regen case
    regen_available = abs(p_battery_needed)
    if soc < 0.85:
        batt_cmd = -regen_available / p_batt_max_chg  # [-1, 0]
    else:
        batt_cmd = -0.1  # Minimal charge
```

**Issues Identified:**

1. **❌ CRITICAL: Regen handling logic error**
   - When `P_req < 0` (regen), `p_demand_estimate = P_req + P_aux` can be negative
   - Example: `P_req = -500 kW`, `P_aux = 200 kW` → `p_demand = -300 kW`
   - EMS correctly identifies this as regen case
   - **BUT:** The battery command calculation assumes all regen can be used for charging
   - **Problem:** If regen is less than aux (e.g., `P_req = -100 kW`, `P_aux = 200 kW`), then `p_demand = +100 kW` (still positive!), so battery should DISCHARGE to provide aux, not charge

2. **⚠️ Unmet demand compensation is minimal**
   - Only adds 0.05 to batt_cmd if unmet > 0.1 kW
   - May not be sufficient for large unmet demands

3. **⚠️ No consideration of battery efficiency**
   - Battery discharge efficiency (0.94) not accounted for in power calculations
   - If EMS commands 1000 kW discharge, actual power delivered is ~940 kW

---

### 3. Shield (`shield.py`)

**Role:** Applies hard safety constraints

**Constraints Applied:**
- FC ramp limits
- SOC corridor (soft min/max)
- SOC hard limits (emergency)
- C-rate caps
- Tank level checks

**Key Logic:**
```python
# Convert actions to power
p_fc_cmd_kw = fc_frac * p_fc_max_kw
p_batt_cmd_kw = batt_cmd * (p_max_chg if batt_cmd < 0 else p_max_dis)

# Apply constraints...
# Returns ShieldedAction with constrained powers
```

**Issues Identified:**

1. **✅ Shield logic appears correct** - properly constrains actions

---

### 4. Plant (`plant.py`)

**Role:** Executes power balance and updates speed

**Power Balance Logic:**

```python
# Total demand
p_dem_kw = p_req_kw + p_aux_kw  # Traction + Aux

# Power balance
p_batt_plus = max(p_batt_kw, 0.0)  # Only discharge contributes
residual = p_dem_kw - (p_fc_kw + p_batt_plus)

# Unmet demand
p_unmet_kw = max(residual, 0.0)

# Speed update
p_delivered_kw = p_fc_kw + p_batt_plus - p_aux_kw
p_net_kw = p_delivered_kw - p_loss_kw
v_delta = kinematic_gain * p_net_kw * 1000.0 * dt
speed_mps = clip(speed + v_delta, 0, v_max)
```

**Issues Identified:**

1. **❌ CRITICAL: Power balance logic error**
   - Line 98: `residual = p_dem_kw - (p_fc_kw + p_batt_plus)`
   - This calculates unmet demand correctly
   - **BUT:** Line 125: `p_delivered_kw = p_fc_kw + p_batt_plus - p_aux_kw`
   - **Problem:** This subtracts `p_aux_kw` from delivered power, but aux is already part of demand!
   - **Correct logic should be:**
     - `p_delivered_traction = p_fc_kw + p_batt_plus - p_aux_kw` ✓ (this is correct)
     - `p_net_traction = p_delivered_traction - p_loss_kw`
     - Speed should update based on net traction power
   - **However:** The issue is that `p_unmet_kw` doesn't affect speed update!
   - If there's unmet demand, speed won't accelerate properly

2. **❌ CRITICAL: Unmet demand not reflected in speed**
   - When `p_unmet_kw > 0`, the plant doesn't deliver enough power
   - But speed update uses `p_delivered_kw` which doesn't account for unmet
   - **Should be:** `p_delivered_kw = p_fc_kw + p_batt_plus - p_aux_kw - p_unmet_kw`
   - Or: `p_delivered_kw = p_req_kw - p_unmet_kw` (simpler!)

3. **⚠️ Battery efficiency not applied to power balance**
   - Battery discharge efficiency (0.94) is only used for SOC update
   - But actual power delivered from battery should be `p_batt_kw * eta_discharge`
   - Currently: If battery commands 1000 kW discharge, plant assumes 1000 kW delivered
   - **Should be:** `p_batt_delivered = p_batt_plus * eta_discharge`

4. **⚠️ Regen power handling**
   - When `P_req < 0` (regen), battery charging doesn't contribute to traction
   - This is handled correctly (only `p_batt_plus` used)
   - But regen braking should affect speed directly
   - **Current:** Speed update only uses positive power (traction)
   - **Missing:** Regen braking should decelerate the train

---

## Critical Issues Summary

### Issue #1: Unmet Demand Not Affecting Speed ⚠️ CRITICAL
**Location:** `plant.py` line 125

**Problem:** 
- Speed update uses `p_delivered_kw = p_fc_kw + p_batt_plus - p_aux_kw`
- But if there's unmet demand (`p_unmet_kw > 0`), actual delivered power is less
- Speed won't accelerate properly when demand exceeds supply

**Fix:**
```python
# Option 1: Subtract unmet from delivered
p_delivered_kw = p_fc_kw + p_batt_plus - p_aux_kw - p_unmet_kw

# Option 2: Use P_req directly (simpler)
p_delivered_traction = p_req_kw - p_unmet_kw  # Actual traction delivered
p_delivered_kw = p_delivered_traction - p_aux_kw  # Net after aux
```

### Issue #2: Battery Efficiency Not Applied ⚠️ HIGH PRIORITY
**Location:** `plant.py` line 125

**Problem:**
- Battery discharge efficiency (0.94) only affects SOC, not power delivery
- EMS commands 1000 kW, but only 940 kW actually delivered

**Fix:**
```python
p_batt_delivered = p_batt_plus * self.battery_config.eta_discharge
p_delivered_kw = p_fc_kw + p_batt_delivered - p_aux_kw
```

### Issue #3: Regen Braking Not Affecting Speed ⚠️ MEDIUM PRIORITY
**Location:** `plant.py` line 125-135

**Problem:**
- When `P_req < 0` (regen), speed update doesn't account for braking
- Regen should decelerate the train

**Fix:**
```python
# Handle regen braking
if p_req_kw < 0:
    # Regen braking: decelerate based on regen power
    p_regen_kw = abs(p_req_kw)
    # Convert regen to deceleration (simplified)
    p_net_kw = -p_regen_kw - p_loss_kw  # Negative = braking
else:
    # Normal traction
    p_delivered_kw = p_fc_kw + p_batt_plus - p_aux_kw
    p_net_kw = p_delivered_kw - p_loss_kw
```

### Issue #4: EMS Regen Logic Error ⚠️ MEDIUM PRIORITY
**Location:** `rl_hyb_train/policies/baseline.py` line 90-104

**Problem:**
- When `P_req` is negative but magnitude < `P_aux`, demand is still positive
- EMS should discharge battery to provide aux, not charge

**Fix:**
```python
if p_req_kw < 0:
    # Regen case
    regen_power = abs(p_req_kw)
    if regen_power > p_aux_base:
        # Excess regen: charge battery
        excess_regen = regen_power - p_aux_base
        batt_cmd = -excess_regen / p_batt_max_chg
    else:
        # Less regen than aux: need battery to provide aux
        aux_needed = p_aux_base - regen_power
        batt_cmd = aux_needed / p_batt_max_dis
else:
    # Normal traction: discharge to meet demand
    batt_cmd = p_demand_estimate / p_batt_max_dis
```

---

## Recommended Fixes Priority

1. **🔴 CRITICAL:** Fix unmet demand in speed update (Issue #1)
2. **🟠 HIGH:** Apply battery efficiency to power delivery (Issue #2)
3. **🟡 MEDIUM:** Fix EMS regen logic (Issue #4)
4. **🟡 MEDIUM:** Add regen braking to speed update (Issue #3)

---

## Testing Recommendations

After fixes:
1. Test with reference mode (smooth P_req) to verify speed tracking
2. Test with high demand scenarios (P_req > battery capacity)
3. Test regen scenarios (negative P_req)
4. Verify battery efficiency affects power delivery
5. Check that unmet demand reduces speed acceleration
