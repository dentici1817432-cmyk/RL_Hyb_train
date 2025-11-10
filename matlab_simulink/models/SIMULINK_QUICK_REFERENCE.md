# Simulink Model Quick Reference

## Building the Model

```matlab
cd models
build_simulink_model()   % Creates hybrid_train_ems.slx
open_system('hybrid_train_ems')  % Opens the model
```

## Block Diagram Structure

The Simulink model `hybrid_train_ems.slx` contains these subsystems:

### Top-Level Blocks

```
┌─────────────────────────────────────────────────────────┐
│                  HYBRID TRAIN EMS MODEL                  │
├─────────────────────────────────────────────────────────┤
│                                                           │
│   [Driver] ──(P_req, grade, is_dwell)──┐                │
│                                          │                │
│   [Aux_Load] ──(P_aux)──┐              │                │
│                          │              │                │
│   [EMS_Controller] <──(SOC, P_req)─────┤                │
│        │                                 │                │
│        └──(fc_frac, batt_cmd)──┐       │                │
│                                  │       │                │
│   [Safety_Shield] <──(states)───┼───────┤                │
│        │                         │       │                │
│        └──(P_FC, P_batt)────┐  │       │                │
│                              │  │       │                │
│   [Regen_Flow] <──(P_req)───┴──┤       │                │
│        │                         │       │                │
│        └──(regen captured)──────┤       │                │
│                                  │       │                │
│   [Power_Flow] <──(sources)─────┴───────┤                │
│        │                                 │                │
│        └──(P_delivered, P_unmet)────────┤                │
│                                          │                │
│   [Train_Dynamics] <──(P_delivered)─────┤                │
│        │                                 │                │
│        └──(speed, distance, accel)──────┤                │
│                                          │                │
│   [Battery] <──(P_batt)──────────────────┤                │
│        │                                 │                │
│        └──(SOC)──────────────────────────┘                │
│                                                           │
│   [Fuel_Cell] <──(P_FC)                                  │
│        │                                                  │
│        └──(tank_level, H2_consumed)                      │
│                                                           │
│   [Scopes] <──(all signals)                              │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

### Subsystem Details

| Subsystem | Inputs | Outputs | Purpose |
|-----------|--------|---------|---------|
| **Driver** | time | P_req, grade, is_dwell | Generate power demand profile |
| **Aux_Load** | time, aux_bias | P_aux | Auxiliary systems load |
| **EMS_Controller** | SOC, P_req, speed | fc_frac, batt_cmd | Energy management strategy |
| **Safety_Shield** | fc_frac, batt_cmd, states | P_FC, P_batt | Enforce constraints |
| **Regen_Flow** | P_req, speed, P_aux, charge_cmd | regen_captured | Regenerative braking |
| **Power_Flow** | P_req, P_FC, P_batt, P_aux | P_delivered, P_unmet | DC bus power balance |
| **Battery** | P_batt | SOC, delta_SOC | Battery state of charge |
| **Fuel_Cell** | P_FC | tank_level, H2_kg | Hydrogen consumption |
| **Train_Dynamics** | P_delivered, grade, speed | speed, distance, accel | Vehicle motion |
| **Scopes** | various | (display) | Signal visualization |

## How to Implement Each Block

Each subsystem should contain a **MATLAB Function block** with the implementation from:
- `functions/simple_driver.m` → Driver subsystem
- `functions/baseline_ems.m` → EMS_Controller subsystem
- `functions/shield_constraints.m` → Safety_Shield subsystem
- `functions/regen_flow.m` → Regen_Flow subsystem
- `functions/power_flow.m` → Power_Flow subsystem
- `functions/soc_update.m` → Battery subsystem
- `functions/h2_consumption.m` → Fuel_Cell subsystem
- `functions/train_dynamics.m` → Train_Dynamics subsystem

## Detailed Instructions

See `MODEL_STRUCTURE_GUIDE.txt` for:
- Step-by-step implementation of each subsystem
- Complete MATLAB Function code for each block
- Signal connections and data types
- Solver configuration
- Running and validating the model

## Quick Commands

```matlab
% Build model
build_simulink_model()

% Open model
open_system('hybrid_train_ems')

% Run simulation
sim('hybrid_train_ems')

% Load initial conditions
params = initialize_model();
```

## Tips

1. **Start with the top-level view** - See overall structure
2. **Double-click each subsystem** - View internal blocks
3. **Right-click → Look Under Mask** - If subsystem is masked
4. **Add MATLAB Function blocks** - Copy from `functions/*.m` files
5. **Connect signals** - Match colors: control (red), power (blue), state (green)
6. **Set solver** - Fixed-step, 1.0 second time step
7. **Compare with MATLAB** - Use `run_simulation()` as reference

## Already Complete Alternative

If you just want to run simulations without building Simulink blocks, use:

```matlab
cd ..  % Back to matlab_simulink/
results = run_simulation();
```

The pure MATLAB version is **fully functional** and produces identical results!

---

**Last Updated**: 2025-11-10
