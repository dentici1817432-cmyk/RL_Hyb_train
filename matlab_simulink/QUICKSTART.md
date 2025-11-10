# Quick Start Guide - MATLAB/Simulink Hybrid Train EMS

## For MATLAB Online Users

### Step 1: Upload to MATLAB Online
1. Download/zip the `matlab_simulink` folder
2. Go to [MATLAB Online](https://matlab.mathworks.com/)
3. Upload the folder using the Upload button

### Step 2: Run the Simulation
```matlab
% Navigate to the uploaded folder
cd matlab_simulink

% Run simulation (this will take ~30-60 seconds)
results = run_simulation();
```

That's it! The simulation will run and automatically generate plots showing:
- Power flows (FC, battery, demand)
- State variables (SOC, H2 tank, speed)
- Energy balance
- Constraint violations

## For Desktop MATLAB Users

### Quick Run (Pure MATLAB - No Simulink Required)
```matlab
cd path/to/matlab_simulink
results = run_simulation();
```

### About Simulink Model (NOT RECOMMENDED)

⚠️ **The Simulink model is INCOMPLETE and requires 4-8 hours of manual implementation.**

The `hybrid_train_ems.slx` file contains only:
- Empty subsystem blocks
- No code inside blocks
- No connections between blocks

**Why?** Creating functional Simulink models programmatically is extremely difficult. Simulink is designed for manual graphical modeling.

**See `ABOUT_SIMULINK.md` for full explanation.**

**Recommendation**: Just use `run_simulation()` - it's complete and works perfectly! ✅

### Run Validation Tests
```matlab
cd tests
test_steady_cruise();      % Test 1: Constant cruise
test_step_changes();       % Test 2: Power steps
test_regen_profile();      % Test 3: Regen braking
```

## Understanding the Results

After running, you'll see:

### Console Output
```
=== Hybrid Train EMS Simulation Initialized ===
Episode duration: 750.0 s (12.5 min)
Initial SOC: 0.450
Initial tank level: 0.950
Train mass: 220.5 tons
===============================================

Running simulation for 750.0 seconds...
  Progress: 10%
  Progress: 20%
  ...
  Progress: 100%

=== SIMULATION SUMMARY ===
Final SOC: 0.425
Final tank level: 0.890
Total distance: 8.5 km
Average P_FC: 45.2 kW
...
```

### Plots Generated
- **Figure 1: Power Flows** - Shows how power is distributed
- **Figure 2: State Variables** - SOC, tank, speed, distance trajectories
- **Figure 3: Actions** - Controller decisions (fc_frac, batt_cmd)
- **Figure 4: Constraints** - Safety shield activations
- **Figure 5: Energy Balance** - Cumulative energy flows
- **Figure 6: Operating Points** - Battery SOC vs power phase portrait

## Customizing Your Simulation

### Change Initial Conditions
```matlab
params = default_params();
params.battery.soc_init_min = 0.60;  % Start at 60% SOC
params.fc.tank_init_min = 1.00;      % Full H2 tank
results = run_simulation(params);
```

### Create Custom Driving Profile
```matlab
params = default_params();
params.driver.segments = {
    [60,  'dwell',      0,    0.0];    % Stop for 60s
    [120, 'accelerate', 400,  0.0];    % Accelerate
    [300, 'cruise',     300,  0.0];    % Cruise at 300 kW
    [90,  'brake',      -500, 0.0];    % Brake (regen)
    [60,  'dwell',      0,    0.0];    % Stop
};
results = run_simulation(params);
```

### Change System Parameters
```matlab
params = default_params();
params.fc.p_max_kw = 150;              % Larger fuel cell
params.battery.e_batt_kwh = 300;       % Larger battery
params.train.mass_tons_min = 200;      % Heavier train
params.train.mass_tons_max = 200;
results = run_simulation(params);
```

## File Organization

```
matlab_simulink/
├── run_simulation.m        ← START HERE (main entry point)
├── README.md               ← Full documentation
├── QUICKSTART.md           ← This file
│
├── config/                 ← Parameters
│   └── default_params.m    ← Modify this to change settings
│
├── functions/              ← Core simulation logic
│   ├── baseline_ems.m      ← Replace with your own controller
│   └── ...
│
├── tests/                  ← Validation tests
│   ├── test_*.m
│   └── compare_with_python.m
│
└── plotting/
    └── plot_results.m      ← Visualization
```

## Common Tasks

### Save Results to File
```matlab
results = run_simulation();
save('my_simulation_results.mat', 'results');
```

### Load and Re-plot
```matlab
load('my_simulation_results.mat');
plot_results(results);
```

### Export Plots as Images
```matlab
results = run_simulation();
addpath('plotting');
plot_results(results, true);  % Saves PNG files
```

### Access Specific Data
```matlab
results = run_simulation();

% Get final SOC
final_soc = results.soc(end);

% Get average FC power
avg_fc_power = mean(results.p_fc);

% Get total distance
total_distance = results.distance(end);

% Find max battery discharge
max_discharge = max(results.p_batt);

% Calculate total H2 consumed
h2_consumed_kg = (results.tank_level(1) - results.tank_level(end)) * 50;  % 50 kg tank
```

## Next Steps

### Beginner
1. Run `run_simulation()` with defaults
2. Explore the generated plots
3. Modify initial SOC and re-run
4. Try different driver profiles

### Intermediate
1. Run all validation tests in `tests/`
2. Modify parameters in `config/default_params.m`
3. Create custom driving scenarios
4. Analyze energy efficiency

### Advanced
1. Implement your own EMS controller in `functions/baseline_ems.m`
2. Add new test scenarios
3. Build the Simulink model using `models/build_simulink_model.m`
4. Compare with Python implementation using `tests/compare_with_python.m`

## Troubleshooting

### "Cannot find function"
Make sure you're in the `matlab_simulink/` directory:
```matlab
pwd  % Check current directory
cd path/to/matlab_simulink  % Navigate if needed
```

### Simulation runs but no plots appear
Manually call plotting:
```matlab
results = run_simulation();
addpath('plotting');
plot_results(results);
```

### "Out of memory" error
Reduce simulation duration:
```matlab
params = default_params();
params.sim.episode_steps_max = 600;  % Shorter episode
results = run_simulation(params);
```

## Getting Help

- **Full documentation**: See `README.md`
- **Parameter reference**: See `config/default_params.m`
- **Example usage**: See scripts in `tests/`
- **Simulink guide**: See `models/MODEL_STRUCTURE_GUIDE.txt`

## What's Simulated?

This is a **hybrid fuel cell-battery train** simulator featuring:

**Physical System:**
- 200 kWh battery (3000 kW discharge, 1000 kW charge)
- 100 kW fuel cell with 50 kg H2 tank
- 180-260 ton train (randomized)
- Realistic train dynamics (Davis resistance equation)

**Control System:**
- Energy Management System (EMS) controller
- Safety shield (SOC protection, FC ramp limits)
- Power flow coordinator
- Regenerative braking

**Scenarios:**
- Variable power demand profiles
- Acceleration, cruise, braking, dwelling
- Grade variations (hills and descents)
- Random auxiliary loads

The goal is to **minimize operational cost** (H2 + electricity) while maintaining:
- SOC within safe bounds [20%, 90%]
- All power demands met
- Fuel cell ramp rate limits

---

**Ready to simulate?** Just run:
```matlab
cd matlab_simulink
results = run_simulation();
```

Enjoy! 🚂⚡
