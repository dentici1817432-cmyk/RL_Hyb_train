# Agents Guide — RL Hybrid Train Environment

This document helps AI agents understand and work with the RL Hybrid Train codebase for physics-focused simulation and EMS policy development.

---

## Quick Start

```python
from rl_hyb_train import make_env
from pathlib import Path

# Create environment from config file
env = make_env(Path("conf.yaml"), seed=42)

# Standard Gymnasium interface
obs, info = env.reset()
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

---

## Project Structure

```
rl_hyb_train/
├── __init__.py              # Exports: Config, Env0, make_env()
├── config.py                # Configuration dataclasses and YAML loader
├── env0_env.py              # Main Gymnasium environment (Env0)
├── driver.py                # Full driver/ATO (timetable-based)
├── driver_simple.py         # Simple segment-based P_req generator
├── plant.py                 # Plant dynamics (power balance, SOC, H2, kinematics)
├── shield.py                # Safety shield (constraint enforcement)
├── powerflow.py             # Pure physics functions (power flow, SOC delta, H2)
├── plotting.py              # Centralized plotting utilities
└── policies/                # EMS policy implementations
    ├── __init__.py          # Policy exports
    ├── README.md            # **Policy development guide**
    ├── base.py              # Policy protocol/interface
    ├── baseline.py          # Simple FC load-follower
    ├── baseline_ems.py      # FC-at-nominal strategy
    ├── balanced.py          # SOC-aware balancing
    ├── scenario.py          # Route-preview aware
    ├── mpc.py               # Model Predictive Control
    └── rl.py                # RL policy wrapper

scripts/
├── demo_simple_driver_baseline.py   # Segment-based demo
├── render_simple_driver.py          # Visualization demo
└── train_rl_step1.py                # RL training entry point

test_ems_with_p_req.py     # **Physics test harness** (synthetic P_req curves)
conf.yaml                  # Configuration file (all parameters)
main.py                    # Smoke test / example usage
```

---

## Core Components

### 1. Configuration (`config.py`)

All parameters are defined in `conf.yaml` and loaded via `Config.from_yaml()`.

**Key config sections:**
- `train.*`: Physical model parameters (`plant`, `battery`, `fuel_cell`, `shield`, `costs`)
- `scenario.*`: Episode definition (`sim`, `driver`, `reward_weights`, `randomization`, `observations`, `renderer`, `logging`)
- `policy.*`: Default EMS choice plus tuning knobs per policy (optional)

**Usage:**
```python
from rl_hyb_train import Config
config = Config.from_yaml("conf.yaml")
# Access: config.battery.e_batt_kwh, config.costs.c_h2_eur_per_kg, etc.
```

### 2. Environment (`env0_env.py`)

**Class:** `Env0(gym.Env)`

**Spaces:**
- **Observation**: `Box(-1, 1, (12,), float32)` — 12 normalized channels
- **Action**: `Box([0,-1], [1,1], (2,), float32)` — `[fc_frac, batt_cmd]`

**Key methods:**
- `reset(seed=None, options=None)` → `(obs, info)`
- `step(action)` → `(obs, reward, terminated, truncated, info)`

**Info dict contains:**
- `soc`, `tank_level`, `speed_mps`, `p_req_kw`, `p_fc_kw`, `p_batt_kw`
- `p_unmet_kw`, `distance_km`, `constraint_violations`, `step`

### 3. Drivers (`driver.py`, `driver_simple.py`)

**Driver (full timetable)**
Generates requested traction power `P_req` (kW) from a filtered speed profile with:
- Multiple stops and grade segments
- PID/preview controller, dwell/grade jitter
- Low-pass filtering and rate limits before handing demand to the EMS

**SimplePReqDriver**
A lightweight segment-based generator (dwell → accelerate → cruise → climb → brake, etc.) defined in `driver_simple.py`.
- Enabled automatically when `scenario.driver.manual_p_req_profile` is non-empty.
- Adds loss bias, smoothing, rate limiting, and small noise.
- Useful for deterministic RL training curricula and reproducible policy visualizations.

Both drivers emit `P_req_kw` via `step(dt)`; the environment chooses which implementation to instantiate based on the config.

### 4. Plant (`plant.py`)

**Class:** `Plant`

Simulates power balance and dynamics:
- SOC updates (coulomb counting with efficiency)
- H2 consumption (from FC power and efficiency)
- Speed/kinematics (simplified train dynamics)
- Power balance (demand vs supply with unmet tracking)
- Regenerative braking allocation (aux loads, battery charging, friction brakes)

**Key method:** `step(p_fc_kw, p_batt_kw, p_req_kw, dt_seconds, p_loss_kw=200.0)` → `PlantState`

**PlantState attributes** (fully documented with categories):
- Energy storage: `soc`, `tank_level`
- Motion: `speed_mps`, `distance_km`
- Power commands: `p_fc_kw`, `p_batt_kw`
- Power balance: `p_dem_kw`, `p_supply_kw`, `p_delivered_kw`, `p_unmet_kw`
- Battery detail: `p_batt_discharge_kw`, `p_batt_charge_kw`, `p_batt_delivered_kw`, `p_batt_charge_regen_kw`, `p_batt_charge_fc_kw`
- Braking: `p_brake_total_kw`, `p_brake_regen_kw`, `p_brake_aux_kw`, `p_brake_friction_kw`
- Aux: `aux_bias_kw`

See `plant.py:17` for complete docstring.

### 5. Power Flow Functions (`powerflow.py`)

Pure physics functions for power balance calculations:
- `resolve_battery_flow()`: Split battery command into discharge/charge with efficiency
- `compute_regen_flow()`: Regenerative braking allocation
- `fc_excess_power()`: FC power available for battery charging
- `allocate_battery_charging()`: Battery charging from regen + FC excess
- `compute_power_balance()`: DC bus balance accounting
- `soc_delta()`: SOC change from discharge/charge
- `h2_consumption_kg()`: H2 consumption from FC power

See `powerflow.py:1` for detailed module documentation and examples.

### 6. Shield (`shield.py`)

**Class:** `Shield`

Applies hard safety constraints:
- FC ramp limits (40 kW/s default)
- SOC corridor enforcement (0.20–0.90 default)
- C-rate caps (charge/discharge limits)
- Tank level checks

**Key method:** `apply(fc_frac, batt_cmd, soc, tank_level)` → `ShieldedAction`

**Returns:** `ShieldedAction(p_fc_kw, p_batt_kw, violations)`

---

## Observation Space Details

**12-dimensional normalized vector** `[-1, 1]`:

| Index | Description | Normalization |
|-------|-------------|---------------|
| 0 | Speed (m/s) | `2 * (speed / v_max) - 1` |
| 1 | SOC estimate (noisy) | `2 * (soc + noise) - 1` |
| 2 | H2 tank level (noisy) | `2 * (tank + noise) - 1` |
| 3 | Time to next stop | `2 * (time_to_stop / horizon) - 1` |
| 4 | P_req filtered (kW) | `2 * (p_req / p_req_max) - 1` |
| 5 | P_req trend | `2 * (trend / trend_scale) - 1` |
| 6 | Last action: fc_frac | `2 * fc_frac - 1` (remapped to [-1,1]) |
| 7 | Last action: batt_cmd | Already `[-1, 1]` |
| 8-11 | Nuisance noise | `N(0, noise_std)` |

**Note:** SOC and tank have additive Gaussian noise (configurable std: `soc_noise_std`, `tank_noise_std`).

---

## Action Space Details

**2-dimensional continuous action:**

- `action[0]` (`fc_frac`): `[0, 1]` → FC power fraction
  - Maps to: `P_fc = fc_frac * P_fc_max` (default: 400 kW max)

- `action[1]` (`batt_cmd`): `[-1, 1]` → Battery command
  - `> 0`: Discharge, maps to `[0, P_batt_max_dis]` (default: 600 kW max)
  - `< 0`: Charge, maps to `[-P_batt_max_chg, 0]` (default: 300 kW max)

**Shield applies:** Actions are automatically constrained by the shield before execution.

**Power conventions:**
- Battery power: **Positive = discharge, Negative = charge**
- FC power: Always non-negative (0 ≤ p_fc ≤ p_fc_max)
- P_req: **Positive = traction, Negative = regen braking**

---

## Reward Function

Per-step reward:

```
r = -(c_h2 * Δm_H2 + c_grid * ΔE_charge_kWh + λ_smooth * ||a_t - a_{t-1}|| + λ_delay * Δt * 1[behind_schedule])
    - λ_unmet * P_unmet
```

**Components:**
- **H2 cost:** `c_h2_eur_per_kg * Δm_H2` (default: 6.0 €/kg)
- **Grid cost:** `c_grid_eur_per_kwh * ΔE_charge_kWh` (default: 0.18 €/kWh)
- **Smoothness penalty:** `lambda_smooth * ||action - last_action||` (default: 0.01)
- **Delay penalty:** `lambda_delay * dt * indicator` (default: 0.5)
- **Unmet demand penalty:** `lambda_unmet * P_unmet` (default: 1e-6)

**Note:** Rewards are negative (costs), so higher is better for the agent.

---

## Episode Termination

**Terminated (hard stop):**
- `SOC <= soc_hard_min` (default: 0.15)
- `tank_level <= tank_hard_min` (default: 0.02)

**Truncated (time limit):**
- `episode_step >= episode_length` (random: 1800–2400 steps by default)

---

## Randomization

Per episode (from config):
- **Passenger mass:** `U[180, 260]` tons (hidden, affects dynamics)
- **Initial SOC:** `U[0.55, 0.80]` (if `soc_init_uniform: true`)
- **Initial tank:** `U[0.70, 1.00]` (if `tank_init_uniform: true`)
- **Auxiliary bias:** Random walk with `σ_b` (default: randomization.aux_bias_rw_sigma_kw)
- **Timetable jitter:** ±20% (if `timetable_jitter_enable: true`)

---

## Testing & Development Workflow

### 1. Physics Verification (`test_ems_with_p_req.py`)

**Purpose:** Test EMS logic and train physics with synthetic P_req curves, bypassing the driver.

**Built-in scenarios:**
- `steady_cruise`: 200 kW constant
- `step_changes`: 0→400→200→600→100 kW steps
- `ramp_profile`: Linear ramps up and down
- `regen_profile`: Mixed accel/regen cycles
- `nil_like_profile`: Multi-station mission
- `emergency_braking`: High braking power (tests friction brakes)
- `high_soc_regen`: Long descent with battery near full (tests regen rejection)

**Usage:**
```bash
# Test single scenario
python test_ems_with_p_req.py --scenario step_changes

# Test all scenarios
python test_ems_with_p_req.py --scenario all

# Use with your custom policy
# Edit test_ems_with_p_req.py line 168 to use your policy
```

**What it validates:**
- ✓ SOC corridor compliance (20-90%)
- ✓ FC ramp rate limits (40 kW/s)
- ✓ Battery C-rate caps
- ✓ Power balance (P_fc + P_batt_delivered + P_unmet = P_req + P_aux)
- ✓ Energy conservation
- ✓ Regenerative braking breakdown (aux, battery, friction)

**Output:** Auto-saved plots (`test_ems_<scenario>.png`) showing:
- Power split (stacked: FC, Battery, P_req)
- Power balance (demand vs supply)
- SOC and H2 tank trajectories
- Train speed evolution
- EMS actions (before/after shield)
- Braking power breakdown

### 2. Policy Development (`rl_hyb_train/policies/`)

**See `rl_hyb_train/policies/README.md` for complete guide.**

**Quick template:**
```python
# policies/my_custom_ems.py
class MyCustomEMS:
    def __init__(self, config: Config):
        self.config = config
        # Extract needed parameters

    def reset(self) -> None:
        # Reset internal state
        pass

    def act(self, obs: np.ndarray, info: Optional[Dict] = None) -> np.ndarray:
        # Return [fc_frac, batt_cmd]
        return np.array([fc_frac, batt_cmd], dtype=np.float32)
```

**Available baseline policies:**
| Policy | Strategy | File |
|--------|----------|------|
| `BaselineEMS` | Simple FC load-follower | `baseline.py` |
| `BaselineNominalEMS` | FC at 90% nominal, battery handles peaks | `baseline_ems.py` |
| `BalancedEMS` | SOC-aware balancing with target SOC | `balanced.py` |
| `ScenarioAwareEMS` | Uses timetable/route preview | `scenario.py` |
| `MPCEms` | Model Predictive Control | `mpc.py` |
| `RLEMS` | RL policy wrapper | `rl.py` |

### 3. Visualization & Plotting (`rl_hyb_train/plotting.py`)

**Centralized plotting function:**
```python
from rl_hyb_train.plotting import plot_history

# After running simulation with EMSTestHarness
history = harness.get_history_arrays()
fig = plot_history(history, scenario_name="my_test")
plt.savefig("my_test.png")
```

**Produces consistent multi-panel plots:**
- Power split (stacked)
- Demand vs supply comparison
- SOC with corridor limits
- H2 tank level
- Train speed
- EMS actions (raw + shielded)

---

## Common Tasks

### Load and use environment

```python
from rl_hyb_train import make_env
from pathlib import Path

env = make_env(Path("conf.yaml"), seed=42)
obs, info = env.reset()

for step in range(1000):
    action = env.action_space.sample()  # Replace with agent
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, info = env.reset()
```

### Access configuration

```python
from rl_hyb_train import Config

config = Config.from_yaml("conf.yaml")
print(f"FC max power: {config.fuel_cell.p_fc_max_kw} kW")
print(f"Battery capacity: {config.battery.e_batt_kwh} kWh")
print(f"H2 cost: {config.costs.c_h2_eur_per_kg} €/kg")
```

### Modify configuration programmatically

```python
from rl_hyb_train import Config, Env0

config = Config.from_yaml("conf.yaml")
config.costs.c_h2_eur_per_kg = 8.0  # Change H2 cost
config.battery.e_batt_kwh = 400.0   # Change battery capacity

env = Env0(config, seed=42)
```

### Monitor episode metrics

```python
obs, info = env.reset()
episode_reward = 0.0

for step in range(1000):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    episode_reward += reward

    # Access info dict
    print(f"SOC: {info['soc']:.3f}, Tank: {info['tank_level']:.3f}")
    print(f"FC: {info['p_fc_kw']:.1f} kW, Batt: {info['p_batt_kw']:.1f} kW")
    print(f"Unmet: {info['p_unmet_kw']:.1f} kW, Violations: {info['constraint_violations']}")

    if terminated or truncated:
        print(f"Episode reward: {episode_reward:.4f}")
        obs, info = env.reset()
        episode_reward = 0.0
```

### Test custom EMS policy

```python
from rl_hyb_train import Config, Env0
from rl_hyb_train.policies import BaselineEMS  # Or your custom policy

config = Config.from_yaml("conf.yaml")
env = Env0(config, seed=42)
policy = BaselineEMS(config)

obs, info = env.reset()
policy.reset()

for step in range(100):
    action = policy.act(obs, info)
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        break
```

---

## Extending the Codebase

### Adding new config parameters

1. Add field to appropriate dataclass in `config.py`:
```python
@dataclass
class BatteryConfig:
    # ... existing fields ...
    new_param: float = 100.0  # Default value
```

2. Add to `conf.yaml`:
```yaml
battery:
  # ... existing fields ...
  new_param: 100.0
```

3. Use in code:
```python
value = self.config.battery.new_param
```

### Adding new observation channels

1. Update observation space shape in `env0_env.py`:
```python
self.observation_space = spaces.Box(
    low=-1.0, high=1.0, shape=(13,), dtype=np.float32  # Changed from 12
)
```

2. Add to `_get_observation()`:
```python
obs = np.zeros(13, dtype=np.float32)
# ... existing channels ...
obs[12] = new_channel_value  # Normalized to [-1, 1]
```

### Adding new reward components

1. Add weight to `RewardWeightsConfig` in `config.py`
2. Add to `conf.yaml` under `reward_weights`
3. Compute in `_compute_reward()` in `env0_env.py`

### Adding new termination conditions

Modify termination check in `step()`:
```python
terminated = (
    self.plant.state.soc <= self.config.battery.soc_hard_min or
    self.plant.state.tank_level <= self.config.fuel_cell.tank_hard_min or
    new_condition  # Add here
)
```

---

## Code Quality & Best Practices

### Recent Improvements
- ✅ **Deprecation fixes**: Updated `np.trapz` → `np.trapezoid`
- ✅ **Power balance verification**: Corrected test harness to account for efficiency and unmet demand
- ✅ **Documentation**: Added comprehensive docstrings to `PlantState` and `powerflow` module
- ✅ **Policy guide**: Created `policies/README.md` with templates and examples
- ✅ **Config cleanup**: Removed duplicate `manual_speed_profile` sections

### Modularity
- **Plant**: Pure physics, no EMS logic
- **Shield**: Pure constraint enforcement, no policy logic
- **Policies**: Implement `act(obs, info)` interface, no direct plant access
- **Powerflow**: Pure functions for power balance calculations
- **Config**: Single source of truth for all parameters

### Testing Pipeline
1. **Unit tests**: Physics functions in `powerflow.py` (pure, testable)
2. **Integration tests**: `test_ems_with_p_req.py` (synthetic scenarios)
3. **Policy comparison**: Same test harness with different policies
4. **Visualization**: Consistent plots via `plotting.py`

---

## Important Notes

1. **POMDP:** Passenger mass and auxiliary bias are hidden (not in observations)
2. **Shield:** Actions are automatically constrained; violations are tracked in info
3. **Time step:** Fixed at 1 second (`dt_seconds: 1`)
4. **Normalization:** All observations are normalized to `[-1, 1]`
5. **Config-driven:** All parameters come from `conf.yaml` (no hardcoded values)
6. **Episode length:** Random per episode (1800–2400 steps by default)
7. **Power conventions:** Battery positive = discharge, P_req positive = traction
8. **Test before deploy:** Always run `test_ems_with_p_req.py` after changes

---

## Dependencies

- `gymnasium>=1.2.2` — Gym interface
- `numpy>=2.3.4` — Numerical operations
- `pyyaml>=6.0.3` — Config loading
- `matplotlib>=3.7.0` — Plotting (for test harness)

---

## File Reference

### Documentation
- **`AGENTS.md`**: This file (quick reference for AI agents)
- **`RL_COMBINED_AGENT.md`**: RL training strategy and iteration plan
- **`rl_hyb_train/policies/README.md`**: Policy development guide
- **`conf.yaml`**: All configuration parameters

### Code
- **`rl_hyb_train/plant.py`**: Plant dynamics (see `PlantState` docstring at line 17)
- **`rl_hyb_train/powerflow.py`**: Pure physics functions (see module docstring at line 1)
- **`rl_hyb_train/plotting.py`**: Visualization utilities
- **`test_ems_with_p_req.py`**: Physics test harness

### Examples
- **`main.py`**: Basic smoke test
- **`scripts/demo_simple_driver_baseline.py`**: Segment-based demo
- **`scripts/render_simple_driver.py`**: Visualization demo

---

## Quick Command Reference

```bash
# Run physics tests
python test_ems_with_p_req.py --scenario all

# Test single scenario
python test_ems_with_p_req.py --scenario step_changes

# Smoke test
uv run python main.py

# Run demo with simple driver
python scripts/demo_simple_driver_baseline.py

# Run RL training (step 1)
python scripts/train_rl_step1.py
```

---

## Questions?

Check in order:
1. **`rl_hyb_train/policies/README.md`** — Policy development guide
2. **`conf.yaml`** — All configurable parameters
3. **`powerflow.py:1`** — Power flow physics documentation
4. **`plant.py:17`** — PlantState field documentation
5. **`test_ems_with_p_req.py`** — Test harness usage
6. **Docstrings in source code** — API details

---

**Happy coding!** This codebase is optimized for rapid iteration on EMS policies with physics-grounded simulation. 🚂⚡
