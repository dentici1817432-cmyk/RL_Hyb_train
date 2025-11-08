# Agents Guide — RL Hybrid Train Environment

This document helps AI agents understand and work with the RL Hybrid Train codebase.

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
├── __init__.py          # Exports: Config, Env0, make_env()
├── config.py            # Configuration dataclasses and YAML loader
├── env0_env.py          # Main Gymnasium environment (Env0)
├── driver.py            # Driver/ATO module (generates P_req)
├── plant.py             # Plant dynamics (power balance, SOC, H2, kinematics)
└── shield.py            # Safety shield (constraint enforcement)

conf.yaml                # Configuration file (all parameters)
main.py                  # Smoke test / example usage
env0.md                  # Detailed specification document
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
- Observation: `Box(-1, 1, (10,), float32)` — 10 normalized channels
- Action: `Box([0,-1], [1,1], (2,), float32)` — `[fc_frac, batt_cmd]`

**Key methods:**
- `reset(seed=None, options=None)` → `(obs, info)`
- `step(action)` → `(obs, reward, terminated, truncated, info)`

**Info dict contains:**
- `soc`, `tank_level`, `speed_mps`, `p_req_kw`, `p_fc_kw`, `p_batt_kw`
- `p_unmet_kw`, `distance_km`, `constraint_violations`, `step`

### 3. Driver (`driver.py`)

**Class:** `Driver`

Generates requested traction power `P_req` (kW) from a speed profile with:
- Multiple stops (configurable count)
- Grade segments (uphill/downhill)
- Timetable jitter (±10% by default)
- Dwell time jitter (±20% by default)

**Key method:** `step(dt)` → `P_req_kw`

### 4. Plant (`plant.py`)

**Class:** `Plant`

Simulates power balance and dynamics:
- SOC updates (coulomb counting)
- H2 consumption (from FC power)
- Speed/kinematics (toy model)
- Power balance (demand vs supply)

**Key method:** `step(p_fc_kw, p_batt_kw, p_req_kw, dt_seconds, p_loss_kw)` → `PlantState`

**State attributes:**
- `soc`, `tank_level`, `speed_mps`, `p_fc_kw`, `p_batt_kw`, `p_unmet_kw`, `distance_km`

### 5. Shield (`shield.py`)

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

10-dimensional normalized vector `[-1, 1]`:

| Index | Description | Normalization |
|-------|-------------|---------------|
| 0 | Speed (m/s) | `(speed / v_max) * 2 - 1` |
| 1 | SOC estimate (noisy) | `(soc + noise) * 2 - 1` |
| 2 | H2 tank level (noisy) | `(tank + noise) * 2 - 1` |
| 3 | Timetable phase | `(step / max_steps) * 2 - 1` |
| 4 | P_req filtered (kW) | `(p_req / p_req_max) * 2 - 1` |
| 5 | Last action: fc_frac | `fc_frac * 2 - 1` |
| 6 | Last action: batt_cmd | Already `[-1, 1]` |
| 7-9 | Nuisance noise | `N(0, noise_std)` |

**Note:** SOC and tank have additive Gaussian noise (configurable std).

---

## Action Space Details

2-dimensional continuous action:

- `action[0]` (`fc_frac`): `[0, 1]` → FC power fraction
  - Maps to: `P_fc = fc_frac * P_fc_max` (default: 400 kW max)
  
- `action[1]` (`batt_cmd`): `[-1, 1]` → Battery command
  - `> 0`: Discharge, maps to `[0, P_batt_max_dis]` (default: 600 kW max)
  - `< 0`: Charge, maps to `[-P_batt_max_chg, 0]` (default: 300 kW max)

**Shield applies:** Actions are automatically constrained by the shield before execution.

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
- **Delay penalty:** `lambda_delay * dt * indicator` (default: 0.5, currently placeholder)
- **Unmet demand penalty:** `lambda_unmet * P_unmet` (default: 1e-6)

**Note:** Rewards are negative (costs), so higher is better for the agent.

---

## Episode Termination

**Terminated (hard stop):**
- `SOC <= soc_hard_min` (default: 0.15)
- `tank_level <= tank_hard_min` (default: 0.02)

**Truncated (time limit):**
- `episode_step >= episode_length` (random: 1200–1800 steps)

---

## Randomization

Per episode (from config):
- **Passenger mass:** `U[180, 260]` tons (hidden, affects dynamics)
- **Initial SOC:** `U[0.55, 0.80]` (if `soc_init_uniform: true`)
- **Initial tank:** `U[0.70, 1.00]` (if `tank_init_uniform: true`)
- **Auxiliary bias:** Random walk with `σ_b` (default: 0.10 kW/s)
- **Timetable jitter:** ±10% (if `timetable_jitter_enable: true`)

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
    low=-1.0, high=1.0, shape=(11,), dtype=np.float32  # Changed from 10
)
```

2. Add to `_get_observation()`:
```python
obs = np.zeros(11, dtype=np.float32)
# ... existing channels ...
obs[10] = new_channel_value  # Normalized to [-1, 1]
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

## Testing

Run smoke test:
```bash
uv run python main.py
```

Or test programmatically:
```python
from rl_hyb_train import make_env
from pathlib import Path
import numpy as np

env = make_env(Path("conf.yaml"), seed=42)
obs, info = env.reset()
assert obs.shape == (10,)
assert obs.dtype == np.float32

action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
assert isinstance(reward, (float, np.floating))
assert isinstance(terminated, (bool, np.bool_))  # Note: numpy bools
assert isinstance(truncated, (bool, np.bool_))  # Note: numpy bools
```

---

## Important Notes

1. **POMDP:** Passenger mass and auxiliary bias are hidden (not in observations)
2. **Shield:** Actions are automatically constrained; violations are tracked in info
3. **Time step:** Fixed at 1 second (`dt_seconds: 1`)
4. **Normalization:** All observations are normalized to `[-1, 1]`
5. **Config-driven:** All parameters come from `conf.yaml` (no hardcoded values)
6. **Episode length:** Random per episode (1200–1800 steps by default)

---

## Dependencies

- `gymnasium>=1.2.2` — Gym interface
- `numpy>=2.3.4` — Numerical operations
- `pyyaml>=6.0.3` — Config loading

---

## File Reference

- **`env0.md`**: Detailed specification document (read this for full understanding)
- **`conf.yaml`**: All configuration parameters
- **`main.py`**: Example usage / smoke test
- **`agents.md`**: This file (quick reference for agents)

---

## Questions?

Check:
1. `env0.md` for detailed specification
2. `conf.yaml` for all configurable parameters
3. Docstrings in source code for API details
4. `main.py` for example usage
