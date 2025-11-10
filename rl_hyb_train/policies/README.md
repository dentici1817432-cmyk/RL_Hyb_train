# EMS Policy Development Guide

This directory contains Energy Management System (EMS) policies for the hybrid fuel cell-battery train.

## Quick Start: Creating a New Policy

### 1. Basic Template

Create a new file `my_policy.py`:

```python
"""My custom EMS policy."""
from typing import Dict, Any, Optional
import numpy as np
from ..config import Config

class MyCustomEMS:
    """
    Brief description of your strategy.

    Strategy:
        - How you allocate fuel cell power
        - How you manage battery SOC
        - How you handle regenerative braking
    """

    def __init__(self, config: Config):
        self.config = config
        # Extract needed parameters
        self.p_fc_max_kw = config.fuel_cell.p_fc_max_kw
        self.p_batt_max_dis_kw = config.battery.p_batt_max_discharge_kw
        self.p_batt_max_chg_kw = config.battery.p_batt_max_charge_kw

        # Initialize internal state
        self.some_state_variable = 0.0

    def reset(self) -> None:
        """Reset policy state for new episode."""
        self.some_state_variable = 0.0

    def act(self, obs: np.ndarray, info: Optional[Dict[str, Any]] = None) -> np.ndarray:
        """
        Compute EMS action from observation and optional info.

        Args:
            obs: 12-dim observation vector (normalized to [-1, 1])
                 [0] speed_norm
                 [1] soc_norm
                 [2] tank_level_norm
                 [3] time_to_next_stop_norm
                 [4] p_req_filtered_norm
                 [5] p_req_trend_norm
                 [6-7] last_action (fc_frac, batt_cmd)
                 [8-11] noise channels
            info: Dict with raw state values (optional but useful)
                 - p_req_kw: Power request (kW)
                 - soc: State of charge [0, 1]
                 - tank_level: H2 tank level [0, 1]
                 - speed_mps: Speed (m/s)
                 - p_fc_prev_kw: Previous FC power

        Returns:
            action: [fc_frac, batt_cmd]
                fc_frac: Fuel cell fraction [0, 1]
                batt_cmd: Battery command [-1, 1]
                    positive = discharge
                    negative = charge
        """
        # Extract info (easier than decoding obs)
        if info is not None:
            p_req_kw = info.get('p_req_kw', 0.0)
            soc = info.get('soc', 0.65)
            tank_level = info.get('tank_level', 0.85)
        else:
            # Fallback: decode from observation
            soc = (obs[1] + 1.0) / 2.0
            p_req_max = self.config.observations.p_req_max_kw
            p_req_kw = ((obs[4] + 1.0) / 2.0) * p_req_max

        # YOUR STRATEGY HERE
        # Example: Simple load-following
        if p_req_kw > 0:
            # Traction demand
            fc_power_kw = min(p_req_kw, self.p_fc_max_kw)
            batt_power_kw = max(0.0, p_req_kw - fc_power_kw)
        else:
            # Regenerative braking
            fc_power_kw = 0.0
            batt_power_kw = max(p_req_kw, -self.p_batt_max_chg_kw)

        # Convert to action space
        fc_frac = np.clip(fc_power_kw / self.p_fc_max_kw, 0.0, 1.0)
        if batt_power_kw >= 0:
            batt_cmd = batt_power_kw / self.p_batt_max_dis_kw
        else:
            batt_cmd = batt_power_kw / self.p_batt_max_chg_kw

        return np.array([fc_frac, batt_cmd], dtype=np.float32)
```

### 2. Export Your Policy

Add to `__init__.py`:

```python
from .my_policy import MyCustomEMS

__all__ = [
    # ... existing exports
    "MyCustomEMS",
]
```

### 3. Test Your Policy

```python
from rl_hyb_train.config import Config
from rl_hyb_train.env0_env import Env0
from rl_hyb_train.policies.my_policy import MyCustomEMS

config = Config.from_yaml('conf.yaml')
env = Env0(config)
policy = MyCustomEMS(config)

obs, info = env.reset()
for step in range(100):
    action = policy.act(obs, info)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break
```

Or use the test harness:

```bash
# Add your policy to test_ems_with_p_req.py
python test_ems_with_p_req.py --scenario step_changes
```

## Existing Policies

| Policy | File | Strategy |
|--------|------|----------|
| **BaselineEMS** | `baseline.py` | Simple FC load-follower with ramp limiting |
| **BaselineNominalEMS** | `baseline_ems.py` | FC at nominal power, battery handles peaks |
| **BalancedEMS** | `balanced.py` | SOC-aware balancing with target SOC |
| **ScenarioAwareEMS** | `scenario.py` | Uses timetable/route preview |
| **MPCEms** | `mpc.py` | Model Predictive Control |
| **RLEMS** | `rl.py` | Reinforcement Learning policy wrapper |

## Key Considerations

### Power Conventions
- **Battery power**: Positive = discharge, Negative = charge
- **FC power**: Always non-negative (0 ≤ p_fc ≤ p_fc_max)
- **P_req**: Positive = traction, Negative = regen braking

### SOC Management
- Hard limits: `soc_hard_min` (0.15), `soc_hard_max` (1.0)
- Soft corridor: `soc_soft_min` (0.20), `soc_soft_max` (0.90)
- Shield enforces corridor, but your policy should respect it proactively

### Fuel Cell Ramp Rate
- Limit: `ramp_kw_per_s` (typically 40 kW/s)
- Shield enforces this, but smooth commands avoid corrections

### Battery C-Rates
- Max discharge: `p_batt_max_discharge_kw` (e.g., 600 kW for 300 kWh = 2C)
- Max charge: `p_batt_max_charge_kw` (e.g., 300 kW for 300 kWh = 1C)

### Observation Details

Observations are **normalized to [-1, 1]** via:
```python
obs_norm = 2 * (value / max_value) - 1
```

To decode:
```python
value = ((obs_norm + 1) / 2) * max_value
```

## Common Patterns

### Pattern 1: Power Split by Demand Level
```python
if p_req_kw < fc_nominal:
    fc_power = p_req_kw
    batt_power = 0.0
else:
    fc_power = fc_nominal
    batt_power = p_req_kw - fc_nominal
```

### Pattern 2: SOC-Aware Charging
```python
if soc < target_soc and fc_power < p_fc_max:
    excess_fc = p_fc_max - p_req_kw
    batt_power = -min(excess_fc, p_batt_max_chg)
```

### Pattern 3: Predictive FC Ramp-Up
```python
if time_to_high_demand < ramp_time:
    fc_power = min(fc_power + ramp_rate * dt, p_fc_max)
```

## Performance Metrics

Your policy will be evaluated on:
- **Economic**: €/km proxy, H₂ consumption, grid charging cost
- **Operational**: Unmet demand, wasted regen, delays
- **Durability**: Battery throughput, C-rates, thermal stress
- **Smoothness**: FC power jerk, action volatility

## Tips

1. **Use info dict**: Easier than decoding normalized observations
2. **Test with multiple scenarios**: step_changes, regen_profile, nil_like_profile
3. **Check shield corrections**: If shield frequently corrects your actions, your logic needs improvement
4. **Profile first, optimize later**: Start with readable code, optimize if needed
5. **Log internal state**: Helpful for debugging (use `get_policy_info()` method)

## Advanced: Stateful Policies

For policies that need memory (e.g., filters, integrators):

```python
class StatefulEMS:
    def __init__(self, config: Config):
        self.config = config
        self.fc_power_filtered = 0.0
        self.integral_error = 0.0

    def reset(self):
        self.fc_power_filtered = 0.0
        self.integral_error = 0.0

    def act(self, obs, info=None):
        # Use self.fc_power_filtered, self.integral_error, etc.
        # Update state for next step
        self.fc_power_filtered = 0.9 * self.fc_power_filtered + 0.1 * new_value
        return action
```

## Questions?

- Check existing policies in this directory
- See `test_ems_with_p_req.py` for test harness usage
- Review `RL_COMBINED_AGENT.md` for RL-specific guidance
