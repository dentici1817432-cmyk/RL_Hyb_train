# Baseline EMS

A simple rule-based Energy Management System (EMS) for testing the RL Hybrid Train environment. The
implementation lives in `rl_hyb_train/policies/baseline.py` and is re-exported via
`rl_hyb_train.baseline_ems` for backward compatibility.

## Strategy

The baseline EMS prioritizes **meeting power demand** while managing SOC and costs:

1. **FC Power**: Uses FC at maximum (400 kW) when demand exceeds base load threshold
2. **Battery**: Discharges to cover remaining demand after FC, charges during regen or surplus
3. **SOC Management**: 
   - Charges battery when SOC < 0.4 (low threshold)
   - Avoids charging when SOC > 0.85 (high threshold)
   - Targets SOC around 0.60

## Usage

```python
from rl_hyb_train import make_env, Config, BaselineEMS
from pathlib import Path

# Create environment
config_path = Path("conf.yaml")
env = make_env(config_path, seed=42)
config = Config.from_yaml(config_path)

# Create baseline EMS
ems = BaselineEMS(config)

# Run episode
obs, info = env.reset()
for step in range(1000):
    # Get action from baseline EMS (uses info dict internally)
    action = ems.act(obs, info)
    
    # Step environment
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        break
```

## Performance Notes

- **Unmet Demand**: During high acceleration phases, P_req can exceed 2000 kW, which exceeds the combined FC (400 kW) + Battery (600 kW) = 1000 kW capacity. This results in unmet demand, which is expected behavior.
- **FC Ramp Limits**: The shield enforces FC ramp rate limits (40 kW/s), so FC power may take time to reach maximum during rapid demand changes.
- **Constraint Violations**: Some violations may occur due to shield constraints (SOC corridor, C-rate limits), but the shield prevents hard violations.

## Test Script

Run the test script to evaluate baseline EMS performance:

```bash
uv run python test_baseline_ems.py
```

This will run a full episode and report:
- Total costs (H2 + grid)
- Average and max unmet demand
- Constraint violations
- Performance metrics (cost per km)
