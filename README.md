# RL Hybrid Train (Env0)

Hybrid Fuel-Cell + Battery train environment for RL research. Env0 models a driver-in-the-loop scenario where a driver/ATO requests traction/braking power and an Energy Management System (EMS) allocates power between the Fuel Cell and Battery under a safety shield and physical plant dynamics.

- Gymnasium-compatible environment with standard API
- Config-driven (single `conf.yaml`)
- Deterministic 1 s step physics with auxiliaries, SOC/H₂, and kinematics
- Safety shield for ramp, SOC corridor, and C‑rate constraints
- Baseline, balanced, scenario-aware, and MPC EMS policies included


## Quick Start

```python
from rl_hyb_train import make_env
from pathlib import Path

env = make_env(Path("conf.yaml"), seed=42)
obs, info = env.reset()
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

Run the smoke example:

```bash
uv run python main.py
```


## Installation

Requires Python 3.11+.

- Using pip (editable install):
  ```bash
  python -m venv .venv && source .venv/bin/activate
  pip install -U pip
  pip install -e .
  ```

- Using uv (recommended for local tooling):
  ```bash
  uv run python -V            # ensure Python available via uv
  uv pip install -e .
  ```


## Diagrams

High-level diagrams are pre-generated under `docs/`. Regenerate via the helper scripts (Graphviz `dot` required):

```bash
uv run python scripts/gen_env0_detailed.py
uv run python scripts/gen_env0_physics.py
uv run python scripts/gen_env0_diagram.py
```

Rendered diagrams:

- Detailed dataflow (EMS + Shield + Plant)
  
  ![env0_detailed](docs/env0_detailed.png)

- Physics power balance view
  
  ![env0_physics](docs/env0_physics.png)

- Simplified overview
  
  ![env0_diagram](docs/env0_diagram.png)

- Example renderer snapshot
  
  ![env0_render](env0_render.png)


## Project Structure

```
rl_hyb_train/
├── __init__.py          # Exports: Config, Env0, make_env
├── config.py            # Config dataclasses and YAML loader
├── env0_env.py          # Gymnasium environment (Env0)
├── driver.py            # Driver/ATO (generates P_req)
├── plant.py             # Plant dynamics (SOC, H2, kinematics, power balance)
├── shield.py            # Safety shield (constraint enforcement)
├── policies/            # Baselines (balanced, scenario, MPC, RL hooks)
└── renderer.py          # Plotting/renderer

conf.yaml                # Configuration (all parameters)
main.py                  # Smoke test / example usage
docs/                    # Diagrams (.dot/.png)
scripts/                 # Diagram generators
```


## Environment Interface

- Observation: normalized vector; includes speed, stores, P_req, last action, noise
- Action: `[fc_frac, batt_cmd]` where `fc_frac ∈ [0,1]`, `batt_cmd ∈ [-1,1]`
- Step returns: `(obs, reward, terminated, truncated, info)`
- `info` includes: `soc`, `tank_level`, `speed_mps`, `p_req_kw`, `p_fc_kw`, `p_batt_kw`, `p_unmet_kw`, `distance_km`, `constraint_violations`, `step`

See `env0.md` and `spec.md` for full details on spaces, physics, and rewards.


## Policies

- `baseline`, `balanced`, `scenario`, `mpc`, and RL stubs under `rl_hyb_train/policies/`
- Select via your own training loop or instantiate EMS classes directly


## Reproducing Example Plots

The repo includes several test harness scripts that generate PNGs for different scenarios (steady cruise, ramps, braking). Open the `test_ems_*.py` files to see usage patterns and power flow plots.


## Contributing

- Please open issues/PRs for bugs or feature requests.
- Code is configured with a simple dependency set in `pyproject.toml`.
- Diagrams are generated from scripts in `scripts/` via Graphviz.


## Acknowledgements

- Built for research and teaching on hybrid energy management.
- Uses Gymnasium, NumPy, matplotlib, and optional SB3 for RL baselines.

