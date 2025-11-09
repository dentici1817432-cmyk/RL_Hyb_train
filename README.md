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


## Diagram

The detailed dataflow (EMS + Shield + Plant) for Env0:

![env0_detailed](docs/env0_detailed.png)


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


## Scenario Examples (tests)

Illustrative outputs from the included test harnesses. Each figure shows driver demand, FC/Battery allocations, unmet demand, and store levels.

- Steady cruise — constant speed and demand; EMS maintains SOC and moderates FC usage.

  ![steady_cruise](test_ems_steady_cruise.png)

- Ramp profile — rising traction demand; highlights FC ramping and battery support during transients.

  ![ramp_profile](test_ems_ramp_profile.png)

- NIL-like profile — varied suburban schedule; mixed climbs and dwells.

  ![nil_like](test_ems_nil_like_profile.png)

- Emergency braking — negative P_req with regen; shows regen capture vs friction braking.

  ![emergency_braking](test_ems_emergency_braking.png)

- Regen profile — focused braking segment; tests charge acceptance and SOC corridor.

  ![regen_profile](test_ems_regen_profile.png)

- Step changes — abrupt demand steps; examines FC ramp limit and battery buffering.

  ![step_changes](test_ems_step_changes.png)

- High SOC regen — limited charge headroom; demonstrates regen clipping at high SOC.

  ![high_soc_regen](test_ems_high_soc_regen.png)


## Contributing

- Please open issues/PRs for bugs or feature requests.
- Code is configured with a simple dependency set in `pyproject.toml`.
- Diagrams are generated from scripts in `scripts/` via Graphviz.


## Acknowledgements

- Built for research and teaching on hybrid energy management.
- Uses Gymnasium, NumPy, matplotlib, and optional SB3 for RL baselines.
