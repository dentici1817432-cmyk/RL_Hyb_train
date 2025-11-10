#!/usr/bin/env python3
"""Render the simple driver scenario using the EMSTestHarness plotter."""
from pathlib import Path
import argparse
import numpy as np
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rl_hyb_train.config import Config
from rl_hyb_train.driver_simple import SimplePReqDriver
from rl_hyb_train.policies.balanced import BalancedEMS
from rl_hyb_train.plotting import plot_history
from test_ems_with_p_req import EMSTestHarness


DEFAULT_SEGMENTS = [
    {"duration_s": 120, "mode": "dwell", "label": "Station A"},
    {"duration_s": 180, "mode": "accelerate", "base_kw": 600, "ramp_kw_per_s": 300, "label": "Depart A"},
    {"duration_s": 600, "mode": "cruise", "base_kw": 500, "label": "Cruise"},
    {"duration_s": 300, "mode": "climb", "base_kw": 520, "grade_bias_kw": 120, "label": "Climb"},
    {"duration_s": 180, "mode": "brake", "brake_kw": 400, "label": "Approach/Brake"},
    {"duration_s": 90,  "mode": "dwell", "label": "Station B"},
]


def generate_simple_profile(cfg: Config, duration_s: int) -> tuple[np.ndarray, np.ndarray]:
    """Use SimplePReqDriver to generate a P_req profile."""
    cfg.driver.manual_p_req_profile = DEFAULT_SEGMENTS
    cfg.driver.manual_loop = True

    driver = SimplePReqDriver(cfg.driver, np.random.default_rng(0))
    driver.reset()

    dt = cfg.sim.dt_seconds
    steps = int(duration_s / dt)
    time = np.arange(steps) * dt
    p_req = np.zeros(steps)
    for i in range(steps):
        p_req[i] = driver.step(dt)
    return time, p_req


class BalancedAdapter:
    def __init__(self, cfg: Config):
        self.policy = BalancedEMS(cfg)

    def reset(self):
        self.policy.reset()

    def compute_action(self, obs, info):  # match harness expectation
        return self.policy.act(info=info)


class PPOAdapter:
    """Wrap a saved (Recurrent)PPO policy so harness can query actions."""

    def __init__(self, model_path: Path, deterministic: bool = True):
        try:
            from stable_baselines3 import RecurrentPPO
        except ImportError:  # pragma: no cover
            from sb3_contrib import RecurrentPPO

        self.model = RecurrentPPO.load(str(model_path), device="cpu")
        self.deterministic = deterministic
        self.lstm_state = None
        self.episode_start = np.array([True], dtype=bool)

    def reset(self):
        self.lstm_state = None
        self.episode_start = np.array([True], dtype=bool)

    def compute_action(self, obs, info):
        obs_arr = np.asarray(obs, dtype=np.float32)
        action, self.lstm_state = self.model.predict(
            obs_arr, state=self.lstm_state, episode_start=self.episode_start, deterministic=self.deterministic
        )
        self.episode_start[:] = False
        return action


def main():
    parser = argparse.ArgumentParser(description="Render simple driver EMS response")
    parser.add_argument("--config", type=str, default="conf.yaml", help="Config path")
    parser.add_argument("--duration", type=int, default=600, help="Duration in seconds")
    parser.add_argument("--output", type=str, default="test_simple_driver.png", help="Output PNG path")
    parser.add_argument(
        "--policy",
        type=str,
        choices=("balanced", "ppo"),
        default="balanced",
        help="Which EMS policy to render",
    )
    parser.add_argument(
        "--model-path",
        type=str,
        default=None,
        help="Path to saved (Recurrent)PPO checkpoint when --policy=ppo",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Use stochastic PPO actions (default deterministic)",
    )
    args = parser.parse_args()

    cfg = Config.from_yaml(args.config)

    time_arr, p_req_arr = generate_simple_profile(cfg, args.duration)

    harness = EMSTestHarness(config_path=args.config)
    if args.policy == "balanced":
        harness.ems = BalancedAdapter(harness.cfg)
    else:
        if args.model_path is None:
            raise ValueError("--model-path is required when --policy=ppo")
        harness.ems = PPOAdapter(Path(args.model_path), deterministic=not args.stochastic)
    harness.ems.reset()

    harness.run_scenario(time_arr, p_req_arr, soc_init=0.6)
    history = harness.get_history_arrays()

    fig = plot_history(history, "simple_driver")
    out_path = Path(args.output)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved figure: {out_path}")


if __name__ == "__main__":
    main()
