#!/usr/bin/env python3
"""
Step 1 RL scaffold: train a small PPO+LSTM agent on the simple deterministic
driver profile to learn basic power accommodation behavior.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Callable, List
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

try:  # Prefer core SB3 build; fall back to sb3-contrib on older releases.
    from stable_baselines3 import RecurrentPPO
except ImportError:  # pragma: no cover - sb3_contrib path
    from sb3_contrib import RecurrentPPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# Check if tensorboard is available
try:
    import tensorboard
    TENSORBOARD_AVAILABLE = True
except ImportError:
    TENSORBOARD_AVAILABLE = False

from rl_hyb_train import Config
from rl_hyb_train.env0_env import Env0


DEFAULT_SEGMENTS: List[dict] = [
    {"duration_s": 120, "mode": "dwell", "label": "Station A"},
    {"duration_s": 180, "mode": "accelerate", "base_kw": 600, "ramp_kw_per_s": 300, "label": "Depart A"},
    {"duration_s": 600, "mode": "cruise", "base_kw": 500, "label": "Cruise"},
    {"duration_s": 300, "mode": "climb", "base_kw": 520, "grade_bias_kw": 120, "label": "Climb"},
    {"duration_s": 180, "mode": "brake", "brake_kw": 400, "label": "Approach/Brake"},
    {"duration_s": 90, "mode": "dwell", "label": "Station B"},
]


def apply_step1_overrides(
    config: Config,
    lambda_track: float,
    lambda_smooth: float,
    track_scale_kw: float,
) -> Config:
    """Inject deterministic driver + reward tweaks for the Step 1 curriculum."""
    config.driver.manual_p_req_profile = DEFAULT_SEGMENTS
    config.driver.manual_loop = True

    # Keep a short, repeatable episode with zero timing randomness.
    segment_steps = int(sum(seg["duration_s"] for seg in DEFAULT_SEGMENTS) / config.sim.dt_seconds)
    config.sim.episode_steps_min = segment_steps
    config.sim.episode_steps_max = segment_steps

    # Remove exogenous randomness so PPO sees a deterministic curriculum.
    config.randomization.passenger_mass_tons_min = 220.0
    config.randomization.passenger_mass_tons_max = 220.0
    config.randomization.aux_bias_rw_sigma_kw = 0.0
    config.randomization.soc_init_uniform = False
    mid_soc = (config.battery.soc_init_min + config.battery.soc_init_max) / 2.0
    config.battery.soc_init_min = config.battery.soc_init_max = mid_soc
    config.randomization.tank_init_uniform = False
    mid_tank = (config.fuel_cell.tank_init_min + config.fuel_cell.tank_init_max) / 2.0
    config.fuel_cell.tank_init_min = config.fuel_cell.tank_init_max = mid_tank
    config.randomization.timetable_jitter_enable = False

    # Reward: track-only objective (no economics) with mild smoothness.
    config.costs.c_h2_eur_per_kg = 0.0
    config.costs.c_grid_eur_per_kwh = 0.0
    config.reward_weights.lambda_track = lambda_track
    config.reward_weights.p_scale_kw = max(track_scale_kw, 1e-6)
    config.reward_weights.lambda_unmet = 0.0
    config.reward_weights.lambda_smooth = lambda_smooth
    config.reward_weights.lambda_delay = 0.0

    # Disable renderer to save CPU during training.
    config.renderer.enabled = False

    return config


def make_env_fn(
    config_path: Path,
    base_seed: int,
    rank: int,
    lambda_track: float,
    lambda_smooth: float,
    track_scale_kw: float,
) -> Callable[[], Env0]:
    """Factory used by DummyVecEnv so each worker gets its own Config copy."""

    def _init() -> Env0:
        cfg = Config.from_yaml(config_path)
        cfg = apply_step1_overrides(
            cfg,
            lambda_track=lambda_track,
            lambda_smooth=lambda_smooth,
            track_scale_kw=track_scale_kw,
        )
        env_seed = base_seed + rank
        return Env0(cfg, seed=env_seed)

    return _init


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train PPO/LSTM on simple driver (Step 1 - Accommodate Power)")
    parser.add_argument("--config", type=Path, default=Path("conf.yaml"), help="Path to base YAML config")
    parser.add_argument("--total-steps", type=int, default=1_000_000, help="Total training timesteps")
    parser.add_argument("--n-envs", type=int, default=4, help="Parallel environments for PPO")
    parser.add_argument("--seed", type=int, default=42, help="Base RNG seed")
    parser.add_argument("--lambda-track", type=float, default=1e-3, help="Penalty weight for unmet/wasted power")
    parser.add_argument("--lambda-smooth", type=float, default=5e-3, help="Action smoothness weight")
    parser.add_argument("--track-scale-kw", type=float, default=800.0, help="Normalization factor for tracking penalty (kW)")
    parser.add_argument("--learning-rate", type=float, default=3e-4, help="PPO learning rate")
    parser.add_argument("--rollout-steps", type=int, default=2048, help="Steps per rollout per env before an update")
    parser.add_argument("--log-dir", type=Path, default=Path("runs/step1_accommodate_power"), help="TensorBoard + checkpoints dir")
    parser.add_argument("--checkpoint-freq", type=int, default=200_000, help="Checkpoint frequency in env steps")
    parser.add_argument("--eval-freq", type=int, default=50_000, help="Evaluation frequency in env steps")
    parser.add_argument("--eval-episodes", type=int, default=5, help="Episodes per evaluation sweep")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir = args.log_dir / "checkpoints"
    ckpt_dir.mkdir(exist_ok=True, parents=True)

    env_fns = [
        make_env_fn(args.config, args.seed, rank, args.lambda_track, args.lambda_smooth, args.track_scale_kw)
        for rank in range(args.n_envs)
    ]
    vec_env = VecMonitor(DummyVecEnv(env_fns))

    eval_env = VecMonitor(
        DummyVecEnv(
            [
                make_env_fn(
                    args.config,
                    args.seed + 10_000,
                    0,
                    args.lambda_track,
                    args.lambda_smooth,
                    args.track_scale_kw,
                )
            ]
        )
    )

    # Set tensorboard_log only if tensorboard is available
    tensorboard_log = str(args.log_dir / "tb") if TENSORBOARD_AVAILABLE else None
    if not TENSORBOARD_AVAILABLE:
        print("[Warning] TensorBoard not installed. Training logs will not be saved to TensorBoard.")
        print("  Install with: uv pip install tensorboard")

    model = RecurrentPPO(
        policy="MlpLstmPolicy",
        env=vec_env,
        learning_rate=args.learning_rate,
        n_steps=args.rollout_steps,
        batch_size=512,
        n_epochs=4,
        clip_range=0.2,
        gae_lambda=0.95,
        gamma=0.998,
        ent_coef=0.0,
        vf_coef=0.5,
        tensorboard_log=tensorboard_log,
        seed=args.seed,
        verbose=1,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.checkpoint_freq // args.n_envs, 1),
        save_path=str(ckpt_dir),
        name_prefix="ppo_step1",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(args.log_dir / "best_model"),
        log_path=str(args.log_dir / "eval"),
        eval_freq=max(args.eval_freq // args.n_envs, 1),
        n_eval_episodes=args.eval_episodes,
        deterministic=True,
        render=False,
    )

    print(f"[Step1] Training for {args.total_steps:,} timesteps with {args.n_envs} envs...")
    model.learn(
        total_timesteps=args.total_steps,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=True,
    )

    final_model_path = args.log_dir / "ppo_step1_final.zip"
    model.save(final_model_path)
    print(f"[Step1] Saved final policy to {final_model_path}")

    # Dump run metadata for quick reference.
    metadata = {
        "total_steps": args.total_steps,
        "n_envs": args.n_envs,
        "lambda_track": args.lambda_track,
        "lambda_smooth": args.lambda_smooth,
        "track_scale_kw": args.track_scale_kw,
        "learning_rate": args.learning_rate,
        "rollout_steps": args.rollout_steps,
        "seed": args.seed,
        "segments": DEFAULT_SEGMENTS,
    }
    with (args.log_dir / "run_meta.json").open("w", encoding="utf-8") as fh:
        json.dump(metadata, fh, indent=2)
    print(f"[Step1] Wrote metadata to {args.log_dir / 'run_meta.json'}")


if __name__ == "__main__":
    main()
