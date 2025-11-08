"""RL-based EMS wrapper using Stable-Baselines3 models.

This policy assumes the standard Env0 observation and action spaces:
  - Observation: Box(-1, 1, (10,), float32)
  - Action: [fc_frac \in [0,1], batt_cmd \in [-1,1]]

It loads a trained SB3 model (e.g., PPO/SAC) and uses it to generate EMS
actions. The safety shield in the environment still applies constraints.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np


class RLEMS:
    def __init__(self, model_path: Path, deterministic: bool = True):
        # Lazy import to avoid SB3 dependency at import time when not used
        from stable_baselines3.common.base_class import BaseAlgorithm
        from stable_baselines3.common.save_util import load_from_zip_file

        self.model_path = Path(model_path)
        self.deterministic = deterministic

        # Load policy and reconstruct algorithm class
        data, params, pytorch_variables = load_from_zip_file(
            self.model_path, device="auto"
        )
        alg_cls: type[BaseAlgorithm] = data["algo_class"]
        # Instantiate a dummy algorithm to attach the policy; use env=None since we act directly
        self.model: BaseAlgorithm = alg_cls.load(self.model_path, device="auto")

    def reset(self):
        pass

    def act(self, obs: np.ndarray, info: Optional[Dict[str, Any]] = None) -> np.ndarray:
        action, _ = self.model.predict(obs, deterministic=self.deterministic)
        # Ensure correct dtype/shape
        action = np.asarray(action, dtype=np.float32)
        return action

    # Compatibility with the baseline test harness; prefer act(obs, info)
    def act_from_info(self, info: Dict[str, Any]) -> np.ndarray:
        raise RuntimeError(
            "RLEMS.act_from_info was called, but this policy requires observations."
        )

