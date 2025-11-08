"""RL Hybrid Train - Supervisory EMS (POMDP) Environment"""

__version__ = "0.1.0"

from pathlib import Path
from typing import Optional

from .config import Config
from .env0_env import Env0
from .policies import (
    BaselineEMS,
    BalancedEMS,
    ScenarioAwareEMS,
    MPCEms,
    RLEMS,
)

__all__ = [
    "Config",
    "Env0",
    "BaselineEMS",
    "BalancedEMS",
    "ScenarioAwareEMS",
    "MPCEms",
    "RLEMS",
    "make_env",
]


def make_env(config_path: str | Path, seed: Optional[int] = None) -> Env0:
    """
    Create an Env0 environment from a configuration file path.
    
    Args:
        config_path: Path to the YAML configuration file
        seed: Optional random seed (overrides config if provided)
    
    Returns:
        Env0 environment instance
    """
    config = Config.from_yaml(str(config_path))
    return Env0(config, seed=seed)
