"""Common interfaces and helpers for EMS policies."""
from __future__ import annotations

from typing import Dict, Optional, Any, Protocol

import numpy as np


Action = np.ndarray


class EMSPolicy(Protocol):
    """Protocol for policies that can act given observations and optional info."""

    def reset(self) -> None:
        ...

    def act(self, obs: np.ndarray, info: Optional[Dict[str, Any]] = None) -> Action:
        ...


class InfoDrivenPolicy:
    """
    Helper base class for policies that operate on the info dict.

    Subclasses implement `act_from_info`; this class provides the common `act`
    method so callers can treat all policies uniformly.
    """

    def reset(self) -> None:
        pass

    def act(self, obs: np.ndarray, info: Optional[Dict[str, Any]] = None) -> Action:
        if info is None:
            raise ValueError(f"{self.__class__.__name__} requires env info to act")
        return self.act_from_info(info)

    def act_from_info(self, info: Dict[str, Any]) -> Action:  # pragma: no cover - interface
        raise NotImplementedError
