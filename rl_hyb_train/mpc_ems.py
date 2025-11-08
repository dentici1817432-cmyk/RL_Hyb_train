"""Backward-compatible shim for MPCEms."""
from .policies.mpc import MPCEms, MPCConfig

__all__ = ["MPCEms", "MPCConfig"]
