"""Simple P_req generator: segment-based, LPF + rate limit, small noise.

Intended for early training and smoke tests when a realistic speed-profile
driver is unnecessary. Reads segments from `DriverConfig.manual_p_req_profile`.

Each segment dict may include:
- duration_s: float (required)
- mode: str in {"dwell","accelerate","cruise","climb","descent","brake"}
- base_kw: float (default 0.0)
- ramp_kw_per_s: float (for accelerate, default 300.0)
- grade_bias_kw: float (default 0.0)
- brake_kw: float (cap for braking, default 400.0)
- label: str (optional)

Applies first-order low-pass filtering and rate limiting using
`DriverConfig.p_req_smoothing_tau_s` and `DriverConfig.p_req_rate_limit_kw_per_s`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Dict, Any
import numpy as np

from .config import DriverConfig


@dataclass
class SimpleState:
    t: float = 0.0
    seg_idx: int = 0
    seg_elapsed: float = 0.0
    p_req_kw: float = 0.0
    p_req_lpf_kw: float = 0.0
    target_speed_mps: float = 0.0
    smoothed_target_speed_mps: float = 0.0
    is_dwelling: bool = False
    dwell_end_time: float = 0.0
    current_time: float = 0.0
    next_stop_time: float = 0.0


class SimplePReqDriver:
    """Very simple P_req generator from scripted segments."""

    def __init__(self, config: DriverConfig, rng: np.random.Generator):
        self.config = config
        self.rng = rng
        # Copy segments locally and normalize fields
        self.segments: List[Dict[str, Any]] = []
        for seg in (config.manual_p_req_profile or []):
            self.segments.append({
                "duration_s": float(seg.get("duration_s", seg.get("duration", 0.0))),
                "mode": str(seg.get("mode", "cruise")).lower(),
                "base_kw": float(seg.get("base_kw", 0.0)),
                "ramp_kw_per_s": float(seg.get("ramp_kw_per_s", 300.0)),
                "grade_bias_kw": float(seg.get("grade_bias_kw", 0.0)),
                "brake_kw": float(seg.get("brake_kw", 400.0)),
                "label": seg.get("label", None),
            })
        self.state = SimpleState()
        # Noise and smoothing defaults
        self.noise_eps = 0.03  # ±3% multiplicative noise
        self.tau = max(float(config.p_req_smoothing_tau_s), 0.0)
        self.rate_limit = float(config.p_req_rate_limit_kw_per_s)
        self.p_loss_kw = float(config.p_loss_watts) / 1000.0

    def reset(self) -> None:
        self.state = SimpleState()

    def step(self, dt: float, current_speed_mps: float | None = None) -> float:
        if not self.segments:
            # No segments provided: return zero-demand dwell-like behavior
            return 0.0

        # Advance time within current segment
        s = self.state
        seg = self.segments[s.seg_idx]
        s.t += dt
        s.seg_elapsed += dt
        s.current_time = s.t

        # Determine target based on mode
        mode = seg["mode"]
        base = seg["base_kw"]
        target = 0.0
        if mode == "dwell":
            target = 0.0
        elif mode == "accelerate":
            target = min(base, s.p_req_kw + seg["ramp_kw_per_s"] * dt)
        elif mode == "cruise":
            target = base
        elif mode == "climb":
            target = base + seg["grade_bias_kw"]
        elif mode == "descent":
            target = max(0.0, base + seg["grade_bias_kw"])  # typically negative bias not applied to traction
        elif mode == "brake":
            target = -min(abs(seg["brake_kw"]), abs(base) if base != 0 else seg["brake_kw"])
        else:
            target = base

        # Add small multiplicative noise
        noise = 1.0 + self.rng.uniform(-self.noise_eps, self.noise_eps)
        target *= noise

        # Add fixed loss offset to emulate generic demand bias
        target_with_loss = target + self.p_loss_kw

        # Low-pass filter
        if self.tau > 0.0:
            alpha = dt / (self.tau + dt)
            s.p_req_lpf_kw = s.p_req_lpf_kw + alpha * (target_with_loss - s.p_req_lpf_kw)
        else:
            s.p_req_lpf_kw = target_with_loss

        # Rate limit
        max_step = self.rate_limit * dt if self.rate_limit > 0 else np.inf
        delta = np.clip(s.p_req_lpf_kw - s.p_req_kw, -max_step, max_step)
        s.p_req_kw = s.p_req_kw + delta

        # Segment rollover
        if s.seg_elapsed >= max(seg["duration_s"], 1e-6):
            s.seg_idx = (s.seg_idx + 1) % len(self.segments) if self.config.manual_loop else min(s.seg_idx + 1, len(self.segments) - 1)
            s.seg_elapsed = 0.0

        # Update pseudo speed & dwell metadata for renderer compatibility
        is_dwelling = mode == "dwell"
        s.is_dwelling = is_dwelling
        pseudo_speed = 0.0 if is_dwelling else seg.get("target_speed_mps", 20.0)
        s.target_speed_mps = pseudo_speed
        s.smoothed_target_speed_mps = pseudo_speed
        remaining_this = max(seg["duration_s"] - s.seg_elapsed, 0.0)
        s.dwell_end_time = s.t + remaining_this if is_dwelling else s.t
        s.next_stop_time = s.t + self._time_to_next_dwell(s.seg_idx, remaining_this)

        return float(s.p_req_kw)

    def _time_to_next_dwell(self, seg_idx: int, remaining_this: float) -> float:
        """Rough time until next dwell segment (loop-aware)."""
        if not self.segments:
            return float("inf")

        # If current segment is dwell, time to stop is just remaining duration
        if self.segments[seg_idx]["mode"] == "dwell":
            return remaining_this

        total = max(remaining_this, 0.0)
        count = len(self.segments)
        for offset in range(1, count + 1):
            idx = (seg_idx + offset) % count if self.config.manual_loop else seg_idx + offset
            if idx >= count:
                break
            total += max(self.segments[idx]["duration_s"], 0.0)
            if self.segments[idx]["mode"] == "dwell":
                return total
        return float("inf")
