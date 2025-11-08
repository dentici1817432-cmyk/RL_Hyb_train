"""Driver/ATO module that emits a scripted traction power profile."""
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Any, Tuple, Optional

from .config import DriverConfig, RandomizationConfig


@dataclass
class DriverState:
    """Lightweight driver state exposed to the rest of the environment."""

    current_time: float = 0.0
    current_segment: int = 0
    segment_start_time: float = 0.0
    speed_mps: float = 0.0
    target_speed_mps: float = 0.0
    smoothed_target_speed_mps: float = 0.0
    is_dwelling: bool = False
    dwell_end_time: float = 0.0
    next_stop_time: float = 0.0
    timetable_offset: float = 0.0
    pid_integral: float = 0.0
    pid_prev_error: float = 0.0
    pid_prev_accel: float = 0.0
    p_req_filtered: float = 0.0


class Driver:
    """Generates requested traction power from a predefined P_req time profile."""

    def __init__(
        self,
        config: DriverConfig,
        randomization_config: RandomizationConfig,
        rng: np.random.Generator,
        plant_kinematic_gain_mps_per_watt: float = None,
    ):
        self.config = config
        self.randomization_config = randomization_config
        self.rng = rng
        self.state = DriverState()

        self.p_req_smoothing_tau_s = max(config.p_req_smoothing_tau_s, 0.0)
        self.p_req_rate_limit_kw_per_s = config.p_req_rate_limit_kw_per_s
        self.manual_loop = config.manual_loop
        self.kinematic_gain_mps_per_watt = plant_kinematic_gain_mps_per_watt or 2.5e-6
        self.p_loss_kw = self.config.p_loss_watts / 1000.0
        self.speed_target_smoothing_tau_s = max(self.config.speed_target_smoothing_tau_s, 0.0)
        self.speed_tracking_tau_s = max(self.config.speed_tracking_tau_s, 1e-3)
        self.speed_tracking_accel_limit = max(self.config.speed_tracking_accel_limit_mps2, 1e-3)
        self.speed_tracking_brake_limit = max(self.config.speed_tracking_brake_limit_mps2, 1e-3)
        self.dwell_speed_threshold = max(self.config.dwell_speed_threshold_mps, 0.0)
        self.speed_power_clip_kw = self.config.speed_control_power_clip_kw
        jerk_limit = self.config.speed_tracking_jerk_limit_mps3
        self.speed_tracking_jerk_limit = None if jerk_limit is None else max(float(jerk_limit), 0.0)
        self.speed_profile_interpolation = (self.config.speed_profile_interpolation or "linear").strip().lower()
        if self.speed_profile_interpolation not in {"linear", "hold"}:
            self.speed_profile_interpolation = "linear"
        self.speed_tracking_use_pid = bool(self.config.speed_tracking_use_pid)
        self.speed_pid_kp = float(self.config.speed_pid_kp)
        self.speed_pid_ki = float(self.config.speed_pid_ki)
        self.speed_pid_kd = float(self.config.speed_pid_kd)
        self.speed_pid_integral_limit = max(float(self.config.speed_pid_integral_limit), 0.0)
        self.speed_pid_derivative_tau = max(float(self.config.speed_pid_derivative_filter_tau_s), 0.0)

        self.manual_segments: List[tuple] = []
        self._build_manual_segments()
        self.manual_idx = 0
        self.manual_time_in_segment = 0.0

        self.speed_segments: List[Dict[str, Any]] = []
        self._build_speed_segments()
        self.speed_idx = 0
        self.speed_time_in_segment = 0.0
        self.use_speed_profile = len(self.speed_segments) > 0
        self._pid_derivative_state = 0.0
        self._last_accel_cmd = 0.0

    def reset(self, episode_start_time: float = 0.0):
        """Reset driver state for a new episode."""
        self.state = DriverState(current_time=episode_start_time)
        initial_p_req = self.manual_segments[0][1] if self.manual_segments else 0.0
        self.state.p_req_filtered = initial_p_req
        self.manual_idx = 0
        self.manual_time_in_segment = 0.0
        self.speed_idx = 0
        self.speed_time_in_segment = 0.0
        if self.use_speed_profile and self.speed_segments:
            segment0 = self._get_speed_segment(self.speed_idx)
            initial_speed = segment0["start_speed"]
            self.state.target_speed_mps = initial_speed
            self.state.smoothed_target_speed_mps = initial_speed
        self.state.pid_integral = 0.0
        self.state.pid_prev_error = 0.0
        self.state.pid_prev_accel = 0.0
        self._pid_derivative_state = 0.0
        self._last_accel_cmd = 0.0

    def step(self, dt: float, current_speed_mps: float = None) -> float:
        """Advance driver by dt seconds and return requested traction power."""
        if dt <= 0.0:
            return self.state.p_req_filtered

        if current_speed_mps is None:
            current_speed_mps = 0.0

        self.state.current_time += dt
        self.state.speed_mps = current_speed_mps

        if self.use_speed_profile:
            target_speed, target_accel, segment_meta = self._manual_speed_step(dt)
            self.state.target_speed_mps = target_speed
            self._update_smoothed_target_speed(target_speed, dt)
            self.state.is_dwelling = self.state.target_speed_mps <= self.dwell_speed_threshold
            if self.speed_tracking_use_pid:
                raw_p_req = self._compute_speed_based_p_req_pid(
                    current_speed_mps=current_speed_mps,
                    dt=dt,
                    target_accel=target_accel,
                    segment_meta=segment_meta,
                )
            else:
                raw_p_req = self._compute_speed_based_p_req_first_order(
                    current_speed_mps=current_speed_mps,
                    dt=dt,
                    target_accel=target_accel,
                    segment_meta=segment_meta,
                )
        else:
            raw_p_req = self._manual_p_req_step(dt)
            self.state.target_speed_mps = 0.0
            self.state.smoothed_target_speed_mps = 0.0
            self.state.is_dwelling = False

        return self._filter_p_req(raw_p_req, dt)

    def _build_manual_segments(self):
        if not self.config.manual_p_req_profile:
            self.manual_segments.append((1.0, 0.0))
            return

        for segment in self.config.manual_p_req_profile:
            duration = float(segment.get("duration_s", 0.0))
            p_req = float(segment.get("p_req_kw", 0.0))
            if duration <= 0.0:
                continue
            self.manual_segments.append((duration, p_req))

        if not self.manual_segments:
            self.manual_segments.append((1.0, 0.0))
    
    def _build_speed_segments(self):
        if not self.config.manual_speed_profile:
            return

        prev_speed = None
        for segment in self.config.manual_speed_profile:
            duration = float(segment.get("duration_s", 0.0))
            v_target = float(
                segment.get(
                    "speed_mps",
                    segment.get("v_target_mps", segment.get("target_speed_mps", 0.0)),
                )
            )
            if duration <= 0.0:
                continue

            seg_interp = (segment.get("interp") or self.speed_profile_interpolation).strip().lower()
            if seg_interp not in {"linear", "hold"}:
                seg_interp = "linear"
            hold_flag = bool(segment.get("hold", False) or seg_interp == "hold")

            start_speed = v_target if hold_flag or prev_speed is None else prev_speed
            end_speed = v_target
            meta = {
                "label": segment.get("label"),
                "p_bias_kw": float(segment.get("p_bias_kw", 0.0)),
                "grade_percent": float(segment.get("grade_percent", 0.0)),
                "dwell": bool(segment.get("is_dwell", hold_flag or v_target <= self.dwell_speed_threshold)),
            }
            slope = 0.0 if duration <= 0.0 or hold_flag else (end_speed - start_speed) / duration
            self.speed_segments.append(
                {
                    "duration": duration,
                    "start_speed": start_speed,
                    "end_speed": end_speed,
                    "meta": meta,
                    "hold": hold_flag,
                    "interp": seg_interp,
                    "slope": slope,
                }
            )
            prev_speed = end_speed

    def _get_speed_segment(self, idx: int) -> Dict[str, Any]:
        return self.speed_segments[idx]

    def _manual_speed_step(self, dt: float) -> Tuple[float, float, Dict[str, Any]]:
        segment = self._get_speed_segment(self.speed_idx)
        duration = segment["duration"]
        self.speed_time_in_segment += dt

        while self.speed_time_in_segment >= duration:
            self.speed_time_in_segment -= duration

            if self.manual_loop:
                self.speed_idx = (self.speed_idx + 1) % len(self.speed_segments)
            else:
                if self.speed_idx < len(self.speed_segments) - 1:
                    self.speed_idx += 1
                else:
                    self.speed_time_in_segment = duration
                    break

            segment = self._get_speed_segment(self.speed_idx)
            duration = segment["duration"]

        if duration <= 0.0:
            progress = 1.0
        else:
            progress = min(1.0, max(0.0, self.speed_time_in_segment / duration))

        if segment["hold"]:
            v_target = segment["end_speed"]
        elif segment["interp"] == "linear":
            v_target = segment["start_speed"] + (segment["end_speed"] - segment["start_speed"]) * progress
        else:
            v_target = segment["end_speed"]

        target_accel = 0.0 if segment["hold"] else segment["slope"]
        return v_target, target_accel, segment["meta"]

    def _update_smoothed_target_speed(self, target_speed: float, dt: float) -> None:
        tau = self.speed_target_smoothing_tau_s
        if tau <= 0.0:
            self.state.smoothed_target_speed_mps = target_speed
        else:
            alpha = dt / (tau + dt)
            prev = self.state.smoothed_target_speed_mps
            self.state.smoothed_target_speed_mps = alpha * target_speed + (1.0 - alpha) * prev

    def _compute_speed_based_p_req_first_order(
        self,
        current_speed_mps: float,
        dt: float,
        target_accel: float,
        segment_meta: Optional[Dict[str, Any]] = None,
    ) -> float:
        target_speed = self.state.smoothed_target_speed_mps
        speed_error = target_speed - current_speed_mps
        base_accel = target_accel
        correction = speed_error / self.speed_tracking_tau_s
        desired_accel = base_accel + correction
        accel_cmd, _ = self._apply_accel_constraints(desired_accel, dt)
        return self._accel_to_power(accel_cmd, segment_meta)

    def _compute_speed_based_p_req_pid(
        self,
        current_speed_mps: float,
        dt: float,
        target_accel: float,
        segment_meta: Optional[Dict[str, Any]] = None,
    ) -> float:
        target_speed = self.state.smoothed_target_speed_mps
        error = target_speed - current_speed_mps
        if dt <= 0.0:
            dt = 1.0

        prev_integral = self.state.pid_integral
        integral = self.state.pid_integral + error * dt
        if self.speed_pid_integral_limit > 0.0:
            integral = float(np.clip(integral, -self.speed_pid_integral_limit, self.speed_pid_integral_limit))
        self.state.pid_integral = integral

        derivative = (error - self.state.pid_prev_error) / dt if dt > 1e-6 else 0.0
        if self.speed_pid_derivative_tau > 0.0:
            alpha = dt / (self.speed_pid_derivative_tau + dt)
            self._pid_derivative_state = (1.0 - alpha) * self._pid_derivative_state + alpha * derivative
            derivative = self._pid_derivative_state

        accel_unclamped = (
            self.speed_pid_kp * error
            + self.speed_pid_ki * integral
            + self.speed_pid_kd * derivative
        )
        accel_cmd, saturated = self._apply_accel_constraints(target_accel + accel_unclamped, dt)
        if saturated:
            self.state.pid_integral = prev_integral
        self.state.pid_prev_error = error
        self.state.pid_prev_accel = accel_cmd

        return self._accel_to_power(accel_cmd, segment_meta)

    def _apply_accel_constraints(self, accel_cmd: float, dt: float) -> tuple[float, bool]:
        saturated = False
        clipped = float(
            np.clip(
                accel_cmd,
                -self.speed_tracking_brake_limit,
                self.speed_tracking_accel_limit,
            )
        )
        if abs(clipped - accel_cmd) > 1e-6:
            saturated = True
        accel_cmd = clipped

        if self.speed_tracking_jerk_limit is not None and dt > 0.0:
            max_delta = self.speed_tracking_jerk_limit * dt
            jerk_clipped = float(
                np.clip(
                    accel_cmd,
                    self._last_accel_cmd - max_delta,
                    self._last_accel_cmd + max_delta,
                )
            )
            if abs(jerk_clipped - accel_cmd) > 1e-6:
                saturated = True
            accel_cmd = jerk_clipped

        self._last_accel_cmd = accel_cmd
        return accel_cmd, saturated

    def _accel_to_power(self, accel_cmd: float, segment_meta: Optional[Dict[str, Any]]) -> float:
        if abs(self.kinematic_gain_mps_per_watt) < 1e-9:
            net_power_kw = 0.0
        else:
            net_power_kw = accel_cmd / (self.kinematic_gain_mps_per_watt * 1000.0)

        p_req = self.p_loss_kw + net_power_kw

        if segment_meta:
            p_req += float(segment_meta.get("p_bias_kw", 0.0))

        if self.speed_power_clip_kw is not None and self.speed_power_clip_kw > 0.0:
            p_req = float(
                np.clip(
                    p_req,
                    -self.speed_power_clip_kw,
                    self.speed_power_clip_kw,
                )
            )

        return p_req

    def _manual_p_req_step(self, dt: float) -> float:
        duration, p_req = self.manual_segments[self.manual_idx]
        self.manual_time_in_segment += dt

        while self.manual_time_in_segment >= duration:
            self.manual_time_in_segment -= duration

            if self.manual_loop:
                self.manual_idx = (self.manual_idx + 1) % len(self.manual_segments)
            else:
                if self.manual_idx < len(self.manual_segments) - 1:
                    self.manual_idx += 1
                else:
                    # Stick to the final segment and stop consuming time
                    self.manual_time_in_segment = duration
                    break

            duration, p_req = self.manual_segments[self.manual_idx]

        return p_req

    def _filter_p_req(self, p_req_kw: float, dt: float) -> float:
        if self.p_req_rate_limit_kw_per_s and self.p_req_rate_limit_kw_per_s > 0.0:
            dp_max = self.p_req_rate_limit_kw_per_s * dt
            lower = self.state.p_req_filtered - dp_max
            upper = self.state.p_req_filtered + dp_max
            p_req_kw = float(np.clip(p_req_kw, lower, upper))

        tau = self.p_req_smoothing_tau_s
        if tau <= 0.0:
            self.state.p_req_filtered = float(p_req_kw)
        else:
            alpha = dt / (tau + dt)
            self.state.p_req_filtered = alpha * p_req_kw + (1.0 - alpha) * self.state.p_req_filtered

        return self.state.p_req_filtered
