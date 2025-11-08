"""Driver/ATO module that emits a scripted traction power profile."""
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Any, Tuple, Optional

from .config import DriverConfig, RandomizationConfig
from .timetable import TimetableGenerator, Timetable, Stop, RouteSegment


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
    # New state for timetable-based driving
    current_position_m: float = 0.0
    current_stop_idx: int = 0
    current_route_segment_idx: int = 0


class Driver:
    """Generates requested traction power from a predefined P_req time profile."""

    def __init__(
        self,
        config: DriverConfig,
        randomization_config: RandomizationConfig,
        rng: np.random.Generator,
        plant_kinematic_gain_mps_per_watt: float = None,
        route_config=None,
        stops_config=None,
    ):
        self.config = config
        self.randomization_config = randomization_config
        self.rng = rng
        self.state = DriverState()
        
        # Timetable generation support
        self.timetable = None
        self.use_timetable = route_config is not None and stops_config is not None
        if self.use_timetable:
            self.timetable_generator = TimetableGenerator(route_config, stops_config, rng)
        else:
            self.timetable_generator = None

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
        self.speed_profile_dt = max(float(self.config.speed_profile_dt_seconds), 1e-3)
        self.speed_profile_filter_enable = bool(self.config.speed_profile_filter_enable)
        self.speed_profile_filter_tau_s = max(float(self.config.speed_profile_filter_tau_s), 0.0)
        self.speed_preview_horizon_s = max(float(self.config.speed_preview_horizon_s), 0.0)
        self.speed_planner_enable = bool(self.config.speed_planner_enable)
        self.speed_planner_horizon_s = max(float(self.config.speed_planner_horizon_s), self.speed_profile_dt)
        self.speed_planner_min_horizon_s = max(float(self.config.speed_planner_min_horizon_s), self.speed_profile_dt)
        self.speed_planner_penalty_accel = max(float(self.config.speed_planner_penalty_accel), 0.0)
        self.speed_planner_penalty_jerk = max(float(self.config.speed_planner_penalty_jerk), 0.0)
        # Added robustness knobs
        self.speed_error_deadband_mps = max(float(self.config.speed_error_deadband_mps), 0.0)
        self.speed_integral_separation_mps = max(float(self.config.speed_integral_separation_mps), 0.0)
        self.speed_disable_integral_when_saturated = bool(self.config.speed_disable_integral_when_saturated)
        self.speed_measurement_filter_tau_s = max(float(self.config.speed_measurement_filter_tau_s), 0.0)

        self.manual_segments: List[tuple] = []
        self._build_manual_segments()
        self.manual_idx = 0
        self.manual_time_in_segment = 0.0

        self._speed_schedule_raw = np.array([], dtype=np.float64)
        self._speed_schedule_filtered = np.array([], dtype=np.float64)
        self._speed_schedule_meta: List[Dict[str, Any]] = []
        self._speed_schedule_total_time = 0.0
        self.speed_segments: List[Dict[str, Any]] = []
        self._build_speed_segments()
        self.speed_idx = 0
        self.speed_time_in_segment = 0.0
        self.use_speed_profile = len(self.speed_segments) > 0
        self._pid_derivative_state = 0.0
        self._last_accel_cmd = 0.0
        self._speed_meas_filtered = 0.0

    def reset(self, episode_start_time: float = 0.0):
        """Reset driver state for a new episode."""
        self.state = DriverState(current_time=episode_start_time)
        
        # Generate timetable if using timetable mode
        if self.use_timetable:
            self.timetable = self.timetable_generator.generate()
            self.state.current_position_m = 0.0
            self.state.current_stop_idx = 0
            self.state.current_route_segment_idx = 0
            initial_speed = 0.0
        else:
            initial_p_req = self.manual_segments[0][1] if self.manual_segments else 0.0
            self.state.p_req_filtered = initial_p_req
            self.manual_idx = 0
            self.manual_time_in_segment = 0.0
            self.speed_idx = 0
            self.speed_time_in_segment = 0.0
            if self.use_speed_profile and self.speed_segments:
                if self._speed_schedule_filtered.size > 0:
                    initial_speed = float(self._speed_schedule_filtered[0])
                else:
                    segment0 = self._get_speed_segment(self.speed_idx)
                    initial_speed = segment0["start_speed"]
            else:
                initial_speed = 0.0
        
        self.state.target_speed_mps = initial_speed
        self.state.smoothed_target_speed_mps = initial_speed
        self.state.pid_integral = 0.0
        self.state.pid_prev_error = 0.0
        self.state.pid_prev_accel = 0.0
        self._pid_derivative_state = 0.0
        self._last_accel_cmd = 0.0
        self._speed_meas_filtered = 0.0

    def step(self, dt: float, current_speed_mps: float = None) -> float:
        """Advance driver by dt seconds and return requested traction power."""
        if dt <= 0.0:
            return self.state.p_req_filtered

        if current_speed_mps is None:
            current_speed_mps = 0.0

        self.state.current_time += dt
        # Filter measured speed to reduce noise in D action and error
        if self.speed_measurement_filter_tau_s > 0.0 and dt > 0.0:
            alpha_m = dt / (self.speed_measurement_filter_tau_s + dt)
            self._speed_meas_filtered = (
                (1.0 - alpha_m) * self._speed_meas_filtered + alpha_m * float(current_speed_mps)
            )
            self.state.speed_mps = self._speed_meas_filtered
        else:
            self.state.speed_mps = current_speed_mps

        # Choose control mode: timetable vs manual profiles
        if self.use_timetable:
            return self._step_timetable(dt, current_speed_mps)
        elif self.use_speed_profile:
            if self._speed_schedule_filtered.size > 0:
                target_speed, schedule_accel, segment_meta = self._sample_speed_schedule(self.state.current_time)
                planner_accel = self._plan_preview_accel(current_speed_mps, dt, schedule_accel)
                target_accel = planner_accel
            else:
                target_speed, target_accel, segment_meta = self._manual_speed_step(dt)
            self.state.target_speed_mps = target_speed
            self._update_smoothed_target_speed(target_speed, dt)
            self.state.is_dwelling = self.state.target_speed_mps <= self.dwell_speed_threshold or bool(segment_meta.get("dwell"))
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
        # Build pre-filtered schedule for planning / smoothing
        self._build_speed_schedule()

    def _get_speed_segment(self, idx: int) -> Dict[str, Any]:
        return self.speed_segments[idx]

    def _build_speed_schedule(self) -> None:
        if not self.speed_segments:
            self._speed_schedule_raw = np.array([], dtype=np.float64)
            self._speed_schedule_filtered = np.array([], dtype=np.float64)
            self._speed_schedule_meta = []
            self._speed_schedule_total_time = 0.0
            return

        samples: List[float] = []
        metas: List[Dict[str, Any]] = []
        dt = self.speed_profile_dt
        for seg in self.speed_segments:
            duration = float(seg["duration"])
            if duration <= 0.0:
                continue
            steps = max(1, int(np.ceil(duration / dt)))
            for step_idx in range(steps):
                progress = min(1.0, (step_idx * dt) / max(duration, 1e-6))
                if seg["hold"]:
                    speed = seg["end_speed"]
                elif seg["interp"] == "linear":
                    speed = seg["start_speed"] + (seg["end_speed"] - seg["start_speed"]) * progress
                else:
                    speed = seg["end_speed"]
                samples.append(float(speed))
                metas.append(seg["meta"])
        if not samples:
            self._speed_schedule_raw = np.array([], dtype=np.float64)
            self._speed_schedule_filtered = np.array([], dtype=np.float64)
            self._speed_schedule_meta = []
            self._speed_schedule_total_time = 0.0
            return
        self._speed_schedule_raw = np.asarray(samples, dtype=np.float64)
        self._speed_schedule_meta = metas
        self._speed_schedule_total_time = len(samples) * dt
        if self.speed_profile_filter_enable and self.speed_profile_filter_tau_s > 0.0:
            filtered = self._zero_phase_filter(self._speed_schedule_raw, dt, self.speed_profile_filter_tau_s)
        else:
            filtered = self._speed_schedule_raw.copy()
        self._speed_schedule_filtered = filtered

    @staticmethod
    def _zero_phase_filter(data: np.ndarray, dt: float, tau: float) -> np.ndarray:
        if data.size == 0:
            return data
        alpha = dt / (tau + dt)
        fwd = data.copy()
        for i in range(1, fwd.size):
            fwd[i] = fwd[i - 1] + alpha * (data[i] - fwd[i - 1])
        bwd = fwd.copy()
        for i in range(bwd.size - 2, -1, -1):
            bwd[i] = bwd[i + 1] + alpha * (fwd[i] - bwd[i + 1])
        return bwd

    def _sample_speed_schedule(self, time_s: float) -> Tuple[float, float, Dict[str, Any]]:
        if self._speed_schedule_filtered.size == 0:
            raise RuntimeError("Speed schedule not initialized")
        dt = self.speed_profile_dt
        total_time = max(self._speed_schedule_total_time, dt)
        if self.manual_loop:
            t = float(time_s % total_time)
        else:
            t = float(min(time_s, total_time - dt))
        idx = int(np.clip(t // dt, 0, self._speed_schedule_filtered.size - 1))
        frac = float(min(max((t - idx * dt) / dt, 0.0), 1.0))
        idx_next = min(idx + 1, self._speed_schedule_filtered.size - 1)
        speed0 = self._speed_schedule_filtered[idx]
        speed1 = self._speed_schedule_filtered[idx_next]
        speed = (1.0 - frac) * speed0 + frac * speed1
        accel = (speed1 - speed0) / dt
        meta = self._speed_schedule_meta[idx]
        return float(speed), float(accel), meta

    def _lookup_filtered_speed(self, time_s: float) -> float:
        if self._speed_schedule_filtered.size == 0:
            return 0.0
        dt = self.speed_profile_dt
        total_time = max(self._speed_schedule_total_time, dt)
        if self.manual_loop:
            t = float(time_s % total_time)
        else:
            t = float(min(time_s, total_time - dt))
        idx = int(np.clip(t // dt, 0, self._speed_schedule_filtered.size - 1))
        frac = float(min(max((t - idx * dt) / dt, 0.0), 1.0))
        idx_next = min(idx + 1, self._speed_schedule_filtered.size - 1)
        speed0 = self._speed_schedule_filtered[idx]
        speed1 = self._speed_schedule_filtered[idx_next]
        return float((1.0 - frac) * speed0 + frac * speed1)

    def _plan_preview_accel(self, current_speed_mps: float, dt: float, fallback_accel: float) -> float:
        if (
            not self.speed_planner_enable
            or self._speed_schedule_filtered.size == 0
            or dt <= 0.0
        ):
            return fallback_accel
        horizon = max(
            self.speed_planner_horizon_s,
            self.speed_planner_min_horizon_s,
            self.speed_preview_horizon_s,
            dt,
        )
        horizon_steps = max(1, int(round(horizon / dt)))
        v_sim = float(current_speed_mps)
        accel_sim = float(self._last_accel_cmd)
        planned_accel = fallback_accel
        for step in range(horizon_steps):
            t_future = self.state.current_time + (step + 1) * dt
            v_target = self._lookup_filtered_speed(t_future)
            remaining_steps = max(1, horizon_steps - step)
            t_remaining = remaining_steps * dt
            desired_accel = (v_target - v_sim) / max(t_remaining, dt)
            desired_accel = float(
                np.clip(
                    desired_accel,
                    -self.speed_tracking_brake_limit,
                    self.speed_tracking_accel_limit,
                )
            )
            if self.speed_tracking_jerk_limit is not None:
                max_delta = self.speed_tracking_jerk_limit * dt
                desired_accel = float(
                    np.clip(
                        desired_accel,
                        accel_sim - max_delta,
                        accel_sim + max_delta,
                    )
                )
            if self.speed_planner_penalty_jerk > 0.0:
                desired_accel = (
                    (1.0 - self.speed_planner_penalty_jerk) * desired_accel
                    + self.speed_planner_penalty_jerk * accel_sim
                )
            # Blend with fallback to penalize aggressive commands
            if self.speed_planner_penalty_accel > 0.0:
                desired_accel = (
                    (1.0 - self.speed_planner_penalty_accel) * desired_accel
                    + self.speed_planner_penalty_accel * fallback_accel
                )
            if step == 0:
                planned_accel = desired_accel
            v_sim += desired_accel * dt
            accel_sim = desired_accel
        return planned_accel

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
        # Use filtered measurement for error to reduce oscillations
        meas_speed = self.state.speed_mps
        error = target_speed - meas_speed
        # Apply error deadband (shrink to zero inside band)
        if self.speed_error_deadband_mps > 0.0:
            abs_e = abs(error)
            if abs_e <= self.speed_error_deadband_mps:
                error = 0.0
            else:
                error = np.sign(error) * (abs_e - self.speed_error_deadband_mps)
        if dt <= 0.0:
            dt = 1.0

        prev_integral = self.state.pid_integral
        integral = self.state.pid_integral
        # Integral separation: only integrate when sufficiently away from setpoint
        do_integrate = True
        if self.speed_integral_separation_mps > 0.0 and abs(error) < self.speed_integral_separation_mps:
            do_integrate = False
        # Disable integral while dwelling (helps avoid windup around zero speed)
        if self.state.is_dwelling:
            do_integrate = False
        if do_integrate:
            integral = self.state.pid_integral + error * dt
            if self.speed_pid_integral_limit > 0.0:
                integral = float(
                    np.clip(integral, -self.speed_pid_integral_limit, self.speed_pid_integral_limit)
                )
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
        # Anti-windup: revert integral if saturated and policy enabled
        if saturated and self.speed_disable_integral_when_saturated:
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
    
    def set_passenger_mass(self, mass_tons: float):
        """Set passenger mass for adhesion calculations."""
        self._passenger_mass_tons = mass_tons
    
    def _step_timetable(self, dt: float, current_speed_mps: float) -> float:
        """Step using timetable-based control."""
        if not self.timetable:
            return 0.0
        
        # Update position based on current speed
        self.state.current_position_m += current_speed_mps * dt
        
        # Check if at a stop
        at_stop = False
        dwell_time = 0.0
        next_stop_time = float('inf')
        
        if self.state.current_stop_idx < len(self.timetable.stops):
            stop = self.timetable.stops[self.state.current_stop_idx]
            distance_to_stop = stop.position_m - self.state.current_position_m
            
            # Check if we've arrived at stop (within threshold)
            if distance_to_stop <= 5.0 and abs(current_speed_mps) < 1.0:  # 5m threshold
                at_stop = True
                dwell_time = stop.dwell_time_s
                next_stop_time = self.state.current_time + dwell_time
                
                # Update next stop info for observation
                if self.state.current_stop_idx + 1 < len(self.timetable.stops):
                    next_stop = self.timetable.stops[self.state.current_stop_idx + 1]
                    self.state.next_stop_time = self.state.current_time + (next_stop.position_m - self.state.current_position_m) / 15.0
                else:
                    self.state.next_stop_time = float('inf')
        
        # Get current route segment
        current_segment = None
        if self.state.current_route_segment_idx < len(self.timetable.route_segments):
            current_segment = self.timetable.route_segments[self.state.current_route_segment_idx]
            
            # Update route segment index if we've passed this segment
            while (current_segment and 
                   self.state.current_position_m >= current_segment.end_position_m and 
                   self.state.current_route_segment_idx + 1 < len(self.timetable.route_segments)):
                self.state.current_route_segment_idx += 1
                current_segment = self.timetable.route_segments[self.state.current_route_segment_idx]
        
        # Determine target speed based on speed limit and schedule
        speed_limit = current_segment.speed_limit_mps if current_segment else 30.0
        
        if at_stop:
            target_speed = 0.0
            self.state.is_dwelling = True
            self.state.dwell_end_time = next_stop_time
        else:
            target_speed = speed_limit
            self.state.is_dwelling = False
            self.state.dwell_end_time = 0.0
        
        # Use PID control to track target speed
        speed_error = target_speed - current_speed_mps
        pid_output = (
            self.config.pid_kp * speed_error +
            self.config.pid_ki * self.state.pid_integral +
            self.config.pid_kd * (speed_error - self.state.pid_prev_error)
        )
        
        # Update PID state
        if not at_stop:  # Don't accumulate integral during dwell
            self.state.pid_integral += speed_error * dt
            self.state.pid_integral = np.clip(
                self.state.pid_integral, -10.0, 10.0  # Anti-windup
            )
        else:
            self.state.pid_integral = 0.0
        
        self.state.pid_prev_error = speed_error
        
        # Convert PID output to force (N) then power (kW)
        target_force = pid_output * 1000.0  # Scale to reasonable force
        
        # Apply adhesion limits
        if hasattr(self, '_passenger_mass_tons'):
            mass_kg = self._passenger_mass_tons * 1000.0
        else:
            mass_kg = 220000.0  # Default 220 tons
        
        adhesion_mu = self.rng.uniform(
            self.config.adhesion_mu_min, 
            self.config.adhesion_mu_max
        )
        max_force = adhesion_mu * mass_kg * 9.81
        target_force = np.clip(target_force, -max_force, max_force)
        
        # Convert to requested power
        if current_speed_mps > 0.5:  # Avoid division by very small numbers
            p_req_kw = (target_force * current_speed_mps) / 1000.0
        else:
            # At very low speeds, use force directly
            p_req_kw = np.sign(target_force) * min(abs(target_force * 5.0 / 1000.0), 100.0)
        
        # Apply power limits
        p_req_kw = np.clip(
            p_req_kw, 
            -self.config.regen_power_max_kw, 
            self.config.traction_power_max_kw
        )
        
        # Apply grade effect (simplified - additional power needed for grades)
        if current_segment:
            grade_power_kw = (mass_kg * 9.81 * current_segment.grade_percent / 100.0 * current_speed_mps) / 1000.0
            p_req_kw += grade_power_kw
        
        return p_req_kw
