"""Env0: Supervisory EMS (POMDP) Gymnasium environment for hybrid FC–Battery train."""
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Dict, Tuple, Optional, Any
from dataclasses import dataclass
from collections import deque

from .config import Config
from .driver import Driver
from .plant import Plant
from .shield import Shield, ShieldedAction


@dataclass
class EpisodeInfo:
    """Episode-level information for logging."""
    total_reward: float = 0.0
    total_cost_h2: float = 0.0
    total_cost_grid: float = 0.0
    total_penalty_unmet: float = 0.0
    total_penalty_delay: float = 0.0
    total_penalty_smooth: float = 0.0
    constraint_violations: int = 0
    distance_km: float = 0.0
    delta_m_h2_kg: float = 0.0
    delta_e_charge_kwh: float = 0.0


class Env0(gym.Env):
    """
    Env0: Supervisory EMS (POMDP) environment for hybrid FC–Battery train.
    
    Observation: Box(low=-1, high=1, shape=(12,), dtype=float32)
    Action: Box(low=[0,-1], high=[1,1], dtype=float32) → [fc_frac, batt_cmd]
    """
    
    metadata = {"render_modes": ["human", "rgb_array", "ansi"], "render_fps": 1}
    
    def __init__(self, config: Config, seed: Optional[int] = None):
        """
        Initialize Env0 environment.
        
        Args:
            config: Configuration object
            seed: Random seed (overrides config if provided)
        """
        super().__init__()
        self.config = config
        self.rng = np.random.default_rng(seed if seed is not None else config.sim.seed)
        
        # Initialize components
        self.driver = Driver(
            config.driver,
            config.randomization,
            self.rng,
            plant_kinematic_gain_mps_per_watt=config.plant.kinematic_gain_mps_per_watt
        )
        self.plant = Plant(
            config.plant,
            config.battery,
            config.fuel_cell,
            self.rng
        )
        self.shield = Shield(
            config.shield,
            config.battery,
            config.fuel_cell
        )
        
        # Observation space: Box(low=-1, high=1, shape=(12,), dtype=float32)
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),
            dtype=np.float32
        )
        
        # Action space: Box(low=[0,-1], high=[1,1], dtype=float32)
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )
        
        # Episode state
        self.episode_step = 0
        self.episode_length = 0
        self.last_action = np.array([0.5, 0.0], dtype=np.float32)
        self.p_req_filtered = 0.0  # Low-pass filtered P_req for observation
        self.p_req_next_kw = 0.0  # Pre-computed P_req for next step (fixes timing lag)
        self.p_req_prev_filtered = 0.0  # Previous filtered P_req for trend calculation
        self.episode_info = EpisodeInfo()
        
        # Normalization ranges (for observation)
        self.v_max = config.plant.v_max_mps
        self.p_req_max = config.observations.p_req_max_kw
        
        # Render buffer (stores full episode history)
        if config.renderer.enabled:
            self._render_buf = deque()  # No maxlen - stores full history
            self._render_step_counter = 0
            # Store per-step costs for renderer
            self._step_costs = {
                "cost_h2_eur": 0.0,
                "cost_grid_eur": 0.0,
                "penalty_delay_eur": 0.0,
                "penalty_unmet_eur": 0.0,
                "penalty_smooth_eur": 0.0,
            }
            # Delay tracking
            self._scheduled_time = 0.0  # Expected time based on schedule
            self._cumulative_delay_s = 0.0
        else:
            self._render_buf = None
            self._render_step_counter = 0
        
        # Initialize renderer if enabled
        if config.renderer.enabled:
            if config.renderer.use_enhanced_renderer:
                from .enhanced_renderer import EnhancedRenderer
                self._renderer = EnhancedRenderer(config.renderer)
            else:
                from .renderer import Renderer
                self._renderer = Renderer(config.renderer)
        else:
            self._renderer = None
        
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Reset environment for new episode.
        
        Returns:
            observation: Initial observation
            info: Info dict
        """
        if seed is not None:
            self.rng = np.random.default_rng(seed)
            self.driver.rng = self.rng
            self.plant.rng = self.rng
        
        # Randomize episode parameters
        passenger_mass_tons = self.rng.uniform(
            self.config.randomization.passenger_mass_tons_min,
            self.config.randomization.passenger_mass_tons_max
        )
        
        if self.config.randomization.soc_init_uniform:
            soc_init = self.rng.uniform(
                self.config.battery.soc_init_min,
                self.config.battery.soc_init_max
            )
        else:
            soc_init = (self.config.battery.soc_init_min + self.config.battery.soc_init_max) / 2.0
        
        if self.config.randomization.tank_init_uniform:
            tank_init = self.rng.uniform(
                self.config.fuel_cell.tank_init_min,
                self.config.fuel_cell.tank_init_max
            )
        else:
            tank_init = (self.config.fuel_cell.tank_init_min + self.config.fuel_cell.tank_init_max) / 2.0
        
        # Reset components
        self.driver.reset()
        self.driver.set_passenger_mass(passenger_mass_tons)
        self.plant.reset(
            soc_init=soc_init,
            tank_init=tank_init,
            passenger_mass_tons=passenger_mass_tons,
            aux_bias_sigma_kw=self.config.randomization.aux_bias_rw_sigma_kw
        )
        self.shield.reset()
        
        # Episode state
        self.episode_step = 0
        self.episode_length = self.rng.integers(
            self.config.sim.episode_steps_min,
            self.config.sim.episode_steps_max + 1
        )
        self.last_action = np.array([0.5, 0.0], dtype=np.float32)
        self.p_req_filtered = 0.0
        self.p_req_prev_filtered = 0.0
        self.episode_info = EpisodeInfo()
        
        # Reset delay tracking
        if self.config.renderer.enabled:
            self._scheduled_time = 0.0
            self._cumulative_delay_s = 0.0
            self._render_buf.clear()
            self._render_step_counter = 0
        
        # Pre-compute P_req for first step (fixes timing lag)
        self.p_req_next_kw = self.driver.step(self.config.sim.dt_seconds, current_speed_mps=self.plant.state.speed_mps)
        # Update filtered P_req (save previous for trend calculation)
        self.p_req_prev_filtered = self.p_req_filtered
        alpha = self.config.sim.dt_seconds / (
            self.config.driver.p_req_smoothing_tau_s + self.config.sim.dt_seconds
        )
        self.p_req_filtered = alpha * self.p_req_next_kw + (1.0 - alpha) * self.p_req_filtered

        # Get initial observation
        obs = self._get_observation()
        info = self._get_info()

        # Capture initial render data
        if self.config.renderer.enabled:
            self._capture_render_data(info, 0.0, None)

        return obs, info
    
    def step(
        self,
        action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Step environment forward.
        
        Args:
            action: [fc_frac, batt_cmd]
        
        Returns:
            observation: Next observation
            reward: Reward for this step
            terminated: Episode terminated (hard stop)
            truncated: Episode truncated (time limit)
            info: Info dict
        """
        # Clip action to valid range
        action = np.clip(action, self.action_space.low, self.action_space.high)
        
        # Apply shield
        shielded = self.shield.apply(
            fc_frac=float(action[0]),
            batt_cmd=float(action[1]),
            soc=self.plant.state.soc,
            tank_level=self.plant.state.tank_level
        )

        # Use pre-computed P_req for this step (fixes timing lag - EMS saw this P_req when deciding action)
        p_req_kw = self.p_req_next_kw
        
        # Update delay tracking (compare actual time vs scheduled time)
        if self.config.renderer.enabled:
            self._scheduled_time += self.config.sim.dt_seconds
            target_speed = self.driver.state.target_speed_mps
            if (
                not self.driver.state.is_dwelling
                and abs(target_speed) > 1e-6
            ):
                speed_error = target_speed - self.plant.state.speed_mps
                if speed_error > 0.5:  # Significantly behind target
                    self._cumulative_delay_s += speed_error * self.config.sim.dt_seconds / target_speed
                elif speed_error < -0.5:  # Ahead of schedule
                    self._cumulative_delay_s = max(0.0, self._cumulative_delay_s + speed_error * self.config.sim.dt_seconds / target_speed)

        # Advance plant
        p_loss_kw = self.config.driver.p_loss_watts / 1000.0
        self.plant.step(
            p_fc_kw=shielded.p_fc_kw,
            p_batt_kw=shielded.p_batt_kw,
            p_req_kw=p_req_kw,
            dt_seconds=self.config.sim.dt_seconds,
            p_loss_kw=p_loss_kw
        )
        
        # Force speed to 0 during dwell (per spec)
        if self.driver.state.is_dwelling:
            self.plant.state.speed_mps = 0.0
        
        # Compute reward (before updating last_action)
        reward = self._compute_reward(shielded, p_req_kw, action)
        
        # Update episode info
        self.episode_info.total_reward += reward
        self.episode_info.constraint_violations += shielded.violations
        
        # Store last action (for next step's smoothness penalty)
        self.last_action = action.copy()
        
        # Check termination conditions
        terminated = (
            self.plant.state.soc <= self.config.battery.soc_hard_min or
            self.plant.state.tank_level <= self.config.fuel_cell.tank_hard_min
        )
        
        # Check truncation (time limit)
        self.episode_step += 1
        truncated = self.episode_step >= self.episode_length

        # Pre-compute P_req for NEXT step (fixes timing lag - EMS will see this P_req when deciding next action)
        self.p_req_next_kw = self.driver.step(self.config.sim.dt_seconds, current_speed_mps=self.plant.state.speed_mps)
        # Update filtered P_req for observation (save previous for trend calculation)
        self.p_req_prev_filtered = self.p_req_filtered
        alpha = self.config.sim.dt_seconds / (
            self.config.driver.p_req_smoothing_tau_s + self.config.sim.dt_seconds
        )
        self.p_req_filtered = alpha * self.p_req_next_kw + (1.0 - alpha) * self.p_req_filtered

        # Get observation and info
        obs = self._get_observation()
        info = self._get_info()

        # Capture render data
        if self.config.renderer.enabled:
            self._capture_render_data(info, reward, shielded)

        return obs, reward, terminated, truncated, info
    
    def _get_observation(self) -> np.ndarray:
        """Build observation vector (normalized to [-1, 1])."""
        obs = np.zeros(12, dtype=np.float32)
        
        # 1. Speed (normalized)
        obs[0] = np.clip(
            (self.plant.state.speed_mps / self.v_max) * 2.0 - 1.0,
            -1.0, 1.0
        )
        
        # 2. SOC estimate (noisy)
        soc_noise = self.rng.normal(0.0, self.config.observations.soc_noise_std)
        soc_obs = np.clip(self.plant.state.soc + soc_noise, 0.0, 1.0)
        obs[1] = soc_obs * 2.0 - 1.0  # [0,1] -> [-1,1]
        
        # 3. H2 tank level proxy (noisy, normalized)
        tank_noise = self.rng.normal(0.0, self.config.observations.tank_noise_std)
        tank_obs = np.clip(self.plant.state.tank_level + tank_noise, 0.0, 1.0)
        obs[2] = tank_obs * 2.0 - 1.0
        
        # 4. Time/Distance to next stop (normalized)
        # Use time to next stop if available, otherwise episode phase
        if hasattr(self.driver.state, 'next_stop_time') and self.driver.state.next_stop_time > self.driver.state.current_time:
            time_to_stop = self.driver.state.next_stop_time - self.driver.state.current_time
            # Normalize assuming max 10 minutes to next stop
            obs[3] = np.clip((time_to_stop / 600.0) * 2.0 - 1.0, -1.0, 1.0)
        else:
            # Fallback to episode phase
            phase = self.episode_step / max(1, self.episode_length)
            obs[3] = phase * 2.0 - 1.0
        
        # 5. Grade preview/segment indicator (normalized)
        # Get current segment grade if available
        current_grade = 0.0
        if hasattr(self.driver, 'speed_segments') and self.driver.speed_segments:
            if self.driver.speed_idx < len(self.driver.speed_segments):
                segment_meta = self.driver.speed_segments[self.driver.speed_idx].get('meta', {})
                current_grade = segment_meta.get('grade_percent', 0.0)
        # Normalize assuming grades in range [-5%, +5%]
        obs[4] = np.clip((current_grade / 5.0) * 2.0 - 1.0, -1.0, 1.0)
        
        # 6. Requested traction power P_req (filtered, normalized)
        obs[5] = np.clip(
            (self.p_req_filtered / self.p_req_max) * 2.0 - 1.0,
            -1.0, 1.0
        )
        
        # 7. P_req trend (derivative, normalized)
        p_req_trend = (self.p_req_filtered - self.p_req_prev_filtered) / self.config.sim.dt_seconds
        # Normalize assuming max change of 1000 kW/s
        obs[6] = np.clip((p_req_trend / 1000.0) * 2.0 - 1.0, -1.0, 1.0)
        
        # 8-9. Last action
        obs[7] = self.last_action[0] * 2.0 - 1.0  # fc_frac [0,1] -> [-1,1]
        obs[8] = self.last_action[1]  # batt_cmd already [-1,1]
        
        # 10-12. Nuisance noise channels
        noise_std = self.config.observations.nuisance_noise_std
        obs[9] = self.rng.normal(0.0, noise_std)
        obs[10] = self.rng.normal(0.0, noise_std)
        obs[11] = self.rng.normal(0.0, noise_std)
        
        # Clip to [-1, 1]
        obs = np.clip(obs, -1.0, 1.0)
        
        return obs.astype(np.float32)
    
    def _compute_reward(
        self,
        shielded: ShieldedAction,
        p_req_kw: float,
        action: np.ndarray
    ) -> float:
        """
        Compute reward for current step.
        
        Reward formula:
        r = -(c_h2 * Δm_H2 + c_grid * ΔE_charge_kWh + λ_smooth * ||a_t - a_{t-1}|| + λ_delay * Δt * 1[behind_schedule])
            - λ_unmet * P_unmet
        """
        dt_hours = self.config.sim.dt_seconds / 3600.0
        
        # H2 cost
        if shielded.p_fc_kw > 0:
            delta_m_h2_kg = (
                shielded.p_fc_kw / 
                (self.config.fuel_cell.eta_fc * self.config.fuel_cell.h2_lhv_kwh_per_kg)
            ) * dt_hours
        else:
            delta_m_h2_kg = 0.0
        
        cost_h2 = self.config.costs.c_h2_eur_per_kg * delta_m_h2_kg
        self.episode_info.total_cost_h2 += cost_h2
        self.episode_info.delta_m_h2_kg += delta_m_h2_kg
        
        # Grid cost (net charging only)
        p_batt_charge = max(-shielded.p_batt_kw, 0.0)
        delta_e_charge_kwh = p_batt_charge * dt_hours * self.config.battery.eta_charge
        cost_grid = self.config.costs.c_grid_eur_per_kwh * delta_e_charge_kwh
        self.episode_info.total_cost_grid += cost_grid
        self.episode_info.delta_e_charge_kwh += delta_e_charge_kwh
        
        # Smoothness penalty (compare current action to last action)
        # Use raw action (before shield) for smoothness penalty
        action_diff = np.linalg.norm(self.last_action - action)
        penalty_smooth = self.config.reward_weights.lambda_smooth * action_diff
        self.episode_info.total_penalty_smooth += penalty_smooth
        
        # Delay penalty (based on cumulative delay)
        if self.config.renderer.enabled:
            delay_seconds = self._cumulative_delay_s
        else:
            delay_seconds = 0.0
        penalty_delay = self.config.reward_weights.lambda_delay * delay_seconds * self.config.sim.dt_seconds
        self.episode_info.total_penalty_delay += penalty_delay
        
        # Unmet demand penalty
        penalty_unmet = self.config.reward_weights.lambda_unmet * self.plant.state.p_unmet_kw
        self.episode_info.total_penalty_unmet += penalty_unmet
        
        # Total reward
        reward = -(
            cost_h2 +
            cost_grid +
            penalty_smooth +
            penalty_delay
        ) - penalty_unmet
        
        # Store per-step costs for renderer
        if self.config.renderer.enabled:
            self._step_costs = {
                "cost_h2_eur": cost_h2,
                "cost_grid_eur": cost_grid,
                "penalty_delay_eur": penalty_delay,
                "penalty_unmet_eur": penalty_unmet,
                "penalty_smooth_eur": penalty_smooth,
            }
        
        return float(reward)
    
    def _get_info(self) -> Dict[str, Any]:
        """Get info dict for current step."""
        info = {
            "step": self.episode_step,
            "soc": float(self.plant.state.soc),
            "tank_level": float(self.plant.state.tank_level),
            "speed_mps": float(self.plant.state.speed_mps),
            "p_req_kw": float(self.p_req_filtered),
            "p_fc_kw": float(self.plant.state.p_fc_kw),
            "p_batt_kw": float(self.plant.state.p_batt_kw),
            "p_unmet_kw": float(self.plant.state.p_unmet_kw),
            "p_dem_kw": float(self.plant.state.p_dem_kw),
            "p_supply_kw": float(self.plant.state.p_supply_kw),
            "p_delivered_kw": float(self.plant.state.p_delivered_kw),
            "p_loss_kw": float(self.plant.state.p_loss_kw),
            "p_batt_discharge_kw": float(self.plant.state.p_batt_discharge_kw),
            "p_batt_charge_kw": float(self.plant.state.p_batt_charge_kw),
            "p_batt_delivered_kw": float(self.plant.state.p_batt_delivered_kw),
            "p_regen_post_aux_kw": float(self.plant.state.p_regen_post_aux_kw),
            "p_brake_total_kw": float(self.plant.state.p_brake_total_kw),
            "p_brake_regen_kw": float(self.plant.state.p_brake_regen_kw),
            "p_brake_aux_kw": float(self.plant.state.p_brake_aux_kw),
            "p_brake_friction_kw": float(self.plant.state.p_brake_friction_kw),
            "distance_km": float(self.plant.state.distance_km),
            "constraint_violations": self.episode_info.constraint_violations,
            "p_batt_charge_regen_kw": float(self.plant.state.p_batt_charge_regen_kw),
            "p_batt_charge_fc_kw": float(self.plant.state.p_batt_charge_fc_kw),
        }
        
        # Add renderer-required fields
        if self.config.renderer.enabled:
            info.update({
                "target_speed_mps": float(self.driver.state.target_speed_mps),
                "at_stop": bool(self.driver.state.is_dwelling),
                "dwell_remaining_s": max(0.0, float(self.driver.state.dwell_end_time - self.driver.state.current_time)) if self.driver.state.is_dwelling else 0.0,
                "p_aux_kw": float(self.plant.state.aux_bias_kw + self.config.plant.p_aux_base_watts / 1000.0),
                "cost_h2_eur": self._step_costs["cost_h2_eur"],
                "cost_grid_eur": self._step_costs["cost_grid_eur"],
                "penalty_delay_eur": self._step_costs["penalty_delay_eur"],
                "penalty_unmet_eur": self._step_costs["penalty_unmet_eur"],
                "penalty_smooth_eur": self._step_costs["penalty_smooth_eur"],
                "delay_seconds": self._cumulative_delay_s,
                "passenger_mass_t": float(self.plant.passenger_mass_tons),
                "aux_bias_kw": float(self.plant.state.aux_bias_kw),
            })
        
        info["power_flow"] = {
            "p_req_kw": info["p_req_kw"],
            "p_dem_kw": info["p_dem_kw"],
            "p_supply_kw": info["p_supply_kw"],
            "p_loss_kw": info["p_loss_kw"],
            "p_unmet_kw": info["p_unmet_kw"],
            "p_aux_kw": info.get("p_aux_kw", 0.0),
            "p_fc_kw": info["p_fc_kw"],
            "p_batt_discharge_kw": info["p_batt_discharge_kw"],
            "p_batt_charge_kw": info["p_batt_charge_kw"],
            "p_batt_charge_regen_kw": info["p_batt_charge_regen_kw"],
            "p_batt_charge_fc_kw": info["p_batt_charge_fc_kw"],
            "p_regen_post_aux_kw": info["p_regen_post_aux_kw"],
            "p_delivered_kw": info["p_delivered_kw"],
            "p_brake_total_kw": info["p_brake_total_kw"],
            "p_brake_regen_kw": info["p_brake_regen_kw"],
            "p_brake_aux_kw": info["p_brake_aux_kw"],
            "p_brake_friction_kw": info["p_brake_friction_kw"],
        }
        
        return info
    
    def _capture_render_data(self, info: Dict[str, Any], reward: float, shielded: Optional[ShieldedAction]):
        """Capture render data for current step."""
        if not self.config.renderer.enabled:
            return
        
        render_data = {
            "t": self.episode_step * self.config.sim.dt_seconds,
            "dt": self.config.sim.dt_seconds,
            "episode_step": self.episode_step,
            "episode_length": self.episode_length,
            "speed_mps": info["speed_mps"],
            "target_speed_mps": info.get("target_speed_mps", 0.0),
            "at_stop": info.get("at_stop", False),
            "dwell_remaining_s": info.get("dwell_remaining_s", 0.0),
            "p_req_kw": info["p_req_kw"],
            "p_fc_kw": info["p_fc_kw"],
            "p_batt_kw": info["p_batt_kw"],
            "p_batt_discharge_kw": info.get("p_batt_discharge_kw", max(info["p_batt_kw"], 0.0)),
            "p_batt_charge_kw": info.get("p_batt_charge_kw", max(-info["p_batt_kw"], 0.0)),
            "p_batt_charge_regen_kw": info.get("p_batt_charge_regen_kw", 0.0),
            "p_batt_charge_fc_kw": info.get("p_batt_charge_fc_kw", 0.0),
            "p_batt_delivered_kw": info.get("p_batt_delivered_kw", max(info["p_batt_kw"], 0.0)),
            "p_aux_kw": info.get("p_aux_kw", 0.0),
            "p_unmet_kw": info["p_unmet_kw"],
            "p_dem_kw": info.get("p_dem_kw", info["p_req_kw"] + info.get("p_aux_kw", 0.0)),
            "p_supply_kw": info.get("p_supply_kw", info["p_fc_kw"] + max(info["p_batt_kw"], 0.0)),
            "p_loss_kw": info.get("p_loss_kw", self.config.driver.p_loss_watts / 1000.0),
            "p_regen_post_aux_kw": info.get("p_regen_post_aux_kw", 0.0),
            "soc": info["soc"],
            "tank_level_norm": info["tank_level"],
            "p_brake_total_kw": info.get("p_brake_total_kw", 0.0),
            "p_brake_regen_kw": info.get("p_brake_regen_kw", 0.0),
            "p_brake_aux_kw": info.get("p_brake_aux_kw", 0.0),
            "p_brake_friction_kw": info.get("p_brake_friction_kw", 0.0),
            "cost_h2_eur": info.get("cost_h2_eur", 0.0),
            "cost_grid_eur": info.get("cost_grid_eur", 0.0),
            "penalty_delay_eur": info.get("penalty_delay_eur", 0.0),
            "penalty_unmet_eur": info.get("penalty_unmet_eur", 0.0),
            "penalty_smooth_eur": info.get("penalty_smooth_eur", 0.0),
            "r_step": reward,
            "soc_low": shielded.soc_low if shielded else False,
            "soc_high": shielded.soc_high if shielded else False,
            "fc_ramp_limited": shielded.fc_ramp_limited if shielded else False,
            "crate_capped": shielded.crate_capped if shielded else False,
            "regen_clipped": shielded.regen_clipped if shielded else False,
            "passenger_mass_t": info.get("passenger_mass_t", 0.0),
            "aux_bias_kw": info.get("aux_bias_kw", 0.0),
            "distance_km": info["distance_km"],
            "delay_seconds": info.get("delay_seconds", 0.0),
        }
        
        self._render_buf.append(render_data)
    
    def render(self, mode: str = "human"):
        """
        Render environment.
        
        Args:
            mode: Render mode - "human", "rgb_array", or "ansi"
        
        Returns:
            None for human mode, RGB array for rgb_array mode, string for ansi mode
        """
        if not self.config.renderer.enabled or self._renderer is None:
            return None
        
        # Check render_every counter
        self._render_step_counter += 1
        if self._render_step_counter < self.config.renderer.render_every:
            return None
        self._render_step_counter = 0
        
        # Dispatch to appropriate renderer method
        if mode == "human":
            self._renderer.render_human(self._render_buf)
            return None
        elif mode == "rgb_array":
            return self._renderer.render_rgb_array(self._render_buf)
        elif mode == "ansi":
            if len(self._render_buf) > 0:
                return self._renderer.render_ansi(self._render_buf[-1])
            return ""
        else:
            raise ValueError(f"Unknown render mode: {mode}")
    
    def close(self):
        """Clean up renderer resources."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
