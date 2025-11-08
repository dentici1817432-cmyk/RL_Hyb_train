"""Configuration loading and validation."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Any

import yaml


# ---------------------------------------------------------------------------
# Atomic config sections
# ---------------------------------------------------------------------------


@dataclass
class SimConfig:
    dt_seconds: float = 1.0
    episode_steps_min: int = 1200
    episode_steps_max: int = 1800
    seed: Optional[int] = None


@dataclass
class RouteConfig:
    profiles: List[str] = field(default_factory=lambda: ["flat", "rolling", "mild_mountain"])
    grade_segments_percent: List[float] = field(default_factory=lambda: [0.02, 0.00, -0.01])
    segment_duration_s: float = 600.0
    speed_limits_mps: List[float] = field(default_factory=lambda: [40.0, 35.0, 30.0])


@dataclass
class StopsConfig:
    count: int = 3
    dwell_mean_s: float = 60.0
    dwell_jitter_frac: float = 0.2


@dataclass
class DrivelineConfig:
    eta_traction: float = 0.85
    eta_regen: float = 0.75
    aux_base_kw: float = 200.0
    aux_bias_rw_sigma_kw: float = 0.1


@dataclass
class DriverConfig:
    p_req_smoothing_tau_s: float = 2.0
    p_req_rate_limit_kw_per_s: float = 400.0
    p_loss_watts: float = 200000.0
    manual_loop: bool = False
    manual_p_req_profile: List[Dict[str, float]] = field(default_factory=list)
    manual_speed_profile: List[Dict[str, float]] = field(default_factory=list)
    # New spec-compliant driver parameters
    pid_kp: float = 5.0
    pid_ki: float = 0.0
    pid_kd: float = 1.5
    adhesion_mu_min: float = 0.2
    adhesion_mu_max: float = 0.35
    traction_power_max_kw: float = 1000.0
    regen_power_max_kw: float = 400.0
    req_lpf_tau_s: float = 3.0
    trend_window_s: float = 3.0
    speed_target_smoothing_tau_s: float = 8.0
    speed_tracking_tau_s: float = 12.0
    speed_tracking_accel_limit_mps2: float = 0.5
    speed_tracking_brake_limit_mps2: float = 0.6
    dwell_speed_threshold_mps: float = 0.5
    speed_control_power_clip_kw: Optional[float] = None
    speed_profile_interpolation: str = "linear"
    speed_tracking_jerk_limit_mps3: Optional[float] = 0.6
    speed_tracking_use_pid: bool = True
    speed_pid_kp: float = 0.6
    speed_pid_ki: float = 0.05
    speed_pid_kd: float = 0.08
    speed_pid_integral_limit: float = 1.5
    speed_pid_derivative_filter_tau_s: float = 0.6
    # Trajectory filtering + preview planner knobs
    speed_profile_dt_seconds: float = 1.0
    speed_profile_filter_enable: bool = True
    speed_profile_filter_tau_s: float = 6.0
    speed_preview_horizon_s: float = 12.0
    speed_planner_enable: bool = True
    speed_planner_horizon_s: float = 10.0
    speed_planner_min_horizon_s: float = 4.0
    speed_planner_penalty_accel: float = 0.05
    speed_planner_penalty_jerk: float = 0.05
    # Smoothing and robustness additions for speed controller
    speed_error_deadband_mps: float = 0.15
    speed_integral_separation_mps: float = 0.6
    speed_disable_integral_when_saturated: bool = True
    speed_measurement_filter_tau_s: float = 4.0


@dataclass
class TrainConfig:
    """Train physics parameters for realistic dynamics."""
    mass_tons_min: float = 180.0
    mass_tons_max: float = 260.0
    davis_A_N: float = 5000.0  # Rolling resistance constant
    davis_B_N_per_mps: float = 100.0  # Rolling resistance linear term  
    davis_C_N_per_mps2: float = 5.0  # Rolling resistance quadratic term


@dataclass
class PlantConfig:
    v_max_mps: float = 50.0
    kinematic_gain_mps_per_watt: float = 2.5e-6
    p_aux_base_watts: float = 200000.0


@dataclass
class BatteryConfig:
    e_batt_kwh: float = 300.0
    p_batt_max_discharge_kw: float = 600.0
    p_batt_max_charge_kw: float = 300.0
    eta_charge: float = 0.95
    eta_discharge: float = 0.94
    soc_init_min: float = 0.55
    soc_init_max: float = 0.80
    soc_hard_min: float = 0.15
    soc_hard_max: float = 1.00


@dataclass
class FuelCellConfig:
    p_fc_max_kw: float = 400.0
    eta_fc: float = 0.50
    h2_lhv_kwh_per_kg: float = 33.3
    tank_capacity_kg: float = 50.0
    tank_init_min: float = 0.70
    tank_init_max: float = 1.00
    tank_hard_min: float = 0.02
    ramp_kw_per_s: float = 40.0


@dataclass
class ShieldConfig:
    soc_soft_min: float = 0.20
    soc_soft_max: float = 0.90
    enforce_soc_corridor: bool = True
    enforce_fc_ramp: bool = True
    enforce_crate_caps: bool = True


@dataclass
class CostsConfig:
    c_h2_eur_per_kg: float = 6.0
    c_grid_eur_per_kwh: float = 0.18


@dataclass
class RewardWeightsConfig:
    lambda_smooth: float = 0.01
    lambda_delay: float = 0.5
    lambda_unmet: float = 1.0e-6


@dataclass
class RandomizationConfig:
    passenger_mass_tons_min: float = 180.0
    passenger_mass_tons_max: float = 260.0
    aux_bias_rw_sigma_kw: float = 0.10
    soc_init_uniform: bool = True
    tank_init_uniform: bool = True
    timetable_jitter_enable: bool = True


@dataclass
class ObservationsConfig:
    include_p_req_kw: bool = True
    include_last_action: bool = True
    include_noise_channels: int = 3
    normalize_to_unit_box: bool = True
    p_req_max_kw: float = 2000.0
    soc_noise_std: float = 0.02
    tank_noise_std: float = 0.02
    nuisance_noise_std: float = 0.1


@dataclass
class RendererConfig:
    enabled: bool = True
    mode: str = "human"
    render_every: int = 5
    rolling_window_s: int = 180
    dpi: int = 110
    figsize: List[float] = field(default_factory=lambda: [12.0, 7.0])
    show_hidden_debug: bool = False
    draw_schedule: bool = True
    write_video: bool = False
    video_path: str = "runs/env0_episode.mp4"
    write_png_every: int = 0
    interactive: bool = False
    use_enhanced_renderer: bool = True  # Enable enhanced renderer with 3D visualization
    colors: Dict[str, str] = field(
        default_factory=lambda: {
            "demand": "#000000",
            "fc": "#1f77b4",
            "batt_dis": "#ff7f0e",
            "batt_chg": "#2ca02c",
            "unmet": "#d62728",
            "soc_band": "#e5e5e5",
        }
    )


@dataclass
class LoggingConfig:
    log_terms: List[str] = field(default_factory=list)
    trace_signals: List[str] = field(default_factory=list)
    kpi_aggregates: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Bundled modalities
# ---------------------------------------------------------------------------


@dataclass
class TrainModelConfig:
    """Parameters that define the physical train model."""

    train: TrainConfig = field(default_factory=TrainConfig)
    plant: PlantConfig = field(default_factory=PlantConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    fuel_cell: FuelCellConfig = field(default_factory=FuelCellConfig)
    shield: ShieldConfig = field(default_factory=ShieldConfig)
    costs: CostsConfig = field(default_factory=CostsConfig)


@dataclass
class ScenarioConfig:
    """Scenario definition: track, schedule, observation model."""

    sim: SimConfig = field(default_factory=SimConfig)
    route: RouteConfig = field(default_factory=RouteConfig)
    stops: StopsConfig = field(default_factory=StopsConfig)
    driveline: DrivelineConfig = field(default_factory=DrivelineConfig)
    driver: DriverConfig = field(default_factory=DriverConfig)
    reward_weights: RewardWeightsConfig = field(default_factory=RewardWeightsConfig)
    randomization: RandomizationConfig = field(default_factory=RandomizationConfig)
    observations: ObservationsConfig = field(default_factory=ObservationsConfig)
    renderer: RendererConfig = field(default_factory=RendererConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


@dataclass
class PolicyConfig:
    """Policy selection and tuning knobs."""

    default: str = "baseline"
    baseline: Dict[str, Any] = field(default_factory=dict)
    balanced: Dict[str, Any] = field(default_factory=dict)
    scenario: Dict[str, Any] = field(default_factory=dict)
    mpc: Dict[str, Any] = field(default_factory=dict)
    rl: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Root config with backward-compatible accessors
# ---------------------------------------------------------------------------


@dataclass
class Config:
    train: TrainModelConfig = field(default_factory=TrainModelConfig)
    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)
    policy: PolicyConfig = field(default_factory=PolicyConfig)

    # Backwards-compatible attribute accessors --------------------------------
    @property
    def sim(self) -> SimConfig:
        return self.scenario.sim

    @property
    def driver(self) -> DriverConfig:
        return self.scenario.driver

    @property
    def route(self) -> RouteConfig:
        return self.scenario.route

    @property
    def stops(self) -> StopsConfig:
        return self.scenario.stops

    @property
    def driveline(self) -> DrivelineConfig:
        return self.scenario.driveline

    @property
    def train_physics(self) -> TrainConfig:
        return self.train.train

    @property
    def plant(self) -> PlantConfig:
        return self.train.plant

    @property
    def battery(self) -> BatteryConfig:
        return self.train.battery

    @property
    def fuel_cell(self) -> FuelCellConfig:
        return self.train.fuel_cell

    @property
    def shield(self) -> ShieldConfig:
        return self.train.shield

    @property
    def costs(self) -> CostsConfig:
        return self.train.costs

    @property
    def reward_weights(self) -> RewardWeightsConfig:
        return self.scenario.reward_weights

    @property
    def randomization(self) -> RandomizationConfig:
        return self.scenario.randomization

    @property
    def observations(self) -> ObservationsConfig:
        return self.scenario.observations

    @property
    def renderer(self) -> RendererConfig:
        return self.scenario.renderer

    # ------------------------------------------------------------------

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Config":
        """Load configuration from YAML file (supports legacy layout)."""
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        train = cls._parse_train(data)
        scenario = cls._parse_scenario(data)
        policy = cls._parse_policy(data.get("policy", {}))
        return cls(train=train, scenario=scenario, policy=policy)

    # ------------------------------------------------------------------
    @staticmethod
    def _parse_train(data: Dict[str, Any]) -> TrainModelConfig:
        train_data = data.get("train")
        if train_data:
            return TrainModelConfig(
                train=TrainConfig(**train_data.get("train", {})),
                plant=PlantConfig(**train_data.get("plant", {})),
                battery=BatteryConfig(**train_data.get("battery", {})),
                fuel_cell=FuelCellConfig(**train_data.get("fuel_cell", {})),
                shield=ShieldConfig(**train_data.get("shield", {})),
                costs=CostsConfig(**train_data.get("costs", {})),
            )
        # Legacy top-level fields
        return TrainModelConfig(
            train=TrainConfig(**data.get("train", {})),
            plant=PlantConfig(**data.get("plant", {})),
            battery=BatteryConfig(**data.get("battery", {})),
            fuel_cell=FuelCellConfig(**data.get("fuel_cell", {})),
            shield=ShieldConfig(**data.get("shield", {})),
            costs=CostsConfig(**data.get("costs", {})),
        )

    @staticmethod
    def _parse_scenario(data: Dict[str, Any]) -> ScenarioConfig:
        scenario_data = data.get("scenario")
        if scenario_data:
            return ScenarioConfig(
                sim=SimConfig(**scenario_data.get("sim", {})),
                route=RouteConfig(**scenario_data.get("route", {})),
                stops=StopsConfig(**scenario_data.get("stops", {})),
                driveline=DrivelineConfig(**scenario_data.get("driveline", {})),
                driver=DriverConfig(**scenario_data.get("driver", {})),
                reward_weights=RewardWeightsConfig(**scenario_data.get("reward_weights", {})),
                randomization=RandomizationConfig(**scenario_data.get("randomization", {})),
                observations=ObservationsConfig(**scenario_data.get("observations", {})),
                renderer=RendererConfig(**scenario_data.get("renderer", {})),
                logging=LoggingConfig(**scenario_data.get("logging", {})),
            )
        # Legacy top-level fields
        return ScenarioConfig(
            sim=SimConfig(**data.get("sim", {})),
            route=RouteConfig(**data.get("route", {})),
            stops=StopsConfig(**data.get("stops", {})),
            driveline=DrivelineConfig(**data.get("driveline", {})),
            driver=DriverConfig(**data.get("driver", {})),
            reward_weights=RewardWeightsConfig(**data.get("reward_weights", {})),
            randomization=RandomizationConfig(**data.get("randomization", {})),
            observations=ObservationsConfig(**data.get("observations", {})),
            renderer=RendererConfig(**data.get("renderer", {})),
            logging=LoggingConfig(**data.get("logging", {})),
        )

    @staticmethod
    def _parse_policy(policy_data: Dict[str, Any]) -> PolicyConfig:
        return PolicyConfig(
            default=policy_data.get("default", "baseline"),
            baseline=policy_data.get("baseline", {}),
            balanced=policy_data.get("balanced", {}),
            scenario=policy_data.get("scenario", {}),
            mpc=policy_data.get("mpc", {}),
            rl=policy_data.get("rl", {}),
        )
