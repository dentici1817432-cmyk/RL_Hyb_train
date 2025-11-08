"""Configuration loading and validation."""
import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Dict


@dataclass
class SimConfig:
    dt_seconds: float = 1.0
    episode_steps_min: int = 1200
    episode_steps_max: int = 1800
    seed: Optional[int] = None


@dataclass
class DriverConfig:
    p_req_smoothing_tau_s: float = 2.0
    p_req_rate_limit_kw_per_s: float = 400.0
    p_loss_watts: float = 200000.0
    manual_loop: bool = False
    manual_p_req_profile: Optional[List[Dict[str, float]]] = None
    manual_speed_profile: Optional[List[Dict[str, float]]] = None
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

    def __post_init__(self):
        if self.manual_p_req_profile is None:
            self.manual_p_req_profile = []
        if self.manual_speed_profile is None:
            self.manual_speed_profile = []


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
    tank_hard_min: float = 0.02  # Minimum tank level for termination
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
    p_req_max_kw: float = 2000.0  # Max P_req for normalization
    soc_noise_std: float = 0.02  # SOC observation noise std
    tank_noise_std: float = 0.02  # Tank observation noise std
    nuisance_noise_std: float = 0.1  # Nuisance noise channels std


@dataclass
class RendererConfig:
    enabled: bool = True
    mode: str = "human"  # human | rgb_array | ansi
    render_every: int = 5  # steps
    rolling_window_s: int = 180  # seconds
    dpi: int = 110
    figsize: List[float] = None
    show_hidden_debug: bool = False
    draw_schedule: bool = True
    write_video: bool = False
    video_path: str = "runs/env0_episode.mp4"
    write_png_every: int = 0  # 0 = disabled, else step interval
    interactive: bool = False  # Enable interactive slider for time scrubbing
    colors: dict = None

    def __post_init__(self):
        if self.figsize is None:
            self.figsize = [12.0, 7.0]
        if self.colors is None:
            self.colors = {
                "demand": "#000000",
                "fc": "#1f77b4",
                "batt_dis": "#ff7f0e",
                "batt_chg": "#2ca02c",
                "unmet": "#d62728",
                "soc_band": "#e5e5e5",
            }


@dataclass
class Config:
    sim: SimConfig
    driver: DriverConfig
    plant: PlantConfig
    battery: BatteryConfig
    fuel_cell: FuelCellConfig
    shield: ShieldConfig
    costs: CostsConfig
    reward_weights: RewardWeightsConfig
    randomization: RandomizationConfig
    observations: ObservationsConfig
    renderer: RendererConfig = None

    def __post_init__(self):
        if self.renderer is None:
            self.renderer = RendererConfig()

    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        """Load configuration from YAML file."""
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        
        return cls(
            sim=SimConfig(**data.get("sim", {})),
            driver=DriverConfig(**data.get("driver", {})),
            plant=PlantConfig(**data.get("plant", {})),
            battery=BatteryConfig(**data.get("battery", {})),
            fuel_cell=FuelCellConfig(**data.get("fuel_cell", {})),
            shield=ShieldConfig(**data.get("shield", {})),
            costs=CostsConfig(**data.get("costs", {})),
            reward_weights=RewardWeightsConfig(**data.get("reward_weights", {})),
            randomization=RandomizationConfig(**data.get("randomization", {})),
            observations=ObservationsConfig(**data.get("observations", {})),
            renderer=RendererConfig(**data.get("renderer", {})),
        )
