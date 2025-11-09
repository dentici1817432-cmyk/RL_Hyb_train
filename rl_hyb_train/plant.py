"""Plant dynamics module: power balance, SOC, H2 tank, kinematics."""
import numpy as np
from dataclasses import dataclass
from .config import PlantConfig, BatteryConfig, FuelCellConfig
from .powerflow import (
    allocate_battery_charging,
    compute_power_balance,
    compute_regen_flow,
    fc_excess_power,
    h2_consumption_kg,
    resolve_battery_flow,
    soc_delta,
)


@dataclass
class PlantState:
    """State of the plant (SOC, tank, speed, etc.)."""
    soc: float = 0.65  # State of charge [0, 1]
    tank_level: float = 0.85  # Normalized H2 tank level [0, 1]
    speed_mps: float = 0.0  # Speed in m/s
    p_fc_kw: float = 0.0  # Current FC power (kW)
    p_batt_kw: float = 0.0  # Current battery power (kW, positive = discharge)
    aux_bias_kw: float = 0.0  # Auxiliary bias (random walk)
    p_unmet_kw: float = 0.0  # Unmet demand
    distance_km: float = 0.0  # Distance traveled
    p_batt_charge_regen_kw: float = 0.0  # Battery charging from regen (kW)
    p_batt_charge_fc_kw: float = 0.0  # Battery charging from FC excess (kW)
    p_dem_kw: float = 0.0  # Total demand (traction + auxiliaries)
    p_supply_kw: float = 0.0  # Total supplied power (FC + batt after efficiency)
    p_delivered_kw: float = 0.0  # Net power delivered to traction bus after aux/unmet
    p_loss_kw: float = 0.0  # Loss term applied this step
    p_regen_post_aux_kw: float = 0.0  # Regen available after hotel loads
    p_batt_discharge_kw: float = 0.0  # Actual discharged power command (kW)
    p_batt_charge_kw: float = 0.0  # Actual charge accepted (kW)
    p_batt_delivered_kw: float = 0.0  # Battery contribution after efficiency (kW)
    p_brake_total_kw: float = 0.0  # Requested braking power magnitude
    p_brake_regen_kw: float = 0.0  # Portion captured electrically (incl. aux+charging)
    p_brake_aux_kw: float = 0.0  # Regen portion that feeds auxiliaries
    p_brake_friction_kw: float = 0.0  # Residual handled by mechanical brakes


class Plant:
    """Plant dynamics: power balance, SOC updates, H2 consumption, kinematics."""
    
    def __init__(
        self,
        plant_config: PlantConfig,
        battery_config: BatteryConfig,
        fc_config: FuelCellConfig,
        rng: np.random.Generator,
        train_config=None
    ):
        self.plant_config = plant_config
        self.battery_config = battery_config
        self.fc_config = fc_config
        self.train_config = train_config
        self.rng = rng
        self.state = PlantState()
        
        # Hidden state (not observable)
        self.passenger_mass_tons = 220.0  # Will be randomized per episode
        self.aux_bias_rw_sigma_kw = 0.1  # Will be set from config
    
    def reset(
        self,
        soc_init: float,
        tank_init: float,
        passenger_mass_tons: float,
        aux_bias_sigma_kw: float
    ):
        """Reset plant state for new episode."""
        self.state = PlantState(
            soc=soc_init,
            tank_level=tank_init,
            speed_mps=0.0,
            p_fc_kw=0.0,
            p_batt_kw=0.0,
            aux_bias_kw=0.0,
            p_unmet_kw=0.0,
            distance_km=0.0,
            p_batt_charge_regen_kw=0.0,
            p_batt_charge_fc_kw=0.0,
            p_dem_kw=0.0,
            p_supply_kw=0.0,
            p_delivered_kw=0.0,
            p_loss_kw=0.0,
            p_regen_post_aux_kw=0.0,
            p_batt_discharge_kw=0.0,
            p_batt_charge_kw=0.0,
            p_batt_delivered_kw=0.0,
            p_brake_total_kw=0.0,
            p_brake_regen_kw=0.0,
            p_brake_aux_kw=0.0,
            p_brake_friction_kw=0.0,
        )
        self.passenger_mass_tons = passenger_mass_tons
        self.aux_bias_rw_sigma_kw = aux_bias_sigma_kw
    
    def step(
        self,
        p_fc_kw: float,
        p_batt_kw: float,
        p_req_kw: float,
        dt_seconds: float,
        p_loss_kw: float = 200.0
    ):
        """
        Advance plant dynamics by one step.
        
        Args:
            p_fc_kw: Fuel cell power (kW)
            p_batt_kw: Battery power (kW, positive = discharge)
            p_req_kw: Requested traction power (kW)
            dt_seconds: Time step (s)
        
        Returns:
            Updated plant state
        """
        dt_hours = dt_seconds / 3600.0
        
        # Update auxiliary bias (random walk)
        self.state.aux_bias_kw += self.rng.normal(0.0, self.aux_bias_rw_sigma_kw)
        
        # Total auxiliary power
        p_aux_kw = (self.plant_config.p_aux_base_watts / 1000.0) + self.state.aux_bias_kw
        
        # Total demand
        p_dem_kw = p_req_kw + p_aux_kw
        self.state.p_dem_kw = p_dem_kw
        self.state.p_loss_kw = p_loss_kw
        
        # Store actual FC power
        self.state.p_fc_kw = p_fc_kw
        
        raw_batt_flow = resolve_battery_flow(p_batt_kw, self.battery_config.eta_discharge)
        regen_flow = compute_regen_flow(
            p_req_kw=p_req_kw,
            speed_mps=self.state.speed_mps,
            p_aux_kw=p_aux_kw,
            charge_cmd_kw=raw_batt_flow.charge_kw,
        )
        fc_excess = fc_excess_power(p_fc_kw, p_dem_kw)
        charge_alloc = allocate_battery_charging(raw_batt_flow.charge_kw, regen_flow, fc_excess)
        
        if p_batt_kw >= 0.0:
            actual_batt_kw = p_batt_kw
        else:
            actual_batt_kw = -charge_alloc.actual_kw if charge_alloc.actual_kw > 0.0 else 0.0
        
        batt_flow = resolve_battery_flow(actual_batt_kw, self.battery_config.eta_discharge)
        self.state.p_batt_kw = actual_batt_kw
        self.state.p_batt_discharge_kw = batt_flow.discharge_kw
        self.state.p_batt_charge_kw = batt_flow.charge_kw
        self.state.p_batt_delivered_kw = batt_flow.delivered_kw
        self.state.p_batt_charge_regen_kw = charge_alloc.from_regen_kw
        self.state.p_batt_charge_fc_kw = charge_alloc.from_fc_kw
        
        balance = compute_power_balance(
            p_req_kw=p_req_kw,
            p_aux_kw=p_aux_kw,
            p_fc_kw=p_fc_kw,
            p_batt_delivered_kw=batt_flow.delivered_kw,
        )
        self.state.p_dem_kw = balance.p_dem_kw
        self.state.p_supply_kw = balance.p_supply_kw
        self.state.p_unmet_kw = balance.p_unmet_kw
        self.state.p_delivered_kw = balance.p_delivered_kw
        
        soc_delta_val = soc_delta(
            discharge_kw=batt_flow.discharge_kw,
            charge_kw=batt_flow.charge_kw,
            eta_discharge=self.battery_config.eta_discharge,
            eta_charge=self.battery_config.eta_charge,
            e_batt_kwh=self.battery_config.e_batt_kwh,
            dt_hours=dt_hours,
        )
        self.state.soc = np.clip(self.state.soc + soc_delta_val, 0.0, 1.0)
        
        delta_m_h2_kg = h2_consumption_kg(
            p_fc_kw=p_fc_kw,
            eta_fc=self.fc_config.eta_fc,
            h2_lhv_kwh_per_kg=self.fc_config.h2_lhv_kwh_per_kg,
            dt_hours=dt_hours,
        )
        if delta_m_h2_kg > 0.0:
            self.state.tank_level = max(
                0.0,
                self.state.tank_level - delta_m_h2_kg / self.fc_config.tank_capacity_kg,
            )
        
        # Realistic kinematics using Davis resistance and grade forces
        # Delivered traction power accounts for limited supply and unmet demand,
        # and braking power acts against motion (regen + friction).
        p_delivered_kw = self.state.p_delivered_kw
        self.state.p_regen_post_aux_kw = regen_flow.post_aux_kw
        p_brake_total_kw = regen_flow.total_kw
        self.state.p_brake_total_kw = p_brake_total_kw
        self.state.p_brake_aux_kw = regen_flow.aux_kw
        self.state.p_brake_regen_kw = regen_flow.aux_kw + charge_alloc.from_regen_kw
        self.state.p_brake_friction_kw = regen_flow.friction_kw

        # Stopped regime: If speed is very low and no positive traction is requested,
        # lock speed at exactly 0 and skip dynamics (just power auxiliaries with FC)
        STOPPED_THRESHOLD_MPS = 0.01
        if self.state.speed_mps < STOPPED_THRESHOLD_MPS and p_req_kw <= 0.0:
            # Train is stopped - just maintain auxiliary power, no traction dynamics
            self.state.speed_mps = 0.0
            # Distance doesn't change when stopped
            # SOC/H2 already updated above based on power flow
            return self.state

        # Convert power to force for physics calculations
        # P = F * v, so F = P / v (handle v=0 case)
        if self.state.speed_mps > 0.1:  # Avoid division by very small numbers
            traction_force_n = p_delivered_kw * 1000.0 / self.state.speed_mps
        elif p_delivered_kw > 0 and self.state.speed_mps <= 0.1:  # Starting from rest
            traction_force_n = min(p_delivered_kw * 1000.0 / 0.1, 50000.0)  # Max starting force
        else:
            traction_force_n = 0.0
        
        # Calculate resistance forces (Davis formula)
        # F_r = A + B*v + C*v^2
        # Use configured Davis coefficients if available
        if hasattr(self, 'train_config') and self.train_config:
            davis_A = self.train_config.davis_A_N
            davis_B = self.train_config.davis_B_N_per_mps
            davis_C = self.train_config.davis_C_N_per_mps2
        else:
            # Fallback values
            davis_A = 5000.0  # N
            davis_B = 100.0   # N/(m/s)
            davis_C = 5.0     # N/(m/s)^2
        
        resistance_force_n = (
            davis_A + 
            davis_B * self.state.speed_mps + 
            davis_C * self.state.speed_mps ** 2
        )
        
        # Calculate grade force (simplified for now)
        # TODO: Add proper grade integration when using timetable driver
        grade_percent = 0.0  # Default flat
        grade_force_n = 0.0
        
        # Get current grade from driver if available
        if hasattr(self, '_current_grade_percent'):
            grade_percent = self._current_grade_percent
        
        grade_force_n = (
            self.passenger_mass_tons * 1000.0 * 9.81 * grade_percent / 100.0
        )
        
        # Calculate net force and resulting acceleration
        # Positive force = acceleration, negative = deceleration
        net_force_n = traction_force_n - resistance_force_n - grade_force_n
        
        # Update velocity using F = ma -> a = F/m
        mass_kg = self.passenger_mass_tons * 1000.0
        accel_mps2 = net_force_n / mass_kg
        
        # Update speed
        v_new_mps = np.clip(
            self.state.speed_mps + accel_mps2 * dt_seconds,
            0.0,  # Can't go backward
            self.plant_config.v_max_mps
        )
        
        self.state.speed_mps = v_new_mps
        
        # Distance update
        self.state.distance_km += (self.state.speed_mps * dt_seconds) / 1000.0
        
        return self.state
