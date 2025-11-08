"""Plant dynamics module: power balance, SOC, H2 tank, kinematics."""
import numpy as np
from dataclasses import dataclass
from .config import PlantConfig, BatteryConfig, FuelCellConfig


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
        rng: np.random.Generator
    ):
        self.plant_config = plant_config
        self.battery_config = battery_config
        self.fc_config = fc_config
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
        
        # Store actual FC and battery power
        self.state.p_fc_kw = p_fc_kw
        self.state.p_batt_kw = p_batt_kw
        
        # Power balance
        p_batt_discharge = max(p_batt_kw, 0.0)  # Only discharge contributes to supply
        p_batt_charge = max(-p_batt_kw, 0.0)
        self.state.p_batt_discharge_kw = p_batt_discharge
        self.state.p_batt_charge_kw = p_batt_charge
        
        # Check available charging sources BEFORE preventing invalid charging
        # Initialize charging sources
        p_batt_charge_regen_available = 0.0
        p_batt_charge_fc_available = 0.0
        p_regen_post_aux = 0.0
        p_brake_total_kw = 0.0
        p_brake_aux_kw = 0.0
        p_brake_regen_kw = 0.0
        p_brake_friction_kw = 0.0
        
        if p_batt_charge > 0.0:
            # Check if regen is available (P_req < 0 and train is moving)
            if p_req_kw < 0.0 and self.state.speed_mps > 0.01:
                p_regen_available = abs(p_req_kw)
                # Aux loads consume regen first; only excess can charge the battery
                p_brake_total_kw = p_regen_available
                p_brake_aux_kw = min(p_regen_available, p_aux_kw)
                p_regen_post_aux = max(0.0, p_regen_available - p_aux_kw)
                p_batt_charge_regen_available = min(p_batt_charge, p_regen_post_aux)
            
            # Check FC excess availability
            if p_fc_kw > 0.0:
                if p_dem_kw <= 0.0:
                    # Negative or zero demand: all FC power can charge battery
                    p_fc_excess = p_fc_kw
                else:
                    # Positive demand: FC excess = FC - demand
                    p_fc_excess = max(0.0, p_fc_kw - p_dem_kw)
                
                if p_fc_excess > 0.0:
                    remaining_charge = p_batt_charge - p_batt_charge_regen_available
                    p_batt_charge_fc_available = min(remaining_charge, p_fc_excess)
            
            # Prevent charging if there's no power source (no regen and no FC excess)
            total_available_charge = p_batt_charge_regen_available + p_batt_charge_fc_available
            if total_available_charge < p_batt_charge:
                # Battery is trying to charge more than available power sources allow
                # Clamp charging to available power sources
                p_batt_charge = total_available_charge
                # Update battery power to reflect actual charging (negative = charge)
                self.state.p_batt_kw = -p_batt_charge if total_available_charge > 0.0 else 0.0
                # Recalculate discharge
                p_batt_discharge = max(self.state.p_batt_kw, 0.0)
                p_batt_charge = max(-self.state.p_batt_kw, 0.0)
                self.state.p_batt_discharge_kw = p_batt_discharge
                self.state.p_batt_charge_kw = p_batt_charge

        if p_req_kw < 0.0:
            if self.state.speed_mps > 0.01:
                p_brake_regen_kw = p_brake_aux_kw + p_batt_charge_regen_available
                p_brake_friction_kw = max(0.0, abs(p_req_kw) - p_brake_regen_kw)
            else:
                p_brake_total_kw = abs(p_req_kw)
                p_brake_aux_kw = 0.0
                p_brake_regen_kw = 0.0
                p_brake_friction_kw = p_brake_total_kw
        
        # Apply battery efficiency to actual power delivered
        # Discharge efficiency: commanded power * efficiency = actual delivered power
        p_batt_delivered_kw = p_batt_discharge * self.battery_config.eta_discharge
        self.state.p_batt_delivered_kw = p_batt_delivered_kw
        
        # Power balance: check if supply meets demand
        p_supply_kw = p_fc_kw + p_batt_delivered_kw
        self.state.p_supply_kw = p_supply_kw
        residual = p_dem_kw - p_supply_kw
        
        # Unmet demand (when supply < demand)
        self.state.p_unmet_kw = max(residual, 0.0)
        
        # If residual < 0, excess can charge battery (already handled by shield)
        
        # SOC update (coulomb counting)
        soc_delta = (
            -p_batt_discharge / (self.battery_config.e_batt_kwh * self.battery_config.eta_discharge) * dt_hours
            + p_batt_charge * self.battery_config.eta_charge / self.battery_config.e_batt_kwh * dt_hours
        )
        self.state.soc = np.clip(self.state.soc + soc_delta, 0.0, 1.0)
        
        # H2 consumption
        if p_fc_kw > 0:
            delta_m_h2_kg = (p_fc_kw / (self.fc_config.eta_fc * self.fc_config.h2_lhv_kwh_per_kg)) * dt_hours
            self.state.tank_level -= delta_m_h2_kg / self.fc_config.tank_capacity_kg
            self.state.tank_level = max(0.0, self.state.tank_level)
        else:
            delta_m_h2_kg = 0.0
        
        # Kinematics (toy model)
        # Delivered traction power accounts for limited supply and unmet demand,
        # and braking power acts against motion (regen + friction).
        p_delivered_kw = (
            p_fc_kw
            + p_batt_delivered_kw
            - p_aux_kw
            - self.state.p_unmet_kw
        )
        self.state.p_delivered_kw = p_delivered_kw
        self.state.p_regen_post_aux_kw = p_regen_post_aux
        self.state.p_brake_total_kw = p_brake_total_kw
        self.state.p_brake_regen_kw = p_brake_regen_kw
        self.state.p_brake_aux_kw = p_brake_aux_kw
        self.state.p_brake_friction_kw = p_brake_friction_kw
        # Net power after accounting for braking and generic losses
        p_net_kw = p_delivered_kw - p_brake_total_kw - p_loss_kw
        
        # Speed update: v_{t+1} = clip(v_t + k_v * P_net * dt, 0, v_max)
        # Note: kinematic_gain is in (m/s)/W, so convert kW to W
        v_delta = self.plant_config.kinematic_gain_mps_per_watt * p_net_kw * 1000.0 * dt_seconds
        self.state.speed_mps = np.clip(
            self.state.speed_mps + v_delta,
            0.0,
            self.plant_config.v_max_mps
        )
        
        # Store charging sources for info display (use values calculated above)
        self.state.p_batt_charge_regen_kw = p_batt_charge_regen_available
        self.state.p_batt_charge_fc_kw = p_batt_charge_fc_available
        
        # Distance update
        self.state.distance_km += (self.state.speed_mps * dt_seconds) / 1000.0
        
        return self.state
