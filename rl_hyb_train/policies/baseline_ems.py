"""Baseline EMS policy from EMS_policy.md - Rule-based power split."""

from typing import Dict, Any
import numpy as np

from ..config import Config


class BaselineEMS:
    """
    Baseline EMS policy implementing rule-based FC-battery power split.
    
    Based on EMS_policy.md:
    - Baseline A: FC at nominal, charges when possible, battery supplies peaks
    - SOC management with corridors (20% usable below, 90% dump above)
    - Regeneration handling (charge only if SOC < 90%)
    """
    
    def __init__(self, config: Config):
        self.config = config
        
        # EMS parameters from spec/policy
        self.soc_min_usable = 0.20  # Below this, battery is offline
        self.soc_max_regen = 0.90   # Above this, regen is dumped
        
        # FC nominal power (based on current config)
        self.p_fc_nominal_kw = config.fuel_cell.p_fc_max_kw * 0.90  # 90% nominal
        
        # FC standby power (aux-only idle)
        self.p_fc_aux_only_kw = config.driveline.aux_base_kw  # Just auxiliaries
        
        # Power limits
        self.p_batt_max_discharge_kw = config.battery.p_batt_max_discharge_kw
        self.p_batt_max_charge_kw = config.battery.p_batt_max_charge_kw
        
        # Control state
        self.last_action = np.array([0.5, 0.0])  # [fc_frac, batt_cmd]
    
    def reset(self):
        """Reset policy state for new episode."""
        self.last_action = np.array([0.5, 0.0])
    
    def act(self, obs: np.ndarray, info: Dict[str, Any] = None) -> np.ndarray:
        """
        Compute EMS action (EMSPolicy protocol interface).

        Args:
            obs: 12-dim observation vector
            info: Environment info dict (optional)

        Returns:
            action: [fc_frac, batt_cmd] in action space ranges
        """
        if info is None:
            info = {}
        return self.compute_action(obs, info)

    def compute_action(self, obs: np.ndarray, info: Dict[str, Any]) -> np.ndarray:
        """
        Compute EMS action based on current state.

        Args:
            obs: 12-dim observation vector
            info: Environment info dict

        Returns:
            action: [fc_frac, batt_cmd] in action space ranges
        """
        # Extract relevant info from observation and environment state
        # Note: obs[1] is SOC estimate (normalized)
        soc_normalized = obs[1]  # Already includes noise
        soc = (soc_normalized + 1.0) / 2.0  # Convert back from [-1,1] to [0,1]
        
        # Extract demand information  
        # obs[4] is filtered P_req (normalized)
        p_req_normalized = obs[4]
        p_req_kw = ((p_req_normalized + 1.0) / 2.0) * self.config.observations.p_req_max_kw
        
        # Check if FC or tank is offline
        tank_normalized = obs[2]  # Tank level (normalized)
        tank = (tank_normalized + 1.0) / 2.0
        
        fc_offline = tank <= 0.02  # Hard tank minimum
        battery_offline = soc <= self.soc_min_usable
        
        # Determine target speeds/operating mode
        # obs[0] is speed (normalized)
        speed_normalized = obs[0]
        speed_mps = max(0.0, (speed_normalized + 1.0) / 2.0 * self.config.plant.v_max_mps)
        
        is_dwelling = speed_mps < 0.5  # Low speed threshold for dwell
        is_regen = p_req_kw < -50.0  # Negative power request (regenerative)
        
        # Baseline A logic: FC at nominal, charge when possible
        if fc_offline or battery_offline:
            # Emergency mode
            if fc_offline:
                fc_power = 0.0
                battery_power = min(abs(p_req_kw), self.p_batt_max_discharge_kw)
            else:  # Battery offline, use FC as much as possible
                fc_power = min(self.p_fc_nominal_kw * 1.2, self.config.fuel_cell.p_fc_max_kw)  # Boost mode
                battery_power = 0.0
        elif is_dwelling:
            # At station: FC to aux-only, battery handles loads if needed
            fc_power = self.p_fc_aux_only_kw
            if p_req_kw > 0:
                battery_power = min(p_req_kw, self.p_batt_max_discharge_kw)
            else:
                battery_power = 0.0
        elif is_regen:
            # Regenerative braking
            fc_power = 0.0
            if soc < self.soc_max_regen:
                # Accept regen to charge battery
                battery_power = max(p_req_kw, -self.p_batt_max_charge_kw)  # Negative = charging
            else:
                # Dump regen (friction brakes)
                battery_power = 0.0
        else:
            # Normal operation: FC at nominal, battery handles peaks
            if p_req_kw <= self.p_fc_nominal_kw:
                # FC can handle demand, excess charges battery if space
                fc_power = p_req_kw
                if soc < self.soc_max_regen:
                    # Calculate charging capacity
                    charge_capacity = min(
                        self.p_batt_max_charge_kw,
                        (self.p_fc_nominal_kw - p_req_kw) * self.config.battery.eta_charge
                    )
                    battery_power = -charge_capacity  # Negative = charging
                else:
                    battery_power = 0.0
            else:
                # Peak demand: FC at nominal, battery supplies excess
                fc_power = self.p_fc_nominal_kw
                excess_demand = p_req_kw - self.p_fc_nominal_kw
                if soc > self.soc_min_usable:
                    battery_power = min(excess_demand, self.p_batt_max_discharge_kw)
                else:
                    # Battery low, let FC try to handle more
                    fc_power = min(p_req_kw, self.config.fuel_cell.p_fc_max_kw * 1.1)
                    battery_power = max(0.0, p_req_kw - fc_power)
        
        # Convert to action space format
        # fc_frac: [0,1] -> FC power fraction
        fc_frac = np.clip(fc_power / self.config.fuel_cell.p_fc_max_kw, 0.0, 1.0)
        
        # batt_cmd: [-1,1] -> battery command
        if battery_power > 0:
            # Discharge: map [0,1] -> [0, p_max_dis]
            batt_cmd = battery_power / self.p_batt_max_discharge_kw
        elif battery_power < 0:
            # Charge: map [-1,0] -> [-p_max_chg, 0] 
            batt_cmd = battery_power / self.p_batt_max_charge_kw
        else:
            batt_cmd = 0.0
        
        action = np.array([fc_frac, batt_cmd], dtype=np.float32)
        
        # Store action for smoothness penalty tracking
        self.last_action = action
        
        return action
    
    def get_policy_info(self) -> Dict[str, Any]:
        """Get policy information for debugging."""
        return {
            "policy_type": "baseline_ems",
            "soc_min_usable": self.soc_min_usable,
            "soc_max_regen": self.soc_max_regen,
            "p_fc_nominal_kw": self.p_fc_nominal_kw,
            "last_action": self.last_action.tolist()
        }
