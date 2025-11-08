"""Simple battery-only baseline EMS for sanity checking."""
import numpy as np
from typing import Dict, Any
from .config import Config


class BatteryOnlyEMS:
    """
    Simple battery-only EMS for debugging.
    
    Strategy:
    - FC always at 0 (disabled)
    - Battery discharges to meet positive demand
    - Battery charges from regen (negative P_req)
    - Always meets demand exactly
    """
    
    def __init__(self, config: Config):
        """Initialize battery-only EMS."""
        self.config = config
        self.p_batt_max_dis = config.battery.p_batt_max_discharge_kw
        self.p_batt_max_chg = config.battery.p_batt_max_charge_kw
        self.p_aux_base = config.plant.p_aux_base_watts / 1000.0  # kW
    
    def act_from_info(self, info: Dict[str, Any]) -> np.ndarray:
        """
        Compute action using info dict.
        
        Args:
            info: Info dict from environment step
            
        Returns:
            action: [fc_frac, batt_cmd] where fc_frac=0, batt_cmd ∈ [-1,1]
        """
        p_req_kw = info.get('p_req_kw', 0.0)
        
        # FC always disabled
        fc_frac = 0.0
        
        # Calculate total demand (P_req + aux)
        p_demand = p_req_kw + self.p_aux_base
        
        # Determine battery action
        if p_req_kw < 0:
            # Regen: charge battery
            # But we still need to provide aux power!
            regen_power = abs(p_req_kw)
            
            if regen_power > self.p_aux_base:
                # More regen than aux: charge battery with excess
                excess_regen = regen_power - self.p_aux_base
                batt_cmd = -min(1.0, excess_regen / self.p_batt_max_chg)
            else:
                # Less regen than aux: need battery to provide aux
                aux_needed = self.p_aux_base - regen_power
                batt_cmd = min(1.0, aux_needed / self.p_batt_max_dis)
        elif p_demand > 0:
            # Positive demand: discharge battery to meet it
            # Always use full battery to ensure no unmet
            batt_cmd = 1.0
        else:
            # Zero or negative demand: neutral
            batt_cmd = 0.0
        
        # Handle unmet demand from previous step
        p_unmet = info.get('p_unmet_kw', 0.0)
        if p_unmet > 0.1:
            # There's unmet: use full battery
            batt_cmd = 1.0
        
        # Ensure action is in valid range
        fc_frac = np.clip(fc_frac, 0.0, 1.0)
        batt_cmd = np.clip(batt_cmd, -1.0, 1.0)
        
        return np.array([fc_frac, batt_cmd], dtype=np.float32)

