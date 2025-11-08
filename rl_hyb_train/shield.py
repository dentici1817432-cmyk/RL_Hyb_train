"""Shield module: hard safety constraints on actions."""
import numpy as np
from dataclasses import dataclass
from .config import ShieldConfig, BatteryConfig, FuelCellConfig


@dataclass
class ShieldedAction:
    """Shielded action output."""
    p_fc_kw: float
    p_batt_kw: float
    violations: int = 0
    # Detailed shield flags
    soc_low: bool = False
    soc_high: bool = False
    fc_ramp_limited: bool = False
    crate_capped: bool = False
    regen_clipped: bool = False


class Shield:
    """Applies hard safety constraints to actions."""
    
    def __init__(
        self,
        shield_config: ShieldConfig,
        battery_config: BatteryConfig,
        fc_config: FuelCellConfig
    ):
        self.shield_config = shield_config
        self.battery_config = battery_config
        self.fc_config = fc_config
        self.p_fc_prev_kw = 0.0
    
    def reset(self):
        """Reset shield state (previous FC power)."""
        self.p_fc_prev_kw = 0.0
    
    def apply(
        self,
        fc_frac: float,
        batt_cmd: float,
        soc: float,
        tank_level: float
    ) -> ShieldedAction:
        """
        Apply shield constraints to raw actions.
        
        Args:
            fc_frac: FC fraction [0, 1]
            batt_cmd: Battery command [-1, 1]
            soc: Current SOC [0, 1]
            tank_level: Current tank level [0, 1]
        
        Returns:
            ShieldedAction with constrained powers and violation count
        """
        violations = 0
        soc_low = False
        soc_high = False
        fc_ramp_limited = False
        crate_capped = False
        regen_clipped = False
        
        # Convert actions to power commands
        p_fc_cmd_kw = fc_frac * self.fc_config.p_fc_max_kw
        
        # Battery command: [-1, 1] -> [-P_max_chg, +P_max_dis]
        p_batt_cmd_kw = batt_cmd * (
            self.battery_config.p_batt_max_charge_kw if batt_cmd < 0
            else self.battery_config.p_batt_max_discharge_kw
        )
        
        # Store original battery command for regen clipping detection
        p_batt_original_kw = p_batt_cmd_kw
        
        # Apply FC ramp limit
        if self.shield_config.enforce_fc_ramp:
            p_fc_delta_max = self.fc_config.ramp_kw_per_s * 1.0  # dt = 1s
            p_fc_delta = p_fc_cmd_kw - self.p_fc_prev_kw
            if abs(p_fc_delta) > p_fc_delta_max:
                p_fc_cmd_kw = self.p_fc_prev_kw + np.sign(p_fc_delta) * p_fc_delta_max
                violations += 1
                fc_ramp_limited = True
        
        # Clamp FC to [0, P_max]
        p_fc_cmd_kw = np.clip(p_fc_cmd_kw, 0.0, self.fc_config.p_fc_max_kw)
        
        # SOC corridor constraints
        if self.shield_config.enforce_soc_corridor:
            if soc < self.shield_config.soc_soft_min:
                soc_low = True
                # Low SOC: restrict discharge, bias to charge
                if p_batt_cmd_kw > 0:  # Discharge requested
                    p_batt_cmd_kw = 0.0  # Block discharge
                    violations += 1
                # Encourage charging if possible
                if p_batt_cmd_kw == 0.0 and soc < self.shield_config.soc_soft_min - 0.05:
                    # Very low SOC: force some charging
                    p_batt_cmd_kw = -0.3 * self.battery_config.p_batt_max_charge_kw
            
            elif soc > self.shield_config.soc_soft_max:
                soc_high = True
                # High SOC: restrict charge, clip regen
                if p_batt_cmd_kw < 0:  # Charge requested
                    p_batt_cmd_kw = 0.0  # Block charge
                    violations += 1
                    regen_clipped = True
                # Limit discharge to prevent overcharge from regen
                if p_batt_cmd_kw > 0:
                    p_batt_cmd_kw = min(p_batt_cmd_kw, 0.5 * self.battery_config.p_batt_max_discharge_kw)
        
        # Hard SOC limits (emergency)
        if soc <= self.battery_config.soc_hard_min:
            soc_low = True
            p_batt_cmd_kw = max(0.0, p_batt_cmd_kw)  # Only allow charge or zero
            if p_batt_cmd_kw < 0:
                violations += 1
        
        if soc >= self.battery_config.soc_hard_max:
            soc_high = True
            p_batt_cmd_kw = min(0.0, p_batt_cmd_kw)  # Only allow discharge or zero
            if p_batt_cmd_kw > 0:
                violations += 1
        
        # C-rate caps (already enforced by action space, but double-check)
        if self.shield_config.enforce_crate_caps:
            if p_batt_cmd_kw > self.battery_config.p_batt_max_discharge_kw:
                p_batt_cmd_kw = self.battery_config.p_batt_max_discharge_kw
                violations += 1
                crate_capped = True
            if p_batt_cmd_kw < -self.battery_config.p_batt_max_charge_kw:
                p_batt_cmd_kw = -self.battery_config.p_batt_max_charge_kw
                violations += 1
                crate_capped = True
        
        # Detect regen clipping (charge command reduced/blocked)
        if p_batt_original_kw < 0 and p_batt_cmd_kw >= 0:
            regen_clipped = True
        
        # Tank level check (can't use FC if empty)
        if tank_level <= self.fc_config.tank_hard_min:
            if p_fc_cmd_kw > 0:
                violations += 1
            p_fc_cmd_kw = 0.0
        
        # Update previous FC power
        self.p_fc_prev_kw = p_fc_cmd_kw
        
        return ShieldedAction(
            p_fc_kw=p_fc_cmd_kw,
            p_batt_kw=p_batt_cmd_kw,
            violations=violations,
            soc_low=soc_low,
            soc_high=soc_high,
            fc_ramp_limited=fc_ramp_limited,
            crate_capped=crate_capped,
            regen_clipped=regen_clipped
        )

