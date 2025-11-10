function [p_dem_kw, p_supply_kw, p_delivered_kw, p_unmet_kw, p_batt_actual_kw, p_wasted_kw] = ...
    power_flow(p_req_kw, p_fc_kw, p_batt_cmd_kw, p_aux_kw, eta_discharge, speed_mps, ...
               is_charging, regen_captured_kw, eta_traction, eta_regen)
% POWER_FLOW - Compute DC bus power balance for traction demand
%
% Computes the power flow on the DC bus considering:
%   - Demand: traction (p_req) + auxiliaries
%   - Supply: fuel cell + battery (with efficiency losses)
%   - Unmet demand if supply insufficient
%   - Special handling for regenerative braking
%
% Inputs:
%   p_req_kw          - Traction power request (kW), positive = motoring, negative = braking
%   p_fc_kw           - Fuel cell power output (kW)
%   p_batt_cmd_kw     - Battery command (kW), positive = discharge, negative = charge
%   p_aux_kw          - Auxiliary load (kW), always positive
%   eta_discharge     - Battery discharge efficiency (0-1)
%   speed_mps         - Train speed (m/s)
%   is_charging       - Boolean, true if battery is charging
%   regen_captured_kw - Regen energy captured by battery (kW)
%
% Returns:
%   p_dem_kw       - Total demand on DC bus (kW)
%   p_supply_kw    - Total supply to DC bus (kW)
%   p_delivered_kw - Net power delivered to traction after aux and unmet (kW)
%   p_unmet_kw     - Unmet demand shortfall (kW)
%   p_batt_actual_kw - Actual battery power flow (kW)

    speed_threshold = 0.01;  % Speed threshold for zero regime (m/s)
    p_wasted_kw = 0.0;

    % Case 1: Braking mode (p_req < 0)
    if p_req_kw < 0.0 && speed_mps > speed_threshold
        % Regen power available on DC bus after driveline losses
        regen_available_kw = eta_regen * abs(p_req_kw);

        % Aux demand that must still be covered by FC after regen
        regen_for_aux_kw = min(regen_available_kw, max(p_aux_kw, 0.0));
        remaining_aux_kw = max(0.0, p_aux_kw - regen_for_aux_kw);

        % Effective demand on bus once regen has served auxiliaries
        p_dem_kw = remaining_aux_kw;
        p_supply_kw = p_fc_kw;
        p_batt_actual_kw = -regen_captured_kw;  % Negative (charging)

        % Unmet auxiliary power (should rarely trigger)
        p_unmet_kw = max(0.0, p_dem_kw - p_supply_kw);

        % Regen that could not be routed to aux or battery is wasted (friction)
        p_wasted_kw = max(0.0, regen_available_kw - regen_for_aux_kw - regen_captured_kw);

        % Delivered wheel power tracks the driver's braking request (negative)
        p_delivered_kw = p_req_kw;
        return;
    end

    % Case 2: Motoring mode or stopped
    % Demand on DC bus
    demanded_bus_kw = p_req_kw / max(eta_traction, eps) + p_aux_kw;
    p_dem_kw = demanded_bus_kw;

    % Supply from FC
    p_fc_supply = p_fc_kw;

    % Battery contribution to supply
    if is_charging || p_batt_cmd_kw < 0.0
        % Battery is charging (draws from bus)
        p_batt_supply = 0.0;
        p_batt_actual_kw = p_batt_cmd_kw;  % Negative (charging)
    else
        % Battery is discharging (supplies bus)
        p_batt_discharge = max(p_batt_cmd_kw, 0.0);
        p_batt_supply = p_batt_discharge * eta_discharge;  % After efficiency loss
        p_batt_actual_kw = p_batt_discharge;  % Positive (discharging)
    end

    % Total supply on DC bus
    p_supply_kw = p_fc_supply + p_batt_supply;

    % Power available for traction after auxiliaries (bus level)
    p_available_bus_kw = max(0.0, p_supply_kw - p_aux_kw);

    % Convert available bus power back to wheel domain via efficiency
    p_available_wheel_kw = p_available_bus_kw * max(eta_traction, 0.0);

    % Delivered traction power cannot exceed requested wheel power
    p_delivered_kw = min(max(0.0, p_req_kw), p_available_wheel_kw);

    % Unmet demand (wheel domain)
    p_unmet_kw = max(0.0, p_req_kw - p_delivered_kw);

    % Excess supply after satisfying request (wheel domain) counts as wasted
    p_wasted_kw = max(0.0, p_available_wheel_kw - p_delivered_kw);

end
