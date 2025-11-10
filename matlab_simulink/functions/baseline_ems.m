function [fc_frac, batt_cmd] = baseline_ems(soc, p_req_kw, p_aux_kw, speed_mps, params)
% BASELINE_EMS - Simple rule-based energy management strategy
%
% Implements a baseline EMS controller using simple heuristics:
%   - Fuel cell covers base load at constant fraction
%   - Battery handles transients and maintains SOC around target
%   - During braking: attempt to capture regen in battery
%
% This is a placeholder controller that users can replace with their own
% advanced strategies (MPC, RL, optimization, etc.)
%
% Inputs:
%   soc        - Current battery SOC [0,1]
%   p_req_kw   - Power request (kW), positive = motoring, negative = braking
%   p_aux_kw   - Auxiliary load (kW)
%   speed_mps  - Current speed (m/s)
%   params     - Parameter struct
%
% Returns:
%   fc_frac   - FC fraction [0,1]
%   batt_cmd  - Battery command [-1,1], positive = discharge, negative = charge

    % Extract parameters
    fc_base_frac = params.ems.fc_base_frac;
    soc_target = params.ems.soc_target;
    soc_deadband = params.ems.soc_deadband;
    p_fc_max_kw = params.fc.p_max_kw;
    p_batt_max_discharge_kw = params.battery.p_max_discharge_kw;
    p_batt_max_charge_kw = params.battery.p_max_charge_kw;
    %#ok<NASGU> % speed currently unused; kept for consistent signature
    p_aux_kw = max(p_aux_kw, 0.0);
    speed_threshold = params.train.speed_epsilon;
    eta_trac = params.driveline.eta_traction;

    %% 1. Fuel Cell Strategy
    % FC operates at constant base fraction to handle average load
    fc_frac = fc_base_frac;

    % Increase FC if battery SOC is low (supplement FC power)
    if soc < (soc_target - soc_deadband)
        fc_frac = min(1.0, fc_base_frac + 0.3);
    end

    % When demand is high, bias FC upward so base load grows with request (helps limit unmet power)
    if p_req_kw > 0.0
        % Target FC output to cover estimated bus demand (traction + auxiliaries + losses)
        est_bus_kw = p_req_kw / max(eta_trac, 1e-6) + p_aux_kw;
        fc_frac = min(1.0, max(fc_frac, est_bus_kw / p_fc_max_kw));
    end

    %% 2. Battery Strategy
    % Compute FC power output
    p_fc_kw = fc_frac * p_fc_max_kw;

    % Case 1: Braking mode (regen)
    if p_req_kw < 0.0 && speed_mps > speed_threshold
        % Attempt to capture regenerative energy
        % Charge battery with regen power (limited by charge rate)
        regen_power_kw = abs(p_req_kw);
        charge_kw = min(regen_power_kw, p_batt_max_charge_kw);

        % Normalize to command [-1, 1]
        batt_cmd = -charge_kw / p_batt_max_charge_kw;  % Negative = charge

        % Reduce charging if SOC is high
        if soc > (soc_target + soc_deadband)
            batt_cmd = batt_cmd * 0.5;  % Half charging rate
        end

        % Block charging if SOC very high (safety)
        if soc > 0.88
            batt_cmd = 0.0;
        end

        return;
    end

    % Case 2: Motoring mode
    % Battery fills gap between demand and FC supply
    if p_req_kw >= 0.0
        demanded_bus_kw = p_req_kw / max(eta_trac, 1e-6) + p_aux_kw;
    else
        demanded_bus_kw = p_aux_kw;  % Motoring demand is zero during braking
    end

    power_gap_kw = demanded_bus_kw - p_fc_kw;

    if power_gap_kw > 0.0
        % Demand exceeds FC: discharge battery (use commanded gap)
        discharge_kw = min(power_gap_kw, p_batt_max_discharge_kw);
        batt_cmd = discharge_kw / p_batt_max_discharge_kw;
    else
        % FC exceeds demand: charge battery (opportunistic)
        excess_kw = abs(power_gap_kw);
        charge_kw = min(excess_kw, p_batt_max_charge_kw);
        batt_cmd = -charge_kw / p_batt_max_charge_kw;  % Negative = charge
    end

    % SOC-based adjustments
    if soc < (soc_target - soc_deadband)
        % Low SOC: encourage gentle charging but do not throttle traction discharge
        if batt_cmd < 0.0
            batt_cmd = batt_cmd * 1.5;  % Increase recharge aggressiveness
        end
    elseif soc > (soc_target + soc_deadband)
        % High SOC: increase discharge, reduce charge
        if batt_cmd > 0.0
            batt_cmd = batt_cmd * 1.2;  % Increase discharge
        else
            batt_cmd = batt_cmd * 0.3;  % Reduce charge
        end
    end

    % Clip to valid range
    batt_cmd = max(-1.0, min(batt_cmd, 1.0));

end
