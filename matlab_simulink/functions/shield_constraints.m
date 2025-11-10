function [p_fc_kw, p_batt_kw, shield_active, violations] = ...
    shield_constraints(fc_frac, batt_cmd, soc, tank_level, p_fc_prev_kw, p_req_kw, ...
                       speed_mps, params)
% SHIELD_CONSTRAINTS - Enforce safety constraints on EMS actions
%
% Applies hard safety constraints to protect battery and fuel cell:
%   1. SOC corridor enforcement (soft min/max, hard min)
%   2. Fuel cell ramp rate limit
%   3. Battery C-rate limits (charge/discharge)
%   4. Tank level protection
%   5. Regenerative braking coordination
%
% Inputs:
%   fc_frac      - Commanded FC fraction [0,1]
%   batt_cmd     - Commanded battery [-1,1], positive = discharge, negative = charge
%   soc          - Current battery SOC [0,1]
%   tank_level   - Current H2 tank level [0,1]
%   p_fc_prev_kw - Previous FC power (kW), for ramp limit
%   p_req_kw     - Power request (kW), for regen detection
%   speed_mps    - Current speed (m/s), for regen eligibility
%   params       - Parameter struct with limits
%
% Returns:
%   p_fc_kw       - Constrained FC power (kW)
%   p_batt_kw     - Constrained battery power (kW), positive = discharge
%   shield_active - Boolean, true if any constraint was active
%   violations    - Struct with individual violation flags

    % Initialize violations struct
    violations.soc_low = false;
    violations.soc_high = false;
    violations.fc_ramp = false;
    violations.tank_empty = false;
    violations.batt_crate = false;
    violations.traction_limit = false;
    violations.regen_limit = false;

    shield_active = false;

    % Extract parameters
    p_fc_max_kw = params.fc.p_max_kw;
    p_batt_max_discharge_kw = params.battery.p_max_discharge_kw;
    p_batt_max_charge_kw = params.battery.p_max_charge_kw;
    soc_soft_min = params.battery.soc_soft_min;
    soc_soft_max = params.battery.soc_soft_max;
    tank_hard_min = params.fc.tank_hard_min;
    fc_ramp_kw_per_s = params.fc.ramp_kw_per_s;
    dt_s = params.sim.dt;
    speed_threshold = params.train.speed_epsilon;
    p_trac_max_kw = params.train.p_trac_max_kw;
    p_regen_max_kw = params.train.p_regen_max_kw;

    %% 1. Convert actions to power commands
    p_fc_cmd_kw = fc_frac * p_fc_max_kw;

    if batt_cmd >= 0.0
        % Discharge
        p_batt_cmd_kw = batt_cmd * p_batt_max_discharge_kw;
    else
        % Charge (negative power)
        p_batt_cmd_kw = batt_cmd * p_batt_max_charge_kw;
    end

    %% 2. Tank level protection
    if tank_level <= tank_hard_min
        p_fc_cmd_kw = 0.0;
        violations.tank_empty = true;
        shield_active = true;
    end

    %% 3. FC ramp rate limit
    max_ramp_kw = fc_ramp_kw_per_s * dt_s;
    delta_fc = p_fc_cmd_kw - p_fc_prev_kw;

    if delta_fc > max_ramp_kw
        p_fc_cmd_kw = p_fc_prev_kw + max_ramp_kw;
        violations.fc_ramp = true;
        shield_active = true;
    elseif delta_fc < -max_ramp_kw
        p_fc_cmd_kw = p_fc_prev_kw - max_ramp_kw;
        violations.fc_ramp = true;
        shield_active = true;
    end

    p_fc_kw = max(0.0, min(p_fc_cmd_kw, p_fc_max_kw));

    %% 4. SOC corridor enforcement
    is_braking = (p_req_kw < 0.0) && (speed_mps > speed_threshold);

    % Low SOC protection
    if soc < soc_soft_min
        % Block discharge, force charging or allow regen
        if ~is_braking
            % Not braking: force charging
            p_batt_cmd_kw = -p_batt_max_charge_kw * 0.5;  % Moderate charging
            violations.soc_low = true;
            shield_active = true;
        else
            % Braking: allow regen (handled separately)
            % Ensure battery can accept regen
            p_batt_cmd_kw = min(p_batt_cmd_kw, 0.0);  % Block discharge
        end
    end

    % High SOC protection
    if soc > soc_soft_max
        % Block charging, allow limited discharge
        if is_braking
            % Braking: block regen capture
            p_batt_cmd_kw = 0.0;  % No charging
            violations.soc_high = true;
            shield_active = true;
        else
            % Motoring: block any charging command
            p_batt_cmd_kw = max(p_batt_cmd_kw, 0.0);  % Ensure discharge or zero
        end
    end

    %% 5. Battery C-rate limits
    if p_batt_cmd_kw > p_batt_max_discharge_kw
        p_batt_cmd_kw = p_batt_max_discharge_kw;
        violations.batt_crate = true;
        shield_active = true;
    elseif p_batt_cmd_kw < -p_batt_max_charge_kw
        p_batt_cmd_kw = -p_batt_max_charge_kw;
        violations.batt_crate = true;
        shield_active = true;
    end

    p_batt_kw = p_batt_cmd_kw;

    %% 6. Traction / regen envelope limits
    if p_req_kw >= 0.0
        total_supply_kw = p_fc_kw + max(p_batt_kw, 0.0);
        if total_supply_kw > p_trac_max_kw
            excess_kw = total_supply_kw - p_trac_max_kw;
            batt_dis_kw = max(p_batt_kw, 0.0);
            reduce_batt_kw = min(excess_kw, batt_dis_kw);
            p_batt_kw = p_batt_kw - reduce_batt_kw;

            remaining_excess = excess_kw - reduce_batt_kw;
            if remaining_excess > 0.0
                p_fc_kw = max(0.0, p_fc_kw - remaining_excess);
            end

            violations.traction_limit = true;
            shield_active = true;
        end
    else
        regen_kw = max(-p_batt_kw, 0.0);
        if regen_kw > p_regen_max_kw
            p_batt_kw = -p_regen_max_kw;
            violations.regen_limit = true;
            shield_active = true;
        end
    end

    %% 7. Final safety clipping
    p_fc_kw = max(0.0, min(p_fc_kw, p_fc_max_kw));
    p_batt_kw = max(-p_batt_max_charge_kw, min(p_batt_kw, p_batt_max_discharge_kw));

end
