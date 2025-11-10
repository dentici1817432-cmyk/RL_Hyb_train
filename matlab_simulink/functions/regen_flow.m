function [regen_total_kw, regen_aux_kw, regen_post_aux_kw, regen_captured_kw, regen_friction_kw] = ...
    regen_flow(p_req_kw, speed_mps, p_aux_kw, charge_cmd_kw, speed_threshold, eta_regen)
% REGEN_FLOW - Compute regenerative braking power flow breakdown
%
% When braking (p_req < 0), determines how regenerative energy is allocated:
%   1. First to auxiliary loads
%   2. Then to battery charging (if commanded)
%   3. Remainder to friction brakes
%
% Inputs:
%   p_req_kw        - Power request (kW), negative = braking
%   speed_mps       - Train speed (m/s)
%   p_aux_kw        - Auxiliary load (kW)
%   charge_cmd_kw   - Battery charge command (kW), positive = charge
%   speed_threshold - Speed below which regen is disabled (m/s), default 0.01
%
% Returns:
%   regen_total_kw    - Total regen available from braking (kW)
%   regen_aux_kw      - Regen consumed by auxiliaries (kW)
%   regen_post_aux_kw - Regen remaining after aux (kW)
%   regen_captured_kw - Regen captured by battery (kW)
%   regen_friction_kw - Regen lost to friction brakes (kW)

    % Default speed threshold / efficiency
    if nargin < 5
        speed_threshold = 0.01;
    end
    if nargin < 6
        eta_regen = 1.0;
    end

    % Initialize all outputs to zero
    regen_total_kw = 0.0;
    regen_aux_kw = 0.0;
    regen_post_aux_kw = 0.0;
    regen_captured_kw = 0.0;
    regen_friction_kw = 0.0;

    % Check if braking (p_req < 0) and speed above threshold
    if p_req_kw >= 0.0 || speed_mps <= speed_threshold
        % Not braking or speed too low for regen
        if p_req_kw < 0.0
            regen_total_kw = abs(p_req_kw);
            regen_friction_kw = regen_total_kw;  % All to friction
        end
        return;
    end

    % Total regen available
    regen_total_kw = eta_regen * abs(p_req_kw);

    % Step 1: Regen consumed by auxiliaries
    regen_aux_kw = min(regen_total_kw, max(p_aux_kw, 0.0));

    % Step 2: Remaining regen after auxiliaries
    regen_post_aux_kw = max(0.0, regen_total_kw - regen_aux_kw);

    % Step 3: Battery capture (limited by charge command)
    regen_captured_kw = min(max(charge_cmd_kw, 0.0), regen_post_aux_kw);

    % Step 4: Remainder goes to friction brakes
    regen_friction_kw = max(0.0, regen_total_kw - regen_aux_kw - regen_captured_kw);

end
