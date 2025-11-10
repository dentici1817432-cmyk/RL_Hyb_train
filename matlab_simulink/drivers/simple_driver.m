function [p_req_kw, grade_percent, is_dwelling] = simple_driver(t, segments, filter_state, ...
                                                                 filter_alpha, rate_limit_kw_per_s, dt_s)
% SIMPLE_DRIVER - Generate power request profile from scripted segments
%
% Generates P_req based on pre-defined driving segments (dwell, accel, cruise, etc.)
% with low-pass filtering and rate limiting for smooth profiles.
%
% Inputs:
%   t                   - Current simulation time (seconds)
%   segments            - Cell array of segments: {[duration, mode, base_power, grade], ...}
%                         Modes: 'dwell', 'accelerate', 'cruise', 'climb', 'descent', 'brake'
%   filter_state        - Previous filtered P_req (kW), for continuity
%   filter_alpha        - Low-pass filter coefficient [0,1], higher = faster response
%   rate_limit_kw_per_s - Maximum rate of change (kW/s)
%   dt_s                - Time step (seconds)
%
% Returns:
%   p_req_kw      - Power request (kW), positive = motoring, negative = braking
%   grade_percent - Current track grade (%)
%   is_dwelling   - Boolean, true if in dwell mode

    % Default outputs
    p_req_kw = 0.0;
    grade_percent = 0.0;
    is_dwelling = false;

    % Find current segment
    elapsed = 0.0;
    current_segment = [];

    for i = 1:size(segments, 1)
        seg_duration = segments{i, 1};

        if t >= elapsed && t < (elapsed + seg_duration)
            % Found current segment
            current_segment = segments(i, :);
            break;
        end

        elapsed = elapsed + seg_duration;
    end

    % If past all segments, use last segment
    if isempty(current_segment)
        current_segment = segments(end, :);
    end

    % Extract segment parameters
    mode = current_segment{2};
    base_power_kw = current_segment{3};
    grade_percent = current_segment{4};

    % Compute target P_req based on mode
    switch lower(mode)
        case 'dwell'
            p_target_kw = 0.0;
            is_dwelling = true;

        case 'accelerate'
            % Ramp up to base power
            p_target_kw = base_power_kw;

        case 'cruise'
            % Constant cruising power
            p_target_kw = base_power_kw;

        case 'climb'
            % Climbing with grade compensation
            grade_bias_kw = abs(grade_percent) * 20.0;  % Heuristic: 20 kW per % grade
            p_target_kw = base_power_kw + grade_bias_kw;

        case 'descent'
            % Descending with reduced power
            grade_assist_kw = abs(grade_percent) * 15.0;  % Gravity assists
            p_target_kw = max(0.0, base_power_kw - grade_assist_kw);

        case 'brake'
            % Braking: negative power
            p_target_kw = base_power_kw;  % Already negative in segment definition

        otherwise
            % Unknown mode: zero power
            p_target_kw = 0.0;
    end

    % Apply low-pass filter for smooth transitions
    p_filtered_kw = filter_state + filter_alpha * (p_target_kw - filter_state);

    % Apply rate limiter
    max_delta_kw = rate_limit_kw_per_s * dt_s;
    delta_kw = p_filtered_kw - filter_state;

    if delta_kw > max_delta_kw
        p_req_kw = filter_state + max_delta_kw;
    elseif delta_kw < -max_delta_kw
        p_req_kw = filter_state - max_delta_kw;
    else
        p_req_kw = p_filtered_kw;
    end

end
