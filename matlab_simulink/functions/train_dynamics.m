function [accel_mps2, delta_speed_mps, delta_distance_km, f_traction_N, f_resistance_N, f_grade_N] = ...
    train_dynamics(speed_mps, p_delivered_kw, grade_percent, mass_kg, v_max_mps, ...
                   davis_A, davis_B, davis_C, gravity, dt_s, p_req_kw)
% TRAIN_DYNAMICS - Compute train acceleration and motion using point-mass model
%
% Implements realistic train dynamics with Davis resistance equation:
%   F_resistance = A + B*v + C*v^2
%
% Force balance:
%   F_net = F_traction - F_resistance - F_grade
%   a = F_net / m
%
% Inputs:
%   speed_mps       - Current speed (m/s)
%   p_delivered_kw  - Power delivered to traction (kW)
%   grade_percent   - Track grade (%), positive = uphill
%   mass_kg         - Train mass (kg)
%   v_max_mps       - Maximum allowed speed (m/s)
%   davis_A         - Davis resistance constant (N)
%   davis_B         - Davis linear coefficient (N/(m/s))
%   davis_C         - Davis quadratic coefficient (N/(m/s)^2)
%   gravity         - Gravitational acceleration (m/s^2), typically 9.81
%   dt_s            - Time step (seconds)
%   p_req_kw        - Power request (kW), for stopped regime detection
%
% Returns:
%   accel_mps2       - Acceleration (m/s^2)
%   delta_speed_mps  - Change in speed (m/s)
%   delta_distance_km - Change in distance (km)
%   f_traction_N     - Traction force (N)
%   f_resistance_N   - Resistance force (N)
%   f_grade_N        - Grade resistance force (N)

    speed_epsilon = 0.01;  % Speed threshold for zero regime (m/s)

    % Check for stopped regime: speed near zero and not accelerating
    if speed_mps < speed_epsilon && p_req_kw <= 0.0
        % Train is stopped and not trying to move
        accel_mps2 = 0.0;
        delta_speed_mps = 0.0;
        delta_distance_km = 0.0;
        f_traction_N = 0.0;
        f_resistance_N = 0.0;
        f_grade_N = 0.0;
        return;
    end

    % Compute traction force
    % Use small epsilon to avoid division by zero
    v_safe = max(speed_mps, speed_epsilon);
    p_delivered_watts = p_delivered_kw * 1000.0;
    f_traction_N = p_delivered_watts / v_safe;

    % Compute resistance force (Davis equation)
    f_resistance_N = davis_A + davis_B * speed_mps + davis_C * speed_mps^2;

    % Compute grade resistance
    f_grade_N = mass_kg * gravity * (grade_percent / 100.0);

    % Net force
    f_net_N = f_traction_N - f_resistance_N - f_grade_N;

    % Acceleration
    if mass_kg > 0.0
        accel_mps2 = f_net_N / mass_kg;
    else
        accel_mps2 = 0.0;
    end

    % Update speed (Euler integration with clipping)
    new_speed_mps = speed_mps + accel_mps2 * dt_s;
    new_speed_mps = max(0.0, min(new_speed_mps, v_max_mps));  % Clip to [0, v_max]

    delta_speed_mps = new_speed_mps - speed_mps;

    % Update distance (average velocity)
    avg_speed_mps = (speed_mps + new_speed_mps) / 2.0;
    delta_distance_m = avg_speed_mps * dt_s;
    delta_distance_km = delta_distance_m / 1000.0;

end
