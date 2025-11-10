function [h2_kg, delta_tank_level] = h2_consumption(p_fc_kw, eta_fc, h2_lhv_kwh_per_kg, tank_capacity_kg, dt_s)
% H2_CONSUMPTION - Compute hydrogen consumption and tank level change
%
% Calculates H2 mass consumed by fuel cell based on power output and
% efficiency, then computes the change in normalized tank level.
%
% Inputs:
%   p_fc_kw           - Fuel cell power output (kW)
%   eta_fc            - Fuel cell efficiency (0-1)
%   h2_lhv_kwh_per_kg - H2 lower heating value (kWh/kg), typically 33.3
%   tank_capacity_kg  - H2 tank capacity (kg)
%   dt_s              - Time step (seconds)
%
% Returns:
%   h2_kg            - H2 mass consumed (kg)
%   delta_tank_level - Change in normalized tank level (0-1)
%
% Equation:
%   h2_kg = (P_FC * dt) / (eta_FC * LHV_H2)
%   delta_tank_level = -h2_kg / tank_capacity_kg

    % Convert time to hours
    dt_hours = dt_s / 3600.0;

    % Check for invalid inputs
    if p_fc_kw <= 0.0 || eta_fc <= 0.0 || h2_lhv_kwh_per_kg <= 0.0
        h2_kg = 0.0;
        delta_tank_level = 0.0;
        return;
    end

    % Compute H2 consumption (kg)
    h2_kg = (p_fc_kw * dt_hours) / (eta_fc * h2_lhv_kwh_per_kg);

    % Compute change in normalized tank level
    if tank_capacity_kg > 0.0
        delta_tank_level = -h2_kg / tank_capacity_kg;
    else
        delta_tank_level = 0.0;
    end

end
