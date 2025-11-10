function delta_soc = soc_update(p_batt_kw, e_batt_kwh, eta_discharge, eta_charge, dt_s)
% SOC_UPDATE - Compute SOC change due to battery power flow
%
% Implements coulomb counting for battery state of charge update
% based on discharge/charge power and efficiencies.
%
% Inputs:
%   p_batt_kw     - Battery power (kW), positive = discharge, negative = charge
%   e_batt_kwh    - Battery capacity (kWh)
%   eta_discharge - Discharge efficiency (0-1)
%   eta_charge    - Charge efficiency (0-1)
%   dt_s          - Time step (seconds)
%
% Returns:
%   delta_soc - Change in SOC (dimensionless, will be added to current SOC)
%
% Equation:
%   For discharge (p_batt > 0):
%     delta_soc = -p_discharge / (e_batt * eta_discharge) * dt_hours
%   For charge (p_batt < 0):
%     delta_soc = |p_charge| * eta_charge / e_batt * dt_hours

    % Convert time to hours
    dt_hours = dt_s / 3600.0;

    % Handle zero or invalid capacity
    if e_batt_kwh <= 0.0
        delta_soc = 0.0;
        return;
    end

    % Separate discharge and charge components
    p_discharge_kw = max(p_batt_kw, 0.0);  % Positive power = discharge
    p_charge_kw = max(-p_batt_kw, 0.0);    % Negative power → positive charge

    % Compute SOC deltas
    delta_discharge = -p_discharge_kw / (e_batt_kwh * max(eta_discharge, 1e-6)) * dt_hours;
    delta_charge = p_charge_kw * eta_charge / e_batt_kwh * dt_hours;

    % Total change
    delta_soc = delta_discharge + delta_charge;

end
