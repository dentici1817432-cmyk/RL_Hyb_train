function plot_results(results, save_fig)
% PLOT_RESULTS - Visualize simulation results
%
% Creates comprehensive plots of the simulation trajectory including:
%   - Power flows (P_req, P_FC, P_batt, P_delivered)
%   - State variables (SOC, tank level, speed, distance)
%   - Actions (fc_frac, batt_cmd)
%   - Constraints (shield activations, unmet demand)
%
% Inputs:
%   results  - Simulation results struct from run_simulation()
%   save_fig - (Optional) If true, save figures to files. Default: false

    if nargin < 2
        save_fig = false;
    end

    % Check if yline/xline exists (MATLAB) or if we need compatibility (Octave)
    has_yline = exist('yline', 'builtin') || exist('yline', 'file');
    has_xline = exist('xline', 'builtin') || exist('xline', 'file');

    % Define compatibility wrappers for Octave
    if ~has_yline
        yline = @octave_yline;
    end
    if ~has_xline
        xline = @octave_xline;
    end

    % Convert time to minutes for better readability
    time_min = results.time / 60;

    % Try to grab drivetrain efficiency metadata for consistent plotting
    eta_traction = 1.0;
    if isfield(results, 'meta') && isfield(results.meta, 'eta_traction')
        eta_traction = max(results.meta.eta_traction, eps);
    end

    %% Figure 1: Power Flows
    figure('Name', 'Power Flows', 'Position', [100, 100, 1200, 800]);

    subplot(3, 1, 1);
    plot(time_min, results.p_req, 'k-', 'LineWidth', 1.5); hold on;
    plot(time_min, results.p_delivered, 'b--', 'LineWidth', 1.2);
    plot(time_min, results.p_unmet, 'r:', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Power (kW)');
    title('Traction Power');
    legend('P_{req}', 'P_{delivered}', 'P_{unmet}', 'Location', 'best');

    subplot(3, 1, 2);
    plot(time_min, results.p_fc, 'g-', 'LineWidth', 1.5); hold on;
    plot(time_min, results.p_batt, 'b-', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 0.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Power (kW)');
    title('Source Power: FC and Battery');
    legend('P_{FC}', 'P_{batt} (+ = discharge)', 'Location', 'best');

    subplot(3, 1, 3);
    p_total_bus = results.p_fc + max(results.p_batt, 0);  % Bus supply (kW)
    p_total_wheel = p_total_bus * eta_traction;
    plot(time_min, p_total_wheel, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Supply (wheel equiv)'); hold on;
    plot(time_min, p_total_bus, 'c--', 'LineWidth', 1.0, 'DisplayName', 'Supply (bus)');
    grid on;
    xlabel('Time (min)');
    ylabel('Power (kW)');
    title('Total Supply Power');
    legend('Location', 'best');

    if save_fig
        saveas(gcf, 'power_flows.png');
    end

    %% Figure 2: State Variables
    figure('Name', 'State Variables', 'Position', [120, 120, 1200, 800]);

    subplot(4, 1, 1);
    plot(time_min, results.soc * 100, 'b-', 'LineWidth', 1.5); hold on;
    yline(20, 'r--', 'SOC min', 'LineWidth', 1.0);
    yline(90, 'r--', 'SOC max', 'LineWidth', 1.0);
    yline(15, 'r:', 'Hard min', 'LineWidth', 1.0);
    grid on;
    xlabel('Time (min)');
    ylabel('SOC (%)');
    title('Battery State of Charge');
    ylim([0, 100]);

    subplot(4, 1, 2);
    plot(time_min, results.tank_level * 100, 'g-', 'LineWidth', 1.5); hold on;
    yline(2, 'r--', 'Empty threshold', 'LineWidth', 1.0);
    grid on;
    xlabel('Time (min)');
    ylabel('Tank Level (%)');
    title('H2 Tank Level');
    ylim([0, 100]);

    subplot(4, 1, 3);
    plot(time_min, results.speed * 3.6, 'k-', 'LineWidth', 1.5);  % Convert m/s to km/h
    grid on;
    xlabel('Time (min)');
    ylabel('Speed (km/h)');
    title('Train Speed');

    subplot(4, 1, 4);
    plot(time_min, results.distance, 'k-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Distance (km)');
    title('Cumulative Distance');

    if save_fig
        saveas(gcf, 'state_variables.png');
    end

    %% Figure 3: Actions and Control
    figure('Name', 'Actions and Control', 'Position', [140, 140, 1200, 600]);

    subplot(2, 1, 1);
    plot(time_min, results.fc_frac * 100, 'g-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('FC Fraction (%)');
    title('Fuel Cell Command');
    ylim([0, 100]);

    subplot(2, 1, 2);
    plot(time_min, results.batt_cmd, 'b-', 'LineWidth', 1.5); hold on;
    yline(0, 'k--', 'LineWidth', 0.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Battery Command');
    title('Battery Command (+ = discharge, - = charge)');
    ylim([-1.1, 1.1]);

    if save_fig
        saveas(gcf, 'actions_control.png');
    end

    %% Figure 4: Constraint Monitoring
    figure('Name', 'Constraint Monitoring', 'Position', [160, 160, 1200, 600]);

    subplot(2, 1, 1);
    plot(time_min, results.shield_active, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Shield Active');
    title('Safety Shield Activations');
    ylim([-0.1, 1.1]);

    subplot(2, 1, 2);
    plot(time_min, results.p_unmet, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Unmet Power (kW)');
    title('Unmet Demand (Supply Shortfall)');

    if save_fig
        saveas(gcf, 'constraint_monitoring.png');
    end

    %% Figure 5: Energy Balance
    figure('Name', 'Energy Balance', 'Position', [180, 180, 1200, 600]);

    % Compute cumulative energy
    dt_hours = gradient(results.time) / 3600;  % Time step in hours
    e_fc_kwh = cumsum(results.p_fc .* dt_hours);
    e_batt_discharge_kwh = cumsum(max(results.p_batt, 0) .* dt_hours);
    e_batt_charge_kwh = cumsum(max(-results.p_batt, 0) .* dt_hours);
    e_demand_kwh = cumsum(max(results.p_req, 0) .* dt_hours);

    subplot(2, 1, 1);
    plot(time_min, e_fc_kwh, 'g-', 'LineWidth', 1.5); hold on;
    plot(time_min, e_batt_discharge_kwh, 'b-', 'LineWidth', 1.5);
    plot(time_min, e_demand_kwh, 'k--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('Energy (kWh)');
    title('Cumulative Energy Flow');
    legend('FC Energy', 'Battery Discharge', 'Demand', 'Location', 'best');

    subplot(2, 1, 2);
    delta_soc_pct = (results.soc - results.soc(1)) * 100;  % Positive = SOC increase
    plot(time_min, delta_soc_pct, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (min)');
    ylabel('\Delta SOC (%)');
    title('Battery SOC Change from Initial');

    if save_fig
        saveas(gcf, 'energy_balance.png');
    end

    %% Figure 6: Operating Point (Phase Portrait)
    figure('Name', 'Operating Point', 'Position', [200, 200, 800, 600]);

    scatter(results.soc * 100, results.p_batt, 20, results.time, 'filled');
    colormap(jet);
    cb = colorbar;
    ylabel(cb, 'Time (s)');
    grid on;
    xlabel('SOC (%)');
    ylabel('P_{batt} (kW)');
    title('Battery Operating Points (SOC vs P_{batt})');
    hold on;
    xline(20, 'r--', 'SOC min', 'LineWidth', 1.0);
    xline(90, 'r--', 'SOC max', 'LineWidth', 1.0);
    yline(0, 'k--', 'LineWidth', 0.5);

    if save_fig
        saveas(gcf, 'operating_point.png');
    end

    fprintf('Plots generated successfully.\n');
    if save_fig
        fprintf('Figures saved to current directory.\n');
    end

end


function octave_yline(y, varargin)
    % Octave-compatible yline replacement
    % Just draw a horizontal line, ignoring labels
    xl = xlim;
    if length(varargin) >= 1
        plot(xl, [y y], varargin{1});
    else
        plot(xl, [y y], 'k--');
    end
end


function octave_xline(x, varargin)
    % Octave-compatible xline replacement
    % Just draw a vertical line, ignoring labels
    yl = ylim;
    if length(varargin) >= 1
        plot([x x], yl, varargin{1});
    else
        plot([x x], yl, 'k--');
    end
end
