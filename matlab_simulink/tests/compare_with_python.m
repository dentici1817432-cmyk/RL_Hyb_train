function compare_with_python(python_results_file, matlab_results)
% COMPARE_WITH_PYTHON - Compare MATLAB simulation with Python reference
%
% Compares MATLAB simulation results with reference data from the Python
% implementation to validate correctness.
%
% Usage:
%   % First, run Python simulation and save results:
%   % In Python: np.savez('python_results.npz', time=time, soc=soc, ...)
%
%   % Then run MATLAB simulation:
%   matlab_results = run_simulation();
%
%   % Compare:
%   compare_with_python('python_results.npz', matlab_results)
%
% Inputs:
%   python_results_file - Path to .npz file with Python results
%   matlab_results      - MATLAB simulation results struct

    fprintf('\n=== COMPARING MATLAB vs PYTHON RESULTS ===\n\n');

    % Check if file exists
    if ~exist(python_results_file, 'file')
        fprintf('ERROR: Python results file not found: %s\n', python_results_file);
        fprintf('Please run the Python simulation first and save results.\n');
        fprintf('Example Python code:\n');
        fprintf('  np.savez(''python_results.npz'', time=time, soc=soc, tank_level=tank, \n');
        fprintf('           speed=speed, p_fc=p_fc, p_batt=p_batt, distance=distance)\n');
        return;
    end

    % Load Python results
    % Note: .npz files require special handling in MATLAB
    % Users may need to convert to .mat format first
    fprintf('INFO: Loading Python results...\n');
    fprintf('NOTE: MATLAB cannot directly read .npz files.\n');
    fprintf('      Please convert to .mat format using:\n');
    fprintf('      In Python:\n');
    fprintf('        from scipy.io import savemat\n');
    fprintf('        savemat(''python_results.mat'', dict(time=time, soc=soc, ...))\n\n');

    % Try to load as .mat file
    [filepath, name, ext] = fileparts(python_results_file);
    mat_file = fullfile(filepath, [name '.mat']);

    if exist(mat_file, 'file')
        py_data = load(mat_file);
        fprintf('Successfully loaded: %s\n', mat_file);
    else
        fprintf('ERROR: .mat file not found. Please convert Python results to .mat format.\n');
        return;
    end

    % Compare key signals
    fprintf('\n--- SIGNAL COMPARISON ---\n');

    % Interpolate Python data to MATLAB time base (in case different sampling)
    t_matlab = matlab_results.time;
    t_python = py_data.time(:);

    % SOC comparison
    if isfield(py_data, 'soc')
        soc_python_interp = interp1(t_python, py_data.soc(:), t_matlab, 'linear', 'extrap');
        soc_error = matlab_results.soc - soc_python_interp;
        soc_rmse = sqrt(mean(soc_error.^2));
        soc_max_error = max(abs(soc_error));

        fprintf('SOC RMSE: %.6f (%.3f%%)\n', soc_rmse, soc_rmse * 100);
        fprintf('SOC Max Error: %.6f (%.3f%%)\n', soc_max_error, soc_max_error * 100);

        if soc_rmse < 0.01  % 1% tolerance
            fprintf('  ✓ PASS: SOC matches within tolerance\n');
        else
            fprintf('  ✗ FAIL: SOC error exceeds tolerance\n');
        end
    end

    % Tank level comparison
    if isfield(py_data, 'tank_level')
        tank_python_interp = interp1(t_python, py_data.tank_level(:), t_matlab, 'linear', 'extrap');
        tank_error = matlab_results.tank_level - tank_python_interp;
        tank_rmse = sqrt(mean(tank_error.^2));
        tank_max_error = max(abs(tank_error));

        fprintf('Tank RMSE: %.6f (%.3f%%)\n', tank_rmse, tank_rmse * 100);
        fprintf('Tank Max Error: %.6f (%.3f%%)\n', tank_max_error, tank_max_error * 100);

        if tank_rmse < 0.01
            fprintf('  ✓ PASS: Tank level matches within tolerance\n');
        else
            fprintf('  ✗ FAIL: Tank level error exceeds tolerance\n');
        end
    end

    % Speed comparison
    if isfield(py_data, 'speed')
        speed_python_interp = interp1(t_python, py_data.speed(:), t_matlab, 'linear', 'extrap');
        speed_error = matlab_results.speed - speed_python_interp;
        speed_rmse = sqrt(mean(speed_error.^2));
        speed_max_error = max(abs(speed_error));

        fprintf('Speed RMSE: %.4f m/s\n', speed_rmse);
        fprintf('Speed Max Error: %.4f m/s\n', speed_max_error);

        if speed_rmse < 0.1  % 0.1 m/s tolerance
            fprintf('  ✓ PASS: Speed matches within tolerance\n');
        else
            fprintf('  ✗ FAIL: Speed error exceeds tolerance\n');
        end
    end

    % Distance comparison
    if isfield(py_data, 'distance')
        dist_python_interp = interp1(t_python, py_data.distance(:), t_matlab, 'linear', 'extrap');
        dist_error = matlab_results.distance - dist_python_interp;
        dist_rmse = sqrt(mean(dist_error.^2));
        dist_max_error = max(abs(dist_error));

        fprintf('Distance RMSE: %.4f km\n', dist_rmse);
        fprintf('Distance Max Error: %.4f km\n', dist_max_error);

        if dist_rmse < 0.05  % 50 m tolerance
            fprintf('  ✓ PASS: Distance matches within tolerance\n');
        else
            fprintf('  ✗ FAIL: Distance error exceeds tolerance\n');
        end
    end

    % FC Power comparison
    if isfield(py_data, 'p_fc')
        pfc_python_interp = interp1(t_python, py_data.p_fc(:), t_matlab, 'linear', 'extrap');
        pfc_error = matlab_results.p_fc - pfc_python_interp;
        pfc_rmse = sqrt(mean(pfc_error.^2));
        pfc_max_error = max(abs(pfc_error));

        fprintf('P_FC RMSE: %.4f kW\n', pfc_rmse);
        fprintf('P_FC Max Error: %.4f kW\n', pfc_max_error);

        if pfc_rmse < 5.0  % 5 kW tolerance
            fprintf('  ✓ PASS: FC power matches within tolerance\n');
        else
            fprintf('  ✗ FAIL: FC power error exceeds tolerance\n');
        end
    end

    fprintf('\n--- SUMMARY ---\n');
    fprintf('Comparison complete. Review individual signal errors above.\n');
    fprintf('If errors are large, check:\n');
    fprintf('  1. Same initial conditions (SOC, tank, mass, etc.)\n');
    fprintf('  2. Same driver profile/segments\n');
    fprintf('  3. Same parameter values\n');
    fprintf('  4. Same random seed (for aux bias, noise)\n');
    fprintf('=======================================\n\n');

    % Plot comparison
    plot_comparison(matlab_results, py_data, t_matlab, t_python);

end


function plot_comparison(matlab_data, python_data, t_matlab, t_python)
    % PLOT_COMPARISON - Visualize MATLAB vs Python results

    figure('Name', 'MATLAB vs Python Comparison', 'Position', [100, 100, 1200, 800]);

    t_min_matlab = t_matlab / 60;
    t_min_python = t_python / 60;

    % SOC comparison
    if isfield(python_data, 'soc')
        subplot(3, 2, 1);
        plot(t_min_matlab, matlab_data.soc * 100, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_min_python, python_data.soc(:) * 100, 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time (min)');
        ylabel('SOC (%)');
        title('SOC Comparison');
        legend('MATLAB', 'Python', 'Location', 'best');
    end

    % Tank comparison
    if isfield(python_data, 'tank_level')
        subplot(3, 2, 2);
        plot(t_min_matlab, matlab_data.tank_level * 100, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_min_python, python_data.tank_level(:) * 100, 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time (min)');
        ylabel('Tank Level (%)');
        title('H2 Tank Comparison');
        legend('MATLAB', 'Python', 'Location', 'best');
    end

    % Speed comparison
    if isfield(python_data, 'speed')
        subplot(3, 2, 3);
        plot(t_min_matlab, matlab_data.speed, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_min_python, python_data.speed(:), 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time (min)');
        ylabel('Speed (m/s)');
        title('Speed Comparison');
        legend('MATLAB', 'Python', 'Location', 'best');
    end

    % Distance comparison
    if isfield(python_data, 'distance')
        subplot(3, 2, 4);
        plot(t_min_matlab, matlab_data.distance, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_min_python, python_data.distance(:), 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time (min)');
        ylabel('Distance (km)');
        title('Distance Comparison');
        legend('MATLAB', 'Python', 'Location', 'best');
    end

    % FC power comparison
    if isfield(python_data, 'p_fc')
        subplot(3, 2, 5);
        plot(t_min_matlab, matlab_data.p_fc, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_min_python, python_data.p_fc(:), 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time (min)');
        ylabel('P_{FC} (kW)');
        title('Fuel Cell Power Comparison');
        legend('MATLAB', 'Python', 'Location', 'best');
    end

    % Battery power comparison
    if isfield(python_data, 'p_batt')
        subplot(3, 2, 6);
        plot(t_min_matlab, matlab_data.p_batt, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_min_python, python_data.p_batt(:), 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time (min)');
        ylabel('P_{batt} (kW)');
        title('Battery Power Comparison');
        legend('MATLAB', 'Python', 'Location', 'best');
    end

end
