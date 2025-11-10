function results = test_step_changes()
% TEST_STEP_CHANGES - Test simulation with step power demand changes
%
% Validation test: Apply step changes in power demand
% Expected behavior:
%   - FC should ramp smoothly (limited by ramp rate)
%   - Battery should handle transients
%   - Shield should enforce constraints
%
% Usage:
%   results = test_step_changes()

    fprintf('\n=== TEST: Step Changes in Power Demand ===\n\n');

    % Load default parameters
    addpath('../config');
    addpath('../functions');
    addpath('../drivers');
    addpath('../plotting');

    params = default_params();

    % Customize driver for step changes: 0 → 400 → 200 → 600 → 100 kW
    params.driver.segments = {
        [30,   'dwell',  0,   0.0];
        [120,  'cruise', 400, 0.0];  % Step to 400 kW
        [120,  'cruise', 200, 0.0];  % Step down to 200 kW
        [120,  'cruise', 600, 0.0];  % Step up to 600 kW
        [120,  'cruise', 100, 0.0];  % Step down to 100 kW
        [60,   'brake',  -500, 0.0]; % Brake
        [30,   'dwell',  0,   0.0];
    };

    % Increase filter alpha for sharper steps
    params.driver.filter_alpha = 0.2;

    % Run simulation
    results = run_simulation(params);

    % Validation checks
    fprintf('\n=== VALIDATION CHECKS ===\n');

    % Check 1: FC ramp rate respected
    p_fc_diff = diff(results.p_fc);
    dt = params.sim.dt;
    fc_ramp_rate = abs(p_fc_diff / dt);
    max_ramp_rate = params.fc.ramp_kw_per_s;

    if max(fc_ramp_rate) <= (max_ramp_rate * 1.01)  % 1% tolerance
        fprintf('PASS: FC ramp rate respected (max: %.1f kW/s, limit: %.1f kW/s)\n', ...
                max(fc_ramp_rate), max_ramp_rate);
    else
        fprintf('FAIL: FC ramp rate violated! (max: %.1f kW/s, limit: %.1f kW/s)\n', ...
                max(fc_ramp_rate), max_ramp_rate);
    end

    % Check 2: Battery handled transients
    p_batt_range = max(results.p_batt) - min(results.p_batt);
    if p_batt_range > 100.0  % Should see significant battery activity
        fprintf('PASS: Battery active during transients (range: %.1f kW)\n', p_batt_range);
    else
        fprintf('FAIL: Battery not sufficiently active (range: %.1f kW)\n', p_batt_range);
    end

    % Check 3: Shield activations occurred
    shield_count = sum(results.shield_active);
    fprintf('INFO: Shield activated %d times (%.1f%% of time)\n', shield_count, ...
            100 * shield_count / length(results.shield_active));

    % Check 4: SOC stayed within bounds
    if min(results.soc) >= 0.15 && max(results.soc) <= 1.0
        fprintf('PASS: SOC stayed within bounds [0.15, 1.0]\n');
    else
        fprintf('FAIL: SOC violated bounds! Range: [%.3f, %.3f]\n', min(results.soc), max(results.soc));
    end

    fprintf('===========================\n\n');

    % Plot results
    plot_results(results, true);

end
