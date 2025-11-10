function results = test_regen_profile()
% TEST_REGEN_PROFILE - Test simulation with regenerative braking
%
% Validation test: Mixed acceleration and braking profile
% Expected behavior:
%   - Battery should charge during braking (P_batt < 0)
%   - SOC should increase during regen events
%   - Friction braking should activate if battery cannot accept regen
%
% Usage:
%   results = test_regen_profile()

    fprintf('\n=== TEST: Regenerative Braking Profile ===\n\n');

    % Load default parameters
    addpath('..');             % Add parent directory for run_simulation
    addpath('../config');
    addpath('../functions');
    addpath('../drivers');
    addpath('../plotting');

    params = default_params();

    % Customize driver for regen testing
    params.driver.segments = {
        30,   'dwell',      0,    0.0;
        90,   'accelerate', 500,  0.0;  % Accelerate
        120,  'cruise',     350,  0.0;  % Cruise
        60,   'brake',      -600, 0.0;  % Heavy braking (regen)
        30,   'dwell',      0,    0.0;  % Dwell
        90,   'accelerate', 450,  0.0;  % Accelerate again
        120,  'cruise',     320,  0.0;  % Cruise
        60,   'brake',      -700, 0.0;  % Heavy braking (regen)
        30,   'dwell',      0,    0.0;
        60,   'accelerate', 400,  0.0;
        90,   'cruise',     300,  0.0;
        45,   'brake',      -500, 0.0;  % Final braking
        30,   'dwell',      0,    0.0;
    };

    % Start with lower SOC to have room for regen
    params.battery.soc_init_min = 0.40;
    params.battery.soc_init_max = 0.40;

    % Run simulation
    results = run_simulation(params);

    % Validation checks
    fprintf('\n=== VALIDATION CHECKS ===\n');

    % Check 1: Identify braking events (P_req < 0)
    braking_mask = results.p_req < 0;
    n_braking_steps = sum(braking_mask);

    if n_braking_steps > 0
        fprintf('INFO: %d braking steps detected\n', n_braking_steps);

        % Check 2: Battery charging during braking
        charging_during_braking = sum(results.p_batt(braking_mask) < 0);
        regen_percentage = 100 * charging_during_braking / n_braking_steps;

        if regen_percentage > 50.0
            fprintf('PASS: Battery charged during %.1f%% of braking events\n', regen_percentage);
        else
            fprintf('WARN: Battery only charged during %.1f%% of braking events\n', regen_percentage);
        end

        % Check 3: SOC increased after regen
        soc_change_in_regen = results.soc(braking_mask);
        if length(soc_change_in_regen) > 1
            soc_delta = soc_change_in_regen(end) - soc_change_in_regen(1);
            if soc_delta > 0.0
                fprintf('PASS: SOC increased during braking events (+%.3f)\n', soc_delta);
            else
                fprintf('WARN: SOC did not increase during braking (%.3f)\n', soc_delta);
            end
        end
    else
        fprintf('WARN: No braking events detected in profile!\n');
    end

    % Check 4: Overall SOC change
    overall_soc_delta = results.soc(end) - results.soc(1);
    fprintf('INFO: Overall SOC change: %.3f (%.1f → %.1f)\n', ...
            overall_soc_delta, results.soc(1), results.soc(end));

    % Check 5: No SOC violations
    if min(results.soc) >= 0.15 && max(results.soc) <= 1.0
        fprintf('PASS: SOC stayed within bounds [0.15, 1.0]\n');
    else
        fprintf('FAIL: SOC violated bounds! Range: [%.3f, %.3f]\n', min(results.soc), max(results.soc));
    end

    % Check 6: Total regen energy captured
    dt_hours = params.sim.dt / 3600;
    regen_energy = sum(max(-results.p_batt(braking_mask), 0)) * dt_hours;
    fprintf('INFO: Total regenerative energy captured: %.2f kWh\n', regen_energy);

    fprintf('===========================\n\n');

    % Plot results
    plot_results(results, true);

end
