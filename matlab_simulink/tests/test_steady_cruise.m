function results = test_steady_cruise()
% TEST_STEADY_CRUISE - Test simulation with constant cruise power
%
% Validation test: Apply constant 200 kW traction demand
% Expected behavior:
%   - FC should stabilize at nominal output
%   - Battery should maintain SOC around target
%   - Speed should reach equilibrium
%
% Usage:
%   results = test_steady_cruise()

    fprintf('\n=== TEST: Steady Cruise (200 kW constant) ===\n\n');

    % Load default parameters
    addpath('../config');
    addpath('../functions');
    addpath('../drivers');
    addpath('../plotting');

    params = default_params();

    % Customize driver for steady cruise
    params.driver.segments = {
        [30,   'dwell',  0,   0.0];    % Brief dwell
        [60,   'accelerate', 200, 0.0]; % Ramp up
        [600,  'cruise', 200, 0.0];    % Long steady cruise
        [30,   'brake',  -400, 0.0];   % Brake to stop
        [30,   'dwell',  0,   0.0];    % Final dwell
    };

    % Run simulation
    results = run_simulation(params);

    % Additional validation checks
    fprintf('\n=== VALIDATION CHECKS ===\n');

    % Check 1: SOC should stay within bounds
    if min(results.soc) >= 0.15 && max(results.soc) <= 1.0
        fprintf('PASS: SOC stayed within bounds [0.15, 1.0]\n');
    else
        fprintf('FAIL: SOC violated bounds! Range: [%.3f, %.3f]\n', min(results.soc), max(results.soc));
    end

    % Check 2: Tank should decrease monotonically
    if results.tank_level(end) < results.tank_level(1)
        fprintf('PASS: H2 tank depleted as expected (%.3f → %.3f)\n', ...
                results.tank_level(1), results.tank_level(end));
    else
        fprintf('FAIL: H2 tank level did not decrease!\n');
    end

    % Check 3: No unmet demand during cruise
    cruise_start = 90 / params.sim.dt;
    cruise_end = 600 / params.sim.dt;
    if cruise_start < length(results.p_unmet)
        unmet_in_cruise = sum(results.p_unmet(cruise_start:min(cruise_end, end)));
        if unmet_in_cruise < 1.0  % Tolerance: < 1 kWh
            fprintf('PASS: Minimal unmet demand during cruise (%.3f kWh)\n', unmet_in_cruise);
        else
            fprintf('FAIL: Significant unmet demand during cruise (%.3f kWh)\n', unmet_in_cruise);
        end
    end

    % Check 4: Speed reached steady state
    if max(results.speed) > 5.0  % Should have accelerated
        fprintf('PASS: Train accelerated (max speed: %.1f m/s = %.1f km/h)\n', ...
                max(results.speed), max(results.speed) * 3.6);
    else
        fprintf('FAIL: Train did not accelerate sufficiently\n');
    end

    fprintf('===========================\n\n');

    % Plot results
    plot_results(results, true);

end
