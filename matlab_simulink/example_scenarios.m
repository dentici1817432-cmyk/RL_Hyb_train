% EXAMPLE_SCENARIOS - Demonstration of different simulation scenarios
%
% This script runs several interesting scenarios to showcase the
% hybrid train EMS capabilities.
%
% Run this script to see:
%   1. Baseline scenario (mixed driving)
%   2. Low initial SOC scenario (stress test)
%   3. High power demand scenario
%   4. Regenerative braking showcase
%
% Usage:
%   cd matlab_simulink
%   example_scenarios

clear; clc; close all;

fprintf('\n');
fprintf('=======================================================\n');
fprintf('   HYBRID TRAIN EMS - SCENARIO DEMONSTRATIONS\n');
fprintf('=======================================================\n\n');

% Add paths
addpath('config');
addpath('functions');
addpath('drivers');
addpath('plotting');

%% Scenario 1: Baseline Mixed Driving
fprintf('--- SCENARIO 1: Baseline Mixed Driving ---\n');
fprintf('Typical urban/suburban rail mission with acceleration,\n');
fprintf('cruising, station stops, and regenerative braking.\n\n');

params1 = default_params();
params1.driver.segments = {
    [45,  'dwell',      0,    0.0];
    [90,  'accelerate', 420,  0.0];
    [180, 'cruise',     310,  0.0];
    [45,  'brake',      -550, 0.0];
    [60,  'dwell',      0,    0.0];
    [90,  'accelerate', 380,  0.5];
    [240, 'cruise',     330,  0.0];
    [45,  'brake',      -600, 0.0];
    [60,  'dwell',      0,    0.0];
    [90,  'accelerate', 400,  0.0];
    [180, 'cruise',     300,  0.0];
    [45,  'brake',      -500, 0.0];
    [45,  'dwell',      0,    0.0];
};

results1 = run_simulation(params1);

% Rename figure
set(gcf, 'Name', 'Scenario 1: Baseline Mixed Driving');
pause(2);

%% Scenario 2: Low Initial SOC (Stress Test)
fprintf('\n--- SCENARIO 2: Low Initial SOC Challenge ---\n');
fprintf('Starting with low battery SOC (25%%) to test EMS\n');
fprintf('ability to recover and maintain energy balance.\n\n');

params2 = default_params();
params2.battery.soc_init_min = 0.25;
params2.battery.soc_init_max = 0.25;

% Moderate driving profile
params2.driver.segments = {
    [30,  'dwell',      0,    0.0];
    [120, 'accelerate', 350,  0.0];
    [300, 'cruise',     280,  0.0];
    [60,  'brake',      -400, 0.0];
    [45,  'dwell',      0,    0.0];
    [120, 'accelerate', 320,  0.0];
    [240, 'cruise',     260,  0.0];
    [60,  'brake',      -450, 0.0];
    [45,  'dwell',      0,    0.0];
};

results2 = run_simulation(params2);
set(gcf, 'Name', 'Scenario 2: Low Initial SOC Challenge');
pause(2);

%% Scenario 3: High Power Demand
fprintf('\n--- SCENARIO 3: High Power Demand Operation ---\n');
fprintf('Heavy freight operation with sustained high power,\n');
fprintf('including steep grades and high speeds.\n\n');

params3 = default_params();
params3.train.mass_tons_min = 250;  % Heavy train
params3.train.mass_tons_max = 250;

params3.driver.segments = {
    [30,  'dwell',      0,    0.0];
    [120, 'accelerate', 600,  0.0];    % High acceleration demand
    [180, 'cruise',     500,  0.0];    % High cruise power
    [150, 'climb',      700,  2.5];    % Steep climb (2.5% grade)
    [180, 'cruise',     450,  0.5];    % Continued high demand
    [90,  'descent',    200,  -2.0];   % Descent with gravity assist
    [60,  'brake',      -800, 0.0];    % Heavy braking
    [45,  'dwell',      0,    0.0];
};

results3 = run_simulation(params3);
set(gcf, 'Name', 'Scenario 3: High Power Demand');
pause(2);

%% Scenario 4: Regenerative Braking Showcase
fprintf('\n--- SCENARIO 4: Regenerative Braking Showcase ---\n');
fprintf('Frequent stop-and-go operation to demonstrate\n');
fprintf('regenerative energy capture effectiveness.\n\n');

params4 = default_params();
params4.battery.soc_init_min = 0.40;  % Start lower for regen room
params4.battery.soc_init_max = 0.40;

params4.driver.segments = {
    % Repeated acceleration-braking cycles
    [30,  'dwell',      0,    0.0];
    [60,  'accelerate', 450,  0.0];
    [90,  'cruise',     320,  0.0];
    [45,  'brake',      -650, 0.0];
    [30,  'dwell',      0,    0.0];
    [60,  'accelerate', 480,  0.0];
    [90,  'cruise',     340,  0.0];
    [45,  'brake',      -700, 0.0];
    [30,  'dwell',      0,    0.0];
    [60,  'accelerate', 440,  0.0];
    [90,  'cruise',     310,  0.0];
    [45,  'brake',      -620, 0.0];
    [30,  'dwell',      0,    0.0];
    [60,  'accelerate', 460,  0.0];
    [90,  'cruise',     330,  0.0];
    [45,  'brake',      -680, 0.0];
    [30,  'dwell',      0,    0.0];
};

results4 = run_simulation(params4);
set(gcf, 'Name', 'Scenario 4: Regenerative Braking Showcase');

%% Summary Comparison
fprintf('\n');
fprintf('=======================================================\n');
fprintf('   SCENARIO COMPARISON SUMMARY\n');
fprintf('=======================================================\n\n');

scenarios = {'Baseline', 'Low SOC', 'High Power', 'Regen Focus'};
all_results = {results1, results2, results3, results4};

fprintf('%-15s | %8s | %8s | %8s | %8s | %8s\n', ...
        'Scenario', 'SOC Δ', 'Dist(km)', 'H2(kg)', 'Regen(kWh)', 'Shield(%)');
fprintf('----------------------------------------------------------------\n');

for i = 1:length(scenarios)
    res = all_results{i};

    % Calculate metrics
    soc_delta = res.soc(end) - res.soc(1);
    distance = res.distance(end);
    h2_consumed = (res.tank_level(1) - res.tank_level(end)) * 50;  % 50 kg tank

    % Regen energy (charging during braking)
    dt_hours = 1.0 / 3600;  % 1 second timestep
    regen_energy = sum(max(-res.p_batt(res.p_req < 0), 0)) * dt_hours;

    % Shield activation percentage
    shield_pct = 100 * sum(res.shield_active) / length(res.shield_active);

    fprintf('%-15s | %+7.3f | %8.2f | %8.2f | %9.2f | %7.1f%%\n', ...
            scenarios{i}, soc_delta, distance, h2_consumed, regen_energy, shield_pct);
end

fprintf('\n');
fprintf('=======================================================\n\n');

%% Key Insights
fprintf('KEY INSIGHTS:\n\n');

fprintf('1. BASELINE SCENARIO:\n');
fprintf('   - Typical operation with balanced energy flows\n');
fprintf('   - SOC remains relatively stable\n');
fprintf('   - Moderate shield activations for safety\n\n');

fprintf('2. LOW SOC CHALLENGE:\n');
fprintf('   - EMS prioritizes SOC recovery\n');
fprintf('   - Increased FC usage to charge battery\n');
fprintf('   - More frequent shield interventions\n\n');

fprintf('3. HIGH POWER DEMAND:\n');
fprintf('   - Heavy battery utilization for peak demands\n');
fprintf('   - FC operates near maximum frequently\n');
fprintf('   - Significant H2 consumption\n\n');

fprintf('4. REGEN SHOWCASE:\n');
fprintf('   - Frequent braking captures significant energy\n');
fprintf('   - Battery charging improves SOC\n');
fprintf('   - Reduced H2 consumption vs. baseline\n\n');

fprintf('=======================================================\n\n');

% Create comparison figure
figure('Name', 'Scenario Comparison', 'Position', [250, 250, 1200, 600]);

subplot(2, 2, 1);
for i = 1:4
    plot(all_results{i}.time / 60, all_results{i}.soc * 100, 'LineWidth', 1.5);
    hold on;
end
grid on;
xlabel('Time (min)');
ylabel('SOC (%)');
title('Battery SOC Trajectories');
legend(scenarios, 'Location', 'best');

subplot(2, 2, 2);
for i = 1:4
    plot(all_results{i}.time / 60, all_results{i}.p_fc, 'LineWidth', 1.5);
    hold on;
end
grid on;
xlabel('Time (min)');
ylabel('P_{FC} (kW)');
title('Fuel Cell Power');
legend(scenarios, 'Location', 'best');

subplot(2, 2, 3);
for i = 1:4
    plot(all_results{i}.time / 60, all_results{i}.p_batt, 'LineWidth', 1.5);
    hold on;
end
yline(0, 'k--', 'LineWidth', 0.5);
grid on;
xlabel('Time (min)');
ylabel('P_{batt} (kW)');
title('Battery Power');
legend(scenarios, 'Location', 'best');

subplot(2, 2, 4);
for i = 1:4
    plot(all_results{i}.time / 60, all_results{i}.speed * 3.6, 'LineWidth', 1.5);
    hold on;
end
grid on;
xlabel('Time (min)');
ylabel('Speed (km/h)');
title('Train Speed');
legend(scenarios, 'Location', 'best');

fprintf('All scenarios completed! Review the figures to compare.\n\n');
