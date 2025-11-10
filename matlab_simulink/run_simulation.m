function results = run_simulation(custom_params)
% RUN_SIMULATION - Run hybrid train EMS simulation (pure MATLAB, no Simulink)
%
% This is a standalone MATLAB implementation of the simulation that does NOT
% require Simulink. It runs the same dynamics as the Simulink model.
%
% Usage:
%   results = run_simulation()                % Use default parameters
%   results = run_simulation(custom_params)   % Use custom parameters
%
% Inputs:
%   custom_params - (Optional) Struct with parameter overrides
%
% Returns:
%   results - Struct containing simulation time history

    % Add paths (handle being called from different directories)
    script_dir = fileparts(mfilename('fullpath'));
    if ~isempty(script_dir)
        addpath(fullfile(script_dir, 'config'));
        addpath(fullfile(script_dir, 'functions'));
        addpath(fullfile(script_dir, 'drivers'));
        addpath(fullfile(script_dir, 'plotting'));
    else
        % Fallback to relative paths
        addpath('config');
        addpath('functions');
        addpath('drivers');
        addpath('plotting');
    end

    % Initialize parameters
    fprintf('Initializing simulation...\n');
    if nargin < 1
        params = initialize_model();
    else
        params = initialize_model(custom_params);
    end

    % Simulation setup
    dt = params.sim.dt;
    t_final = params.sim.stop_time;
    n_steps = floor(t_final / dt) + 1;

    % Preallocate arrays for results
    policy_mode = lower(strtrim(params.ems.policy));

    results.time = zeros(n_steps, 1);
    results.soc = zeros(n_steps, 1);
    results.tank_level = zeros(n_steps, 1);
    results.speed = zeros(n_steps, 1);
    results.distance = zeros(n_steps, 1);
    results.p_req = zeros(n_steps, 1);
    results.p_fc = zeros(n_steps, 1);
    results.p_batt = zeros(n_steps, 1);
    results.p_delivered = zeros(n_steps, 1);
    results.p_unmet = zeros(n_steps, 1);
    results.fc_frac = zeros(n_steps, 1);
    results.batt_cmd = zeros(n_steps, 1);
    results.grade = zeros(n_steps, 1);
    results.shield_active = zeros(n_steps, 1);
    results.aux_bias = zeros(n_steps, 1);
    results.p_wasted = zeros(n_steps, 1);
    results.meta = struct('eta_traction', params.driveline.eta_traction, ...
                          'eta_regen', params.driveline.eta_regen, ...
                          'policy', policy_mode);

    % Initialize state variables
    state.soc = params.init.soc;
    state.tank_level = params.init.tank_level;
    state.speed = params.init.speed_mps;
    state.distance = params.init.distance_km;
    state.p_fc = params.init.p_fc_kw;
    state.p_batt = params.init.p_batt_kw;
    state.aux_bias = params.init.aux_bias_kw;
    state.p_req_filtered = params.init.p_req_filtered;
    state.fc_ref_kw = params.init.fc_ref_kw;

    % Driver state (for simple driver)
    driver_state.filter_state = 0.0;

    fprintf('Running simulation for %.1f seconds (%.1f minutes)...\n', t_final, t_final/60);
    fprintf('Time step: %.2f s\n', dt);

    % Main simulation loop
    for k = 1:n_steps
        t = (k - 1) * dt;
        results.time(k) = t;

        %% 1. Driver: Generate P_req
        [p_req_kw, grade_percent, is_dwelling] = simple_driver(t, params.driver.segments, ...
                                                               driver_state.filter_state, ...
                                                               params.driver.filter_alpha, ...
                                                               params.driver.rate_limit_kw_per_s, dt);
        driver_state.filter_state = p_req_kw;  % Update filter state

        %% 2. Auxiliary load (with random walk)
        p_aux_kw = params.aux.p_base_kw + state.aux_bias;
        % Random walk update
        state.aux_bias = state.aux_bias + randn() * params.aux.bias_std * sqrt(dt);
        state.aux_bias = max(-20.0, min(state.aux_bias, 20.0));  % Clip bias

        %% 3. EMS Controller: Generate actions
        switch policy_mode
            case 'balanced'
                [fc_frac, batt_cmd, state.fc_ref_kw] = balanced_ems(state.soc, p_req_kw, p_aux_kw, ...
                                                                   state.speed, state.fc_ref_kw, params);
            otherwise
                [fc_frac, batt_cmd] = baseline_ems(state.soc, p_req_kw, p_aux_kw, state.speed, params);
        end

        %% 4. Safety Shield: Enforce constraints
        [p_fc_kw, p_batt_cmd_kw, shield_active, ~] = shield_constraints(fc_frac, batt_cmd, ...
                                                                         state.soc, state.tank_level, ...
                                                                         state.p_fc, p_req_kw, ...
                                                                         state.speed, params);

        %% 5. Regenerative braking flow
        [regen_total, regen_aux, regen_post_aux, regen_captured, regen_friction] = ...
            regen_flow(p_req_kw, state.speed, p_aux_kw, max(-p_batt_cmd_kw, 0.0), ...
                      params.train.speed_epsilon, params.driveline.eta_regen);

        %% 6. Power flow calculation
        is_charging = (p_batt_cmd_kw < 0.0);
        [p_dem_kw, p_supply_kw, p_delivered_kw, p_unmet_kw, p_batt_actual_kw, p_wasted_kw] = ...
            power_flow(p_req_kw, p_fc_kw, p_batt_cmd_kw, p_aux_kw, params.battery.eta_discharge, ...
                       state.speed, is_charging, regen_captured, params.driveline.eta_traction, ...
                       params.driveline.eta_regen);

        %% 7. Battery dynamics
        delta_soc = soc_update(p_batt_actual_kw, params.battery.e_batt_kwh, ...
                               params.battery.eta_discharge, params.battery.eta_charge, dt);
        state.soc = state.soc + delta_soc;
        state.soc = max(0.0, min(state.soc, 1.0));  % Clip to [0, 1]

        %% 8. Fuel cell and H2 tank dynamics
        [h2_kg, delta_tank] = h2_consumption(p_fc_kw, params.fc.eta_fc, ...
                                             params.fc.h2_lhv_kwh_per_kg, ...
                                             params.fc.tank_capacity_kg, dt);
        state.tank_level = state.tank_level + delta_tank;
        state.tank_level = max(0.0, min(state.tank_level, 1.0));  % Clip to [0, 1]

        %% 9. Train dynamics
        [accel, delta_speed, delta_dist, ~, ~, ~] = train_dynamics(state.speed, p_delivered_kw, ...
                                                                    grade_percent, params.init.mass_kg, ...
                                                                    params.train.v_max_mps, ...
                                                                    params.train.davis_A_N, ...
                                                                    params.train.davis_B_N_per_mps, ...
                                                                    params.train.davis_C_N_per_mps2, ...
                                                                    params.train.gravity, dt, p_req_kw);

        state.speed = state.speed + delta_speed;
        state.distance = state.distance + delta_dist;

        % Force speed to zero if dwelling
        if is_dwelling
            state.speed = 0.0;
        end

        %% 10. Update state for next iteration
        state.p_fc = p_fc_kw;
        state.p_batt = p_batt_actual_kw;

        %% 11. Store results
        results.soc(k) = state.soc;
        results.tank_level(k) = state.tank_level;
        results.speed(k) = state.speed;
        results.distance(k) = state.distance;
        results.p_req(k) = p_req_kw;
        results.p_fc(k) = p_fc_kw;
        results.p_batt(k) = p_batt_actual_kw;
        results.p_delivered(k) = p_delivered_kw;
        results.p_unmet(k) = p_unmet_kw;
        results.fc_frac(k) = fc_frac;
        results.batt_cmd(k) = batt_cmd;
        results.grade(k) = grade_percent;
        results.shield_active(k) = double(shield_active);
        results.aux_bias(k) = state.aux_bias;
        results.p_wasted(k) = p_wasted_kw;

        %% 12. Check termination conditions
        if state.soc <= params.battery.soc_hard_min
            fprintf('WARNING: SOC below hard minimum at t=%.1f s. Terminating.\n', t);
            results = truncate_results(results, k);
            break;
        end

        if state.tank_level <= params.fc.tank_hard_min
            fprintf('WARNING: H2 tank depleted at t=%.1f s. Terminating.\n', t);
            results = truncate_results(results, k);
            break;
        end

        % Progress indicator
        if mod(k, max(1, floor(n_steps/10))) == 0
            fprintf('  Progress: %.0f%%\n', 100*k/n_steps);
        end
    end

    fprintf('Simulation complete!\n\n');

    % Print summary
    print_summary(results, params);

    % Plot results (only if output requested or in interactive mode)
    if nargout == 0
        % Called without output argument - show plots
        plot_results(results);
    end

end


function results = truncate_results(results, n)
    % Truncate result arrays to actual length
    fields = fieldnames(results);
    for i = 1:length(fields)
        field = fields{i};
        results.(field) = results.(field)(1:n);
    end
end


function print_summary(results, params)
    % Print simulation summary
    fprintf('=== SIMULATION SUMMARY ===\n');
    fprintf('Duration: %.1f s (%.1f min)\n', results.time(end), results.time(end)/60);
    fprintf('Final SOC: %.3f (Initial: %.3f)\n', results.soc(end), results.soc(1));
    fprintf('Final tank level: %.3f (Initial: %.3f)\n', results.tank_level(end), results.tank_level(1));
    fprintf('Final speed: %.2f m/s\n', results.speed(end));
    fprintf('Total distance: %.2f km\n', results.distance(end));
    fprintf('Average P_FC: %.1f kW\n', mean(results.p_fc));
    fprintf('Max P_batt discharge: %.1f kW\n', max(results.p_batt));
    fprintf('Max P_batt charge: %.1f kW\n', -min(results.p_batt));
    fprintf('Total unmet demand: %.2f kWh\n', sum(results.p_unmet) * params.sim.dt / 3600);
    fprintf('Total wasted power: %.2f kWh\n', sum(results.p_wasted) * params.sim.dt / 3600);
    fprintf('Shield activations: %d (%.1f%%)\n', sum(results.shield_active), ...
            100*sum(results.shield_active)/length(results.shield_active));
    fprintf('===========================\n\n');
end
