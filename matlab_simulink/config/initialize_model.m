function model_params = initialize_model(custom_params)
% INITIALIZE_MODEL - Initialize simulation parameters for Simulink model
%
% This function sets up all parameters needed for the hybrid train EMS
% Simulink model, with optional custom parameter overrides.
%
% Usage:
%   model_params = initialize_model()              % Use all defaults
%   model_params = initialize_model(custom_params) % Override some params
%
% Inputs:
%   custom_params - (Optional) Struct with custom parameter overrides
%
% Returns:
%   model_params - Complete parameter struct for model workspace

    % Load default parameters
    params = default_params();

    % Override with custom parameters if provided
    if nargin > 0 && ~isempty(custom_params)
        params = merge_params(params, custom_params);
    end

    % Initialize random initial conditions
    rng('shuffle');  % Use current time as seed (can be replaced with fixed seed)

    % Random initial SOC
    params.init.soc = params.battery.soc_init_min + ...
                     (params.battery.soc_init_max - params.battery.soc_init_min) * rand();

    % Random initial tank level
    params.init.tank_level = params.fc.tank_init_min + ...
                            (params.fc.tank_init_max - params.fc.tank_init_min) * rand();

    % Random train mass
    params.init.mass_tons = params.train.mass_tons_min + ...
                           (params.train.mass_tons_max - params.train.mass_tons_min) * rand();
    params.init.mass_kg = params.init.mass_tons * 1000.0;

    % Random auxiliary bias
    params.init.aux_bias_kw = params.aux.bias_init_min + ...
                             (params.aux.bias_init_max - params.aux.bias_init_min) * rand();

    % Initial speed and distance
    params.init.speed_mps = 0.0;
    params.init.distance_km = 0.0;

    % Initial FC and battery power
    params.init.p_fc_kw = 0.0;
    params.init.p_batt_kw = 0.0;
    params.init.fc_ref_kw = 0.0;

    % Initial filtered P_req
    params.init.p_req_filtered = 0.0;

    % Initial last action
    params.init.last_fc_frac = 0.0;
    params.init.last_batt_cmd = 0.0;

    % Episode duration (random between min and max)
    episode_steps = randi([params.sim.episode_steps_min, params.sim.episode_steps_max]);
    params.sim.episode_duration_s = episode_steps * params.sim.dt;

    % Compute total driver profile duration
    total_driver_duration = 0;
    for i = 1:size(params.driver.segments, 1)
        total_driver_duration = total_driver_duration + params.driver.segments{i, 1};
    end
    params.driver.total_duration_s = total_driver_duration;

    % Use the shorter of episode duration or driver duration
    params.sim.total_duration_s = min(params.sim.episode_duration_s, total_driver_duration);

    % Simulation stop time
    params.sim.stop_time = params.sim.total_duration_s;

    % Print initialization summary
    fprintf('\n=== Hybrid Train EMS Simulation Initialized ===\n');
    fprintf('Episode duration: %.1f s (%.1f min)\n', params.sim.total_duration_s, params.sim.total_duration_s/60);
    fprintf('Time step: %.2f s\n', params.sim.dt);
    fprintf('Initial SOC: %.3f\n', params.init.soc);
    fprintf('Initial tank level: %.3f\n', params.init.tank_level);
    fprintf('Train mass: %.1f tons\n', params.init.mass_tons);
    fprintf('Initial aux bias: %.2f kW\n', params.init.aux_bias_kw);
    fprintf('===============================================\n\n');

    model_params = params;
end


function merged = merge_params(base, override)
    % MERGE_PARAMS - Recursively merge parameter structs
    %
    % Override fields in base with those in override, preserving structure

    merged = base;
    fields = fieldnames(override);

    for i = 1:length(fields)
        field = fields{i};
        if isfield(merged, field)
            if isstruct(override.(field)) && isstruct(merged.(field))
                % Recursively merge nested structs
                merged.(field) = merge_params(merged.(field), override.(field));
            else
                % Override value
                merged.(field) = override.(field);
            end
        else
            % Add new field
            merged.(field) = override.(field);
        end
    end
end
