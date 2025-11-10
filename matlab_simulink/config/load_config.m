function params = load_config(yaml_file)
% LOAD_CONFIG - Load configuration from YAML file (optional)
%
% This function attempts to load parameters from a YAML file.
% If YAML reading fails or file doesn't exist, falls back to defaults.
%
% Usage:
%   params = load_config()                  % Use defaults
%   params = load_config('custom.yaml')     % Load from YAML file
%
% Inputs:
%   yaml_file - (Optional) Path to YAML configuration file
%
% Returns:
%   params - Configuration parameter struct

    % Start with defaults
    params = default_params();

    % If no YAML file specified, return defaults
    if nargin < 1 || isempty(yaml_file)
        fprintf('Using default parameters (no YAML file specified)\n');
        return;
    end

    % Check if file exists
    if ~exist(yaml_file, 'file')
        warning('YAML file not found: %s. Using default parameters.', yaml_file);
        return;
    end

    % Try to read YAML (requires YAML toolbox or custom parser)
    try
        % Attempt to use yaml.loadfile if available
        if exist('yaml.loadfile', 'file')
            yaml_data = yaml.loadfile(yaml_file);
            params = merge_yaml_data(params, yaml_data);
            fprintf('Successfully loaded configuration from: %s\n', yaml_file);
        else
            warning('YAML parser not available. Install yamlmatlab or use default_params()');
        end
    catch ME
        warning('Failed to parse YAML file: %s\nError: %s\nUsing default parameters.', ...
                yaml_file, ME.message);
    end
end


function params = merge_yaml_data(params, yaml_data)
    % MERGE_YAML_DATA - Merge YAML data into parameter struct
    %
    % Maps YAML fields to MATLAB parameter struct

    % This would contain custom logic to map YAML structure to params
    % For now, this is a placeholder since YAML structure may vary

    % Example mapping (customize based on actual YAML structure):
    if isfield(yaml_data, 'sim')
        if isfield(yaml_data.sim, 'dt_seconds')
            params.sim.dt = yaml_data.sim.dt_seconds;
        end
    end

    if isfield(yaml_data, 'battery')
        if isfield(yaml_data.battery, 'e_batt_kwh')
            params.battery.e_batt_kwh = yaml_data.battery.e_batt_kwh;
        end
        % Add more mappings as needed
    end

    % Add more field mappings based on conf.yaml structure
    fprintf('Note: YAML merging is partially implemented. Verify all parameters.\n');
end
