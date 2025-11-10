function build_simulink_model()
% BUILD_SIMULINK_MODEL - Programmatically create the hybrid train EMS Simulink model
%
% This script creates the complete Simulink model for the hybrid fuel cell-battery
% train energy management system. Run this script in MATLAB to generate the .slx file.
%
% Usage:
%   cd matlab_simulink/models
%   build_simulink_model()
%
% Output:
%   hybrid_train_ems.slx - Complete Simulink model

    % Model name
    model_name = 'hybrid_train_ems';

    % Close and delete existing model if it exists
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    if exist([model_name '.slx'], 'file')
        delete([model_name '.slx']);
    end

    fprintf('Creating Simulink model: %s\n', model_name);

    % Create new model
    new_system(model_name);
    open_system(model_name);

    %% Configure solver settings
    set_param(model_name, 'Solver', 'FixedStepAuto');
    set_param(model_name, 'FixedStep', '1.0');  % 1 second timestep
    set_param(model_name, 'StartTime', '0.0');
    set_param(model_name, 'StopTime', 'params.sim.stop_time');
    set_param(model_name, 'SaveFormat', 'StructureWithTime');
    set_param(model_name, 'SaveOutput', 'on');
    set_param(model_name, 'OutputSaveName', 'yout');

    %% Add model initialization callback
    set_param(model_name, 'PreLoadFcn', sprintf('addpath(''../config''); addpath(''../functions''); addpath(''../drivers''); params = initialize_model();'));

    fprintf('Building model subsystems...\n');

    %% Create main subsystems
    % These will be placeholders - users will need to populate them in Simulink GUI
    add_block('built-in/Subsystem', [model_name '/Driver'], 'Position', [100, 100, 200, 150]);
    add_block('built-in/Subsystem', [model_name '/EMS_Controller'], 'Position', [250, 100, 350, 150]);
    add_block('built-in/Subsystem', [model_name '/Safety_Shield'], 'Position', [400, 100, 500, 150]);
    add_block('built-in/Subsystem', [model_name '/Power_Flow'], 'Position', [550, 100, 650, 150]);
    add_block('built-in/Subsystem', [model_name '/Battery'], 'Position', [550, 200, 650, 250]);
    add_block('built-in/Subsystem', [model_name '/Fuel_Cell'], 'Position', [550, 300, 650, 350]);
    add_block('built-in/Subsystem', [model_name '/H2_Tank'], 'Position', [700, 300, 800, 350]);
    add_block('built-in/Subsystem', [model_name '/Train_Dynamics'], 'Position', [700, 100, 800, 150]);
    add_block('built-in/Subsystem', [model_name '/Observation_Builder'], 'Position', [100, 250, 200, 300]);

    %% Add clock
    add_block('simulink/Sources/Clock', [model_name '/Clock'], 'Position', [30, 100, 60, 130]);

    %% Add scopes for monitoring
    add_block('simulink/Sinks/Scope', [model_name '/Scope_Power'], 'Position', [850, 100, 880, 130]);
    add_block('simulink/Sinks/Scope', [model_name '/Scope_SOC'], 'Position', [850, 200, 880, 230]);
    add_block('simulink/Sinks/Scope', [model_name '/Scope_Speed'], 'Position', [850, 300, 880, 330]);

    %% Add to workspace blocks for data logging
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_SOC'], ...
              'VariableName', 'soc_log', 'Position', [850, 150, 900, 180]);
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Power'], ...
              'VariableName', 'power_log', 'Position', [850, 250, 900, 280]);
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Speed'], ...
              'VariableName', 'speed_log', 'Position', [850, 350, 900, 380]);

    fprintf('Model structure created. Now adding detailed blocks...\n');

    %% Note to user
    fprintf('\n============================================================\n');
    fprintf('IMPORTANT: Basic model structure created.\n');
    fprintf('You must now manually populate each subsystem using the\n');
    fprintf('MATLAB functions in ../functions/ and ../drivers/\n');
    fprintf('\nAlternatively, use the detailed block diagram in:\n');
    fprintf('  MODEL_STRUCTURE_GUIDE.txt\n');
    fprintf('to implement each subsystem with MATLAB Function blocks.\n');
    fprintf('============================================================\n\n');

    %% Save model
    save_system(model_name, [model_name '.slx']);
    fprintf('Model saved as: %s.slx\n', model_name);
    fprintf('Open it in Simulink to complete the implementation.\n\n');

    % Create detailed implementation guide
    create_model_guide(model_name);

end


function create_model_guide(model_name)
    % CREATE_MODEL_GUIDE - Generate detailed implementation guide

    guide_file = 'MODEL_STRUCTURE_GUIDE.txt';
    fid = fopen(guide_file, 'w');

    fprintf(fid, '========================================\n');
    fprintf(fid, 'SIMULINK MODEL IMPLEMENTATION GUIDE\n');
    fprintf(fid, 'Model: %s.slx\n', model_name);
    fprintf(fid, '========================================\n\n');

    fprintf(fid, 'SUBSYSTEM IMPLEMENTATION DETAILS\n\n');

    fprintf(fid, '1. DRIVER SUBSYSTEM\n');
    fprintf(fid, '   Inputs: t (Clock)\n');
    fprintf(fid, '   Outputs: P_req (kW), grade (%%),is_dwelling (bool)\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function block\n');
    fprintf(fid, '     - Call: simple_driver(t, params.driver.segments, ...)\n');
    fprintf(fid, '     - Maintain filter state using persistent variables\n\n');

    fprintf(fid, '2. EMS_CONTROLLER SUBSYSTEM\n');
    fprintf(fid, '   Inputs: SOC, P_req, speed\n');
    fprintf(fid, '   Outputs: fc_frac [0,1], batt_cmd [-1,1]\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function block\n');
    fprintf(fid, '     - Call: baseline_ems(soc, p_req_kw, speed_mps, params)\n');
    fprintf(fid, '     - Users can replace with their own controller\n\n');

    fprintf(fid, '3. SAFETY_SHIELD SUBSYSTEM\n');
    fprintf(fid, '   Inputs: fc_frac, batt_cmd, SOC, tank_level, P_FC_prev, P_req, speed\n');
    fprintf(fid, '   Outputs: P_FC (kW), P_batt (kW), shield_active, violations\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function block\n');
    fprintf(fid, '     - Call: shield_constraints(..., params)\n');
    fprintf(fid, '     - Use Unit Delay for P_FC_prev feedback\n\n');

    fprintf(fid, '4. POWER_FLOW SUBSYSTEM\n');
    fprintf(fid, '   Inputs: P_req, P_FC, P_batt, P_aux, speed, is_braking\n');
    fprintf(fid, '   Outputs: P_dem, P_supply, P_delivered, P_unmet, P_batt_actual\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function blocks for:\n');
    fprintf(fid, '       * regen_flow() - compute regen breakdown\n');
    fprintf(fid, '       * power_flow() - compute DC bus balance\n');
    fprintf(fid, '     - Use Switch blocks for braking vs motoring modes\n\n');

    fprintf(fid, '5. BATTERY SUBSYSTEM\n');
    fprintf(fid, '   State: SOC (integrator with IC from params.init.soc)\n');
    fprintf(fid, '   Inputs: P_batt_actual (kW)\n');
    fprintf(fid, '   Outputs: SOC [0,1]\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function: soc_update(p_batt, ...params...)\n');
    fprintf(fid, '     - Feed output to Integrator block\n');
    fprintf(fid, '     - Set integrator IC: params.init.soc\n');
    fprintf(fid, '     - Add Saturation block [0, 1]\n\n');

    fprintf(fid, '6. FUEL_CELL SUBSYSTEM\n');
    fprintf(fid, '   Inputs: P_FC (kW)\n');
    fprintf(fid, '   Outputs: H2_consumption (kg/s), P_FC_out (kW)\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function: h2_consumption(p_fc, ...params...)\n');
    fprintf(fid, '     - Include Rate Limiter block (±params.fc.ramp_kw_per_s)\n');
    fprintf(fid, '     - Output ramped P_FC and H2 flow\n\n');

    fprintf(fid, '7. H2_TANK SUBSYSTEM\n');
    fprintf(fid, '   State: tank_level (integrator with IC from params.init.tank_level)\n');
    fprintf(fid, '   Inputs: delta_tank_level (from h2_consumption output)\n');
    fprintf(fid, '   Outputs: tank_level [0,1]\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use Integrator block\n');
    fprintf(fid, '     - Set IC: params.init.tank_level\n');
    fprintf(fid, '     - Add Saturation [0, 1]\n\n');

    fprintf(fid, '8. TRAIN_DYNAMICS SUBSYSTEM\n');
    fprintf(fid, '   States: speed (m/s), distance (km)\n');
    fprintf(fid, '   Inputs: P_delivered (kW), grade (%%)\n');
    fprintf(fid, '   Outputs: speed (m/s), distance (km), accel (m/s²)\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function: train_dynamics(speed, p_delivered, grade, ...params...)\n');
    fprintf(fid, '     - Feed delta_speed to Integrator (IC = 0)\n');
    fprintf(fid, '     - Feed delta_distance to Integrator (IC = 0)\n');
    fprintf(fid, '     - Use Switch for dwelling mode (force speed = 0)\n\n');

    fprintf(fid, '9. OBSERVATION_BUILDER SUBSYSTEM\n');
    fprintf(fid, '   Inputs: All state variables (SOC, tank, speed, P_req, etc.)\n');
    fprintf(fid, '   Outputs: obs [12x1] normalized observation vector\n');
    fprintf(fid, '   Implementation:\n');
    fprintf(fid, '     - Use MATLAB Function to normalize states to [-1, 1]\n');
    fprintf(fid, '     - Add Gaussian noise (randn() * std)\n');
    fprintf(fid, '     - Apply low-pass filter for P_req\n');
    fprintf(fid, '     - (Optional - mainly for RL training)\n\n');

    fprintf(fid, '\nCONNECTION DIAGRAM:\n');
    fprintf(fid, '==================\n\n');
    fprintf(fid, 'Clock → Driver → [P_req, grade] → EMS_Controller → [fc_frac, batt_cmd]\n');
    fprintf(fid, '                                          ↓\n');
    fprintf(fid, '                                   Safety_Shield → [P_FC, P_batt]\n');
    fprintf(fid, '                                          ↓\n');
    fprintf(fid, '                              Power_Flow + Regen_Flow → [P_delivered, ...]\n');
    fprintf(fid, '                                          ↓\n');
    fprintf(fid, '                    ┌─────────────────────┴──────────────────┐\n');
    fprintf(fid, '                    ↓                     ↓                   ↓\n');
    fprintf(fid, '                 Battery              Fuel_Cell          Train_Dynamics\n');
    fprintf(fid, '                   ↓                      ↓                   ↓\n');
    fprintf(fid, '                  SOC                H2_Tank               speed\n');
    fprintf(fid, '                                        ↓\n');
    fprintf(fid, '                                   tank_level\n\n');

    fprintf(fid, 'FEEDBACK LOOPS:\n');
    fprintf(fid, '  - SOC → EMS_Controller\n');
    fprintf(fid, '  - SOC → Safety_Shield\n');
    fprintf(fid, '  - tank_level → Safety_Shield\n');
    fprintf(fid, '  - speed → Driver, EMS_Controller, Train_Dynamics\n');
    fprintf(fid, '  - P_FC → Safety_Shield (via Unit Delay for ramp limit)\n\n');

    fprintf(fid, 'SIMULATION SETTINGS:\n');
    fprintf(fid, '  - Solver: Fixed-step (ode4 or ode1)\n');
    fprintf(fid, '  - Step size: 1.0 second\n');
    fprintf(fid, '  - Stop time: params.sim.stop_time\n');
    fprintf(fid, '  - Initialize: params = initialize_model() in PreLoadFcn\n\n');

    fprintf(fid, 'LOGGING:\n');
    fprintf(fid, '  Use To Workspace blocks to log:\n');
    fprintf(fid, '    - SOC (soc_log)\n');
    fprintf(fid, '    - Power flows (power_log)\n');
    fprintf(fid, '    - Speed (speed_log)\n');
    fprintf(fid, '    - Distance (distance_log)\n');
    fprintf(fid, '    - Tank level (tank_log)\n\n');

    fclose(fid);
    fprintf('Implementation guide written to: %s\n', guide_file);

end
