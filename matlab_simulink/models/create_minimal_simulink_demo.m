function create_minimal_simulink_demo()
% CREATE_MINIMAL_SIMULINK_DEMO - Create simple demonstration Simulink model
%
% This creates a minimal but complete Simulink model showing the basic
% structure. Use this as a template to build the full model manually.
%
% Usage:
%   cd matlab_simulink/models
%   create_minimal_simulink_demo()

    model_name = 'train_ems_demo';

    % Close existing
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    if exist([model_name '.slx'], 'file')
        delete([model_name '.slx']);
    end

    fprintf('Creating minimal demo model: %s\n', model_name);

    % Create new model
    new_system(model_name);
    open_system(model_name);

    % Configure solver
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '1.0');
    set_param(model_name, 'StopTime', '100');

    %% Simple structure to demonstrate the concept

    % Input: Power request signal
    add_block('simulink/Sources/Step', [model_name '/P_req_Step'], ...
              'Time', '10', 'After', '300', 'Position', [50, 50, 80, 80]);

    % Controller (gain block as simple example)
    add_block('simulink/Math Operations/Gain', [model_name '/FC_Fraction'], ...
              'Gain', '0.3', 'Position', [150, 50, 180, 80]);
    add_block('simulink/Math Operations/Gain', [model_name '/Batt_Fraction'], ...
              'Gain', '0.7', 'Position', [150, 120, 180, 150]);

    % Power sources
    add_block('simulink/Math Operations/Product', [model_name '/FC_Power'], ...
              'Position', [250, 50, 280, 80]);
    add_block('simulink/Math Operations/Product', [model_name '/Batt_Power'], ...
              'Position', [250, 120, 280, 150]);

    % Sum to get total power
    add_block('simulink/Math Operations/Add', [model_name '/Total_Power'], ...
              'Inputs', '++', 'Position', [350, 80, 380, 110]);

    % Battery SOC integrator (simplified)
    add_block('simulink/Math Operations/Gain', [model_name '/Power_to_SOC'], ...
              'Gain', '-1/(200*3600)', 'Position', [250, 200, 300, 230]);
    add_block('simulink/Discrete/Discrete-Time Integrator', [model_name '/SOC'], ...
              'InitialCondition', '0.5', 'Position', [350, 200, 380, 230]);
    add_block('simulink/Discontinuities/Saturation', [model_name '/SOC_Limit'], ...
              'UpperLimit', '1', 'LowerLimit', '0', 'Position', [420, 200, 450, 230]);

    % Display outputs
    add_block('simulink/Sinks/Scope', [model_name '/Power_Scope'], ...
              'Position', [450, 80, 480, 110]);
    add_block('simulink/Sinks/Scope', [model_name '/SOC_Scope'], ...
              'Position', [500, 200, 530, 230]);

    %% Make connections
    fprintf('Connecting blocks...\n');

    % P_req to controllers
    add_line(model_name, 'P_req_Step/1', 'FC_Fraction/1');
    add_line(model_name, 'P_req_Step/1', 'Batt_Fraction/1');

    % Controllers to power calculations
    add_line(model_name, 'FC_Fraction/1', 'FC_Power/1');
    add_line(model_name, 'P_req_Step/1', 'FC_Power/2');

    add_line(model_name, 'Batt_Fraction/1', 'Batt_Power/1');
    add_line(model_name, 'P_req_Step/1', 'Batt_Power/2');

    % Power to total
    add_line(model_name, 'FC_Power/1', 'Total_Power/1');
    add_line(model_name, 'Batt_Power/1', 'Total_Power/2');

    % Battery SOC integration
    add_line(model_name, 'Batt_Power/1', 'Power_to_SOC/1');
    add_line(model_name, 'Power_to_SOC/1', 'SOC/1');
    add_line(model_name, 'SOC/1', 'SOC_Limit/1');

    % To scopes
    add_line(model_name, 'Total_Power/1', 'Power_Scope/1');
    add_line(model_name, 'SOC_Limit/1', 'SOC_Scope/1');

    fprintf('Connections made!\n');

    %% Save
    save_system(model_name);
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('DEMO MODEL CREATED: %s.slx\n', model_name);
    fprintf('========================================\n\n');
    fprintf('This is a SIMPLIFIED demonstration showing:\n');
    fprintf('  - Power request input (step)\n');
    fprintf('  - Basic FC/Battery power split\n');
    fprintf('  - SOC integration\n');
    fprintf('  - Scopes for visualization\n\n');
    fprintf('To run:\n');
    fprintf('  1. Press the Run button (green play icon)\n');
    fprintf('  2. Double-click scopes to see results\n\n');
    fprintf('For the FULL simulation, use:\n');
    fprintf('  cd ..\n');
    fprintf('  results = run_simulation();\n');
    fprintf('========================================\n\n');

end
