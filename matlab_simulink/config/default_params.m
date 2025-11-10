function params = default_params()
% DEFAULT_PARAMS - Returns default parameters for hybrid train EMS simulation
%
% This function provides all default configuration parameters matching the
% Python implementation in conf.yaml
%
% Returns:
%   params - Struct containing all simulation parameters

    %% Simulation Parameters
    params.sim.dt = 1.0;                    % Time step (seconds)
    params.sim.episode_steps_min = 1800;   % Min episode length (steps)
    params.sim.episode_steps_max = 2400;   % Max episode length (steps)

    %% Battery Parameters
    params.battery.e_batt_kwh = 300.0;                  % Battery capacity (kWh)
    params.battery.p_max_discharge_kw = 600.0;          % Max discharge power (kW)
    params.battery.p_max_charge_kw = 300.0;             % Max charge power (kW)
    params.battery.eta_discharge = 0.94;                % Discharge efficiency
    params.battery.eta_charge = 0.95;                   % Charge efficiency
    params.battery.soc_init_min = 0.55;                 % Min initial SOC
    params.battery.soc_init_max = 0.80;                 % Max initial SOC
    params.battery.soc_hard_min = 0.15;                 % Hard minimum SOC (termination)
    params.battery.soc_soft_min = 0.20;                 % Soft minimum SOC (shield activates)
    params.battery.soc_soft_max = 0.90;                 % Soft maximum SOC (shield activates)

    %% Fuel Cell Parameters
    params.fc.p_max_kw = 400.0;                         % Max FC power (kW)
    params.fc.eta_fc = 0.50;                            % FC efficiency
    params.fc.h2_lhv_kwh_per_kg = 33.3;                 % H2 lower heating value (kWh/kg)
    params.fc.tank_capacity_kg = 50.0;                  % H2 tank capacity (kg)
    params.fc.tank_init_min = 0.70;                     % Min initial tank level
    params.fc.tank_init_max = 1.00;                     % Max initial tank level
    params.fc.tank_hard_min = 0.02;                     % Hard minimum tank (termination)
    params.fc.ramp_kw_per_s = 40.0;                     % Max ramp rate (kW/s)

    %% Train Parameters
    params.train.v_max_mps = 50.0;                      % Maximum speed (m/s)
    params.train.mass_tons_min = 180.0;                 % Min train mass (tons)
    params.train.mass_tons_max = 260.0;                 % Max train mass (tons)
    params.train.davis_A_N = 5000.0;                    % Davis resistance constant (N)
    params.train.davis_B_N_per_mps = 100.0;             % Davis linear term (N/(m/s))
    params.train.davis_C_N_per_mps2 = 5.0;              % Davis quadratic term (N/(m/s)^2)
    params.train.gravity = 9.81;                        % Gravitational acceleration (m/s^2)
    params.train.speed_epsilon = 0.01;                  % Speed threshold for zero regime (m/s)
    params.train.p_trac_max_kw = 1000.0;                % Max traction power envelope (kW)
    params.train.p_regen_max_kw = 400.0;                % Max regen acceptance (kW)

    %% Auxiliary Load Parameters
    params.aux.p_base_kw = 200.0;                       % Base auxiliary power (kW)
    params.aux.bias_std = 2.0;                          % Random walk std dev (kW)
    params.aux.bias_init_min = -10.0;                   % Min initial bias (kW)
    params.aux.bias_init_max = 10.0;                    % Max initial bias (kW)

    %% Driveline Parameters
    params.driveline.eta_traction = 0.85;               % Traction efficiency
    params.driveline.eta_regen = 0.75;                  % Regen efficiency

    %% Cost Parameters (for reward calculation - optional)
    params.cost.h2_euro_per_kg = 6.0;                   % H2 cost (EUR/kg)
    params.cost.grid_euro_per_kwh = 0.18;               % Grid electricity cost (EUR/kWh)
    params.cost.lambda_smooth = 0.01;                   % Smoothness penalty weight
    params.cost.lambda_delay = 0.5;                     % Delay penalty weight
    params.cost.lambda_unmet = 1.0e-6;                  % Unmet demand penalty weight

    %% Observation Filter Parameters
    params.obs.preq_filter_tau = 3.0;                   % P_req low-pass filter time constant (s)
    params.obs.soc_noise_std = 0.02;                    % SOC measurement noise
    params.obs.tank_noise_std = 0.02;                   % Tank level measurement noise
    params.obs.nuisance_noise_std = 0.1;                % Nuisance observation noise

    %% Driver Parameters (Simple Driver)
    params.driver.mode = 'simple';                      % 'simple' or 'pid'
    params.driver.filter_alpha = 0.05;                  % Low-pass filter coefficient
    params.driver.rate_limit_kw_per_s = 500.0;          % P_req rate limit (kW/s)

    % Simple driver segments (example profile)
    % Each segment: duration_s, mode, base_power_kw, grade_percent
    % Modes: 'dwell', 'accelerate', 'cruise', 'climb', 'descent', 'brake'
    params.driver.segments = {
        60,  'dwell',      0,    0.0;
        120, 'accelerate', 400,  0.0;
        300, 'cruise',     300,  0.0;
        180, 'climb',      450,  2.0;
        240, 'cruise',     320,  0.5;
        90,  'descent',    150,  -1.5;
        150, 'cruise',     280,  0.0;
        60,  'brake',      -500, 0.0;
        90,  'dwell',      0,    0.0;
        120, 'accelerate', 380,  0.0;
        300, 'cruise',     290,  0.0;
        60,  'brake',      -600, 0.0;
        120, 'dwell',      0,    0.0;
    };

    %% EMS Parameters
    params.ems.policy = 'baseline';                     % 'baseline' or 'balanced'
    params.ems.fc_base_frac = 0.3;                      % FC fraction for base load (baseline EMS)
    params.ems.soc_target = 0.55;                       % Target SOC for battery management
    params.ems.soc_deadband = 0.05;                     % SOC deadband for battery control

    params.ems_balanced.target_soc = 0.80;
    params.ems_balanced.soc_high_margin = 0.05;
    params.ems_balanced.soc_full = 0.995;
    params.ems_balanced.charge_boost_kw = 40.0;
    params.ems_balanced.regen_clip_soc = 0.99;

end
