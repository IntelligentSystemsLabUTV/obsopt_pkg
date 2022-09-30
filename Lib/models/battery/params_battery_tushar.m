%% PARAMS_BATTERY
% file: params_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function initialises the parameters for a battery
% model (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_battery_tushar

    % Loading input signals and parameter data
    input_data = load('data/ECM_parameters_updated.mat');
    params.input_time = input_data.Time;
    params.input_current = input_data.Current;
    params.input_OCV = input_data.OCV;
    params.input_soc = input_data.SOC';
    params.input_R0 = input_data.R0;
    params.input_R1 = input_data.R1;
    params.input_C1 = input_data.C1;

    % generate modular HPPC
    params.input_current_Ts = 1;
    params.startpos = 320;
    params.stoppos = 677;
    params.input_current_modular_period = params.stoppos-params.startpos;    
    params.input_current_modular_time = 0:params.input_current_modular_period;
    params.input_current_modular_time_dense = 0:params.input_current_Ts:params.input_current_modular_period;
    params.input_current_modular = interp1(params.input_current_modular_time,params.input_current(params.startpos:params.stoppos),params.input_current_modular_time_dense);
    
    % slow modular HPPC
    params.time_slow = 1;
    params.input_current_modular_period_slown = params.input_current_modular_period*params.time_slow;
    params.input_current_modular_time_slown_dense = 0:params.input_current_Ts:params.input_current_modular_period_slown;
    for i=1:length(params.input_current_modular_time_dense)
        params.input_current_modular_slown(params.time_slow*(i-1)+1:params.time_slow*(i-1)+params.time_slow) = params.input_current_modular(i);
    end
    
    % initial SOC
    x10 = 0.8;
    x20 = 0.01;

    % system parameters
    % battery EMF
    params.Voc = 3.5;
    % ohmic internal resistance
    params.R0 = 0.0125;
    % polarization resistance
    params.R1 = 0.040;
    % polarization capacity
    params.C1 = 500;
    % Battery Capacity (converting Ampere-hour to Ampere-second)
    params.C_n_h = 4.1;
    params.C_n = params.C_n_h * 3600;     
    % Battery charging-discharging efficiency (for Li-ion=100%)
    params.eta = 1;  
    
    % testing - init with correct vals
    params.Voc = spline(params.input_soc, params.input_OCV, x10);
    params.R0 = spline(params.input_soc, params.input_R0, x10);
    params.R1 = spline(params.input_soc, params.input_R1, x10);
    params.C1 = spline(params.input_soc, params.input_C1, x10);
    
    % control parameters
    params.K1 = 0.1;
    params.K2 = 0.1;
    params.period = 1;    
    
    % params dynamics
    params.alpha_Voc = 0*1e0;
    params.alpha_R0 = 0*1e0;
    params.alpha_R1 = 0*1e0;
    params.alpha_C1 = 0*1e0;
    
    params.beta_Voc = 0*1e-1;
    params.beta_R0 = 0*1e-1;
    params.beta_R1 = 0*1e-1;
    params.beta_C1 = 0*1e-1;
    
    params.gamma_Voc = 0*1e-1;
    params.gamma_R0 = 0*1e-1;
    params.gamma_R1 = 0*1e-1;
    params.gamma_C1 = 0*1e-1;
    
    params.delta_Voc = 0*1e-1;
    params.delta_R0 = 0*1e-1;
    params.delta_R1 = 0*1e-1;
    params.delta_C1 = 0*1e-1;
    
    params.eps_Voc = 0*1e-1;
    params.eps_R0 = 0*1e-1;
    params.eps_R1 = 0*1e-1;
    params.eps_C1 = 0*1e-1;
    
    params.xi_Voc = 0*1e-1;
    params.xi_R0 = 0*1e-1;
    params.xi_R1 = 0*1e-1;
    params.xi_C1 = 0*1e-1;
    
    params.eps = 1;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 30;
    
    % initial condition
    params.X(1).val(:,1) = [x10; x20; ...
                            params.Voc; params.R0; params.R1; params.C1; ...
                            params.alpha_Voc; params.alpha_R0; params.alpha_R1; params.alpha_C1; ...
                            params.beta_Voc; params.beta_R0; params.beta_R1; params.beta_C1; ...
                            params.gamma_Voc; params.gamma_R0; params.gamma_R1; params.gamma_C1; ...
                            params.delta_Voc; params.delta_R0; params.delta_R1; params.delta_C1; ...
                            params.eps_Voc; params.eps_R0; params.eps_R1; params.eps_C1; ...
                            params.xi_Voc; params.xi_R0; params.xi_R1; params.xi_C1];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [7:30];
    
    % which vars am I optimising
    params.opt_vars = [1:2 8:10 12:14 16:18 20:22];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:6;
    params.plot_params = [7:14];
    params.multi_traj_var = params.nonopt_vars;
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
        params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*randn(length(params.multi_traj_var),1);
%         params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*[1.5; 0.5; 1.5; 0.5];
    end
    
end