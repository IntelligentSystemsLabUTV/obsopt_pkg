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

    % system parameters
    % battery EMF
    params.Voc = 4.1924;
    % ohmic internal resistance
    params.R0 = 0.0116;
    % polarization resistance
    params.R1 = 0.0028114;
    % polarization capacity
    params.C1 = 9118;
    % Battery Capacity (converting Ampere-hour to Ampere-second)
    params.C_n = 28 * 3600; 
    % Battery charging-discharging efficiency (for Li-ion=100%)
    params.eta = 1;
  
    
    % control parameters
    params.K1 = 0.1;
    params.K2 = 0.1;
    params.period = 1;

    % Loading input signals and parameter data
    input_data = load('.\data\ECM_parameters.mat');
    params.input_time = input_data.Time;
    params.input_current = input_data.Current;
    params.input_OCV = input_data.OCV;
    params.input_soc = input_data.SOC;
    params.input_R0 = input_data.R0;
    params.input_R1 = input_data.R1;
    params.input_C1 = input_data.C1;
    
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 2;
    
    % initial condition
    params.X(1).val(:,1) = [0.9; 0.01];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [];
    
    % which vars am I optimising
    params.opt_vars = [1:2];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = params.dim_state;
    params.plot_params = 1;
    
end