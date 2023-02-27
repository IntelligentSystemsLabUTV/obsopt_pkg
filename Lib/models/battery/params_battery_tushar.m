%% PARAMS_BATTERY
% file: params_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function initialises the parameters for a battery
% model (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_battery_tushar(params_sim)

    %%%%%%%%%%% LOAD DATA OF A BATTERY EXPERIMENT %%%%%%%%%%%
        
    % SETUP THE EXPERIMENT  - Battery Capacity (converting Ampere-hour to Ampere-second)
    params.InputAmplitude = -1;
    params.C_n_h = 4.1*abs(params.InputAmplitude);
    params.C_n = params.C_n_h * 3600;                 

    %%% system parameters %%%
    % battery EMF
    params.Voc = 3.3295;
    % ohmic internal resistance
    params.R0 = 0.0118;
    % polarization resistance
    params.R1 = 0.0120;
    % polarization capacity
    params.C1 = 3.31111e3;    
    % Battery charging-discharging efficiency (for Li-ion=100%)
    params.eta = 1;  

    % noise characteristics
    noise = 1;
    params.percNoise = noise*5e-2;
    params.NoisePwr = noise*5e-3;

    % temperature
    params.Temperature = 313.15;

    % initial SOC
    x10 = 0.8;
    x20 = 0.01;    
    
    params.eps = 1;        
    
    %%%%%%% SETUP THE OBSERVER %%%%%
    % out vars - observer
    params.OutDim = 1;
    params.OutDim_compare = [1];

    % input dim - observer
    params.dim_input = 1;                    
       
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 30;

    % output dimension
    params.OutDim = 1;
    params.OutDim_compare = 1;

    % dim input
    params.dim_input = 1;
    
    % initial condition
    params.X(1).val(:,1) = zeros(params.dim_state,1);
    params.X(1).val(1:2,1) = [x10; x20];
    
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
    params.plot_vars = 1:2;
    params.plot_params = [4:6];%[7:14];
    params.multi_traj_var = [1:2];

    % add stuff
    params.multistart = 0;
    params.observed_state = [];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
        params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*randn(length(params.multi_traj_var),1);
%         params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*[1.5; 0.5; 1.5; 0.5];
    end

    %%% import data from sim %%%
    sim = params_sim.out;
    params.u_sim = sim.simout.u.Data';
    params.y_sim = sim.simout.ECM_Vb_noise.Data';
    params.y_true_sim = sim.simout.ECM_Vb.Data';
    params.soc_sim = sim.simout.ECM_soc.Data';

    params = first_guess(params,params_sim);
    
end