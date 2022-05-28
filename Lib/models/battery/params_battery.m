%% PARAMS_BATTERY
% file: params_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function initialises the parameters for a battery
% model (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_battery

    % system parameters
    params.Voc = 5;
    params.R0 = 2;
    params.R1 = 2;
    params.C1 = 1;
    params.Cn = 1;
    
    params.alpha = [1 1 1 1];
    params.beta_1 = [1 1 1 1];
    params.beta_2 = [1 1 1 1];
    params.beta_3 = [1 1 1 1];
    params.beta_4 = [1 1 1 1];
    
    % control parameters
    params.K1 = 0.1;
    params.K2 = 0.1;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 6;
    
    % initial condition
    params.X(1).val(:,1) = [1;1;params.Voc;params.R0;params.R1;params.C1];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [3:6];
    
    % which vars am I optimising
    params.opt_vars = [1:6];
    
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
end