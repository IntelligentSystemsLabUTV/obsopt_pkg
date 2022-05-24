%% PARAMS_OSCILLATOR_VDP
% file: params_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function initialises the parameters for a Van der Pol
% oscillator (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_oscillator_VDP

    % system parameters
    params.mu = 0.5;
    params.eps = 1;
    
    % control parameters
    params.K1 = 0.1;
    params.K2 = 0.1;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 3;
    
    % initial condition
    params.X(1).val(:,1) = [1;1;params.mu];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [3];
    
    % which vars am I optimising
    params.opt_vars = [1:3];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 3;
end