%% PARAMS_DOUBLE_PENDULUM
% file: params_double_pendulum.m
% author: Federico Oliva
% date: 15/06/2022
% description: this function initialises the parameters for a double
% pendulum (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_double_pendulum

    % system parameters
    params.Lt1 = 1;
    params.M1 = 1;
    params.Lt2 = 1;
    params.M2 = 1;
    params.g = 9.81;
    params.c1 = 0.1;
    params.c2 = 0.1;  
    
    % control parameters
    params.K1 = 0;
    params.K2 = 0;
    params.K3 = 0;
    params.K4 = 0;
    params.K5 = 0;
    params.K6 = 0;
    params.K7 = 0;
    params.K8 = 0;
    params.K9 = 0;
    params.K10 = 0;
   
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dymension
    params.dim_state = 4;
    
    % initial condition
    params.X(1).val(:,1) = [-pi/4;-pi/4;0.2;-0.2];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [];
    
    % which vars am I optimising
    params.opt_vars = [1:4];
    
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