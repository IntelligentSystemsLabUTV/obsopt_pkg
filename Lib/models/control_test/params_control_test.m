%% PARAMS_OSCILLATOR_VDP
% file: params_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function initialises the parameters for an unstable LTI
% model
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_control_test

    % system parameters
    
    % control parameters
    params.K1 = 5.0915e+00*0;
    params.K2 = -1.1188e+01*0;
    params.K3 = -20*0;
    params.K4 = -5000*0;
    params.K5 = 0;
    params.K6 = 0;
    params.K7 = 0;
    params.K8 = 0;
    
    % number of reference trajectories (under development)
    params.Ntraj = 4;
    
    % state dimension
    params.dim_state = 6;
    
    % initial condition
%     params.X(1).val(:,1) = [1;1];
    params.X(1).val(:,1) = [1;1;params.K1;params.K2;params.K3;params.K4]; 
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [3:6];
    
    % which vars am I optimising
    params.opt_vars = [3:4];
    
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
    params.plot_params = [3:6];
end