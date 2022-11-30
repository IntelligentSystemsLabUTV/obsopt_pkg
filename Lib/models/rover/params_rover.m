%% PARAMS_ROVER
% file: params_rover.m
% author: Federico Oliva
% date: 30/11/2022
% description: this function initialises the parameters for a double
% integrator
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_rover

    % system parameters
    params.m = 1;
    params.eps = 5;    
    params.Nanchor = 3;
    
    % control parameters
    params.wnx = 1;
    params.wny = 0.5;
    params.rhox = 0.1;
    params.rhoy = 0.05;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 4 + 2*params.Nanchor;

    % input dim
    params.dim_input = 2;

    % output dim
    params.OutDim = 2 + 2*params.Nanchor; % rover position and anchor positions
    params.OutDim_compare = [1 2];
    params.observed_state = [1:2 5:params.dim_state]; % not reading the state    
    
    % initial condition
    params.X(1).val(:,1) = [1;1;0;0;0;1;0;-1;1;0];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [];
    
    % which vars am I optimising
    params.opt_vars = [];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:4;
    params.plot_params = 5:params.dim_state;
    params.dim_out_plot = [1:2];
    params.multi_traj_var = params.nonopt_vars;
end
