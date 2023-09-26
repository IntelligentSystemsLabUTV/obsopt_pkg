%% PARAMS_OSCILLATOR_VDP
% file: params_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function initialises the parameters for a Van der Pol
% oscillator (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_oscillator_VDP(varargin)

    % get varargin
    if numel(varargin) > 0
        Ntraj = varargin{1};
    else
        Ntraj = 1;
    end

    % system parameters
    params.mu = 0;
    params.A_mu = 0;
    params.F_mu = 0;
    params.Phi_mu = 0;
    params.eps = 0.01;
    
    % control parameters
    params.K1 = 0.1;
    params.K2 = 0.1;
    
    % number of reference trajectories (under development)
    params.Ntraj = Ntraj;
    
    % state dimension
    params.dim_state = 3;

    % input dim
    params.dim_input = 2;

    % output dim
    params.OutDim = 1;
    params.OutDim_compare = [1];
    params.observed_state = 2;    
    
    % initial condition
    % params.X(1).val(:,1) = [1;1;params.mu;params.A_mu;params.F_mu];
    params.X(1).val(:,1) = [1;1;params.mu];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [3];
    
    % which vars am I optimising
    params.opt_vars = [1:3];
    params.perturbed_vars = [1:3];
    
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
    params.plot_params = 3;
    params.multi_traj_var = params.nonopt_vars;
end
