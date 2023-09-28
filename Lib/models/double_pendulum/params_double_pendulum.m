%% PARAMS_DOUBLE_PENDULUM
% file: params_double_pendulum.m
% author: Federico Oliva
% date: 15/06/2022
% description: this function initialises the parameters for a double
% pendulum (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_double_pendulum(varargin)

    % get varargin
    if numel(varargin) > 0
        Ntraj = varargin{1};
    else
        Ntraj = 1;
    end

    % system parameters
    params.Lt1 = 1;
    params.M1 = 1;
    params.Lt2 = 1;
    params.M2 = 1.5;
    params.g = 1*9.81;
    params.c1 = 0.5;
    params.c2 = 0.5;  
    
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
    params.Ntraj = Ntraj;
    
    % state dimension
    params.dim_state = 6;

    % input dim
    params.dim_input = 2;

    % output dim
    params.OutDim = 2;
    params.OutDim_compare = [1 2];
    params.observed_state = [1 2];  
    
    % initial condition
    % state estimation
    params.X(1).val(:,1) = [-pi/4;-pi/4;0.2;-0.2;params.M1;params.M2];
    % control design
%     params.X(1).val(:,1) = [pi/6;pi/6;0;0;params.K1;params.K2;params.K3;params.K4];%;params.K5;params.K6;params.K7;params.K8];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [5:6];
    
    % which vars am I optimising
    params.opt_vars = [1:6];
    params.perturbed_vars = [1:6];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
        
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 3:4;
    params.plot_params = [5:6];
end