%% PARAMS_BATTERY
% file: params_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function initialises the parameters for a battery
% model (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_battery_fedeoli

    % system parameters
    % open circuit voltage
    params.Voc = 5;
    % constant charge resistance
    params.Rc = 2;
    % cinstant discharge resistance
    params.Rd = 3;
    % capacitor
    params.C = 1;
    % terminal voltage
    params.Vb = 1;    
    % Rb
    params.Rb = 1;
    
    % charge-discharge period
    params.period = 5;
    params.Rx = params.Rc;
    
    % control parameters
    params.K1 = 0.1;
    params.K2 = 0.1;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 5;
    
    % initial condition
    % state variables [Carnevale 2019]:
    % x1 = Vp
    % x2 = Rx    
    % x3 = Voc
    % x4 = C
    % x5 = Rb
    params.X(1).val(:,1) = [1;params.Rx;params.Voc;params.C;params.Rb];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [2:5];
    
    % which vars am I optimising
    params.opt_vars = [1:5];
    
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