%% PARAMS_OSCILLATOR_VDP
% file: params_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function initialises the parameters for a Van der Pol
% oscillator (both for simulation and observation)
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_VolterraLotka

    % system parameters
    params.mu = 0;
    params.a = 1*0.01;
    params.b = 1*0.005;
    params.c = 1*0.001;
    params.d = 1*0.05;

    % prova a non fa morire tutti, come fai?

    % final
    params.mu = 0;
    params.a = 1*0.3;
    params.b = 1*0.005;
    params.c = 1*0.004;
    params.d = 1*0.1;

    G0 = 100;
    L0 = 100;

    %     G0 = 0;
    %     L0 = 0;

%     G0 = params.d/params.c;
%     L0 = params.a/params.b;

    % control
    params.K1 = 1*-1;
    params.K2 = 0*1;
    params.umax = 50;
    params.target = [50, 50];
        
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 2;

    % input dim
    params.dim_input = 2;

    % output dim
    params.OutDim = 2;
    params.OutDim_compare = [1 2];
    params.observed_state = [1 2];    
    
    % initial condition
    params.X(1).val(:,1) = [G0;L0];
    
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
    params.plot_vars = 1:3;
    params.plot_params = 4:5;
    params.multi_traj_var = params.nonopt_vars;
end
