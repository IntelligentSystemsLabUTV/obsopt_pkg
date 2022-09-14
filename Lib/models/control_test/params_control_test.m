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

    rho = 0.5;
    omega = 2*pi*10;
    
    % control parameters
    params.a0 = 1*1;
    params.a1 = 1*1;
    params.b0 = 0*1;
    params.b1 = 1*1;    
    params.c0 = 1;
    params.c1 = 0;
    params.d0 = 0;    
    
    % true model parameters
    params.A1 = 1*-2*rho*omega;
    params.A2 = 1*-omega^2;
    params.A3 = 1*1;
    params.A4 = 1*0;
    params.B1 = 1*1;
    params.B2 = 1*0;
    params.C1 = 1*0;
    params.C2 = 1*omega^2;
    
    % estimated model parameters
    params.a0est = 1*-omega^2;                %1*-omega^2;
    params.a1est = 1*-2*rho*omega;            %1*-2*rho*omega;
    params.b0est = 1*0;                       %1*0;
    params.b1est = 1*1;                       %1*1;
    params.c0est = 1*omega^2;                 %1*omega^2;
    params.c1est = 1*0;                       %1*0;
    params.d0est = 0*0;                       %1*0;
    
    % reference model
    params.alpha = -50;
    params.p = 2;
    params.a = 2;
    params.offset = -1;
    params.dc = 0.5;
    
    % identification input
    params.rescale = 1e0;
    
    % input stuff
    params.dim_input = 3;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 16;  
    
    % initial condition
    % [xpc, xc, xr, xpi, @P, @C]
    params.X(1).val(:,1) = [0;0;0;0;0;0;0;params.a0est;params.a1est;params.b0est;params.b1est;params.a0;params.a1;params.b0;params.b1;params.d0];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [8:16];
    
    % which vars am I optimising
    params.opt_vars = [12:16];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:7;
    params.plot_params = [8:16];  
    params.multi_traj_var = [1:2];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
        params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + 5e-3*randn(length(params.multi_traj_var),1);
        
        % only for models with both xp and xphat
        params.X(traj).val(3:4,1) = params.X(traj).val(params.multi_traj_var,1);
    end
end