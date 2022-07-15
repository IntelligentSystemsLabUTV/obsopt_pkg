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
    params.K1 = 14.3420*0;
    params.K2 =  -34.5793*0;
    params.K3 = 0;
    params.K4 = 0;
    params.a0 = 0;
    params.a1 = 0;
    params.b0 = 0;
    params.b1 = 0;
    params.d0 = 0;
    
    % true model parameters
    params.A1 = 1*1;
    params.A2 = 1*-1;
    params.A3 = 1*1;
    params.A4 = 1*1;
    params.B1 = 1*1;
    params.B2 = 1*2;
    params.C1 = 1*2;
    params.C2 = 1*1;
    
    % estimated model parameters
    params.a0est = 0;
    params.a1est = 0;
    params.b0est = 0;
    params.b1est = 0;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 13;  
    
    % initial condition
%     params.X(1).val(:,1) = [1;1];
    params.X(1).val(:,1) = [0;0;0;0;params.a0est;params.a1est;params.b0est;params.b1est;params.a0;params.a1;params.b0;params.b1;params.d0];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [3:13];
    
    % which vars am I optimising
    params.opt_vars = [3:13];
    
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
    params.plot_params = [3:13];  
    params.multi_traj_var = 1:2;
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
        params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*randn(length(params.multi_traj_var),1);
%         params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*[1.5; 0.5; 1.5; 0.5];
    end
end