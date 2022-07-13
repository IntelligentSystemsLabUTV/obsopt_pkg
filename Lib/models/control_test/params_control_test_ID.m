%% PARAMS_OSCILLATOR_VDP
% file: params_control_test.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function initialises the parameters for an unstable LTI
% model
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_control_test_ID

    % system parameters
    
    % control parameters
    params.K1 = 815.1611*0;
    params.K2 =  -548.2666*0;
    params.K3 = -49.6254*0;
    params.K4 = -2.5068e+04*0;    
    % model parameters
    params.A1 = 1*1;
    params.A2 = 1*-1;
    params.A3 = 1*1;
    params.A4 = 1*1;
    params.B1 = 1*1;
    params.B2 = 1*2;
    params.C1 = 1*2;
    params.C2 = 1*1;
    
    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 8;  
    
    % initial condition
%     params.X(1).val(:,1) = [1;1];
    params.X(1).val(:,1) = [1;1;params.A1;params.A2;params.A3;params.A4;params.B1;params.B2];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [3:8];
    
    % which vars am I optimising
    params.opt_vars = [3:8];
    
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
    params.plot_params = [3:8];  
    params.multi_traj_var = 1:2;
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
        params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*randn(length(params.multi_traj_var),1);
%         params.X(traj).val(params.multi_traj_var,1) = params.X(traj-1).val(params.multi_traj_var,1) + abs(params.X(traj-1).val(params.multi_traj_var,1)).*[1.5; 0.5; 1.5; 0.5];
    end
end