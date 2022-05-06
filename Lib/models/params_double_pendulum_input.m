%% params_pendulum
% create params structure for pendulum 
function params = params_double_pendulum_input

    % pendulum parameters
    params.Lt1 = 1;
    params.M1 = 1;
    params.Lt2 = 1;
    params.M2 = 1;
    params.g = 0*9.81;
    params.c1 = 0.1;
    params.c2 = 0.1;
    
    % control params
    init_val = 1e-1;
    params.K1 = init_val;
    params.K2 = init_val;
    params.K3 = init_val;
    params.K4 = init_val;
    params.K5 = init_val;
    params.K6 = init_val;
    params.K7 = init_val;
    params.K8 = init_val;
    
    params.K = [params.K1,params.K2,params.K3,params.K4;...
                params.K5,params.K6,params.K7,params.K8];
%     params.K = [params.K1,params.K2;...
%                 params.K3,params.K4];

    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    params.dim_state = 4;
    
    % reference init    
    params.X(1).val(:,1) = [pi/4;pi/3;0;0; params.K1; params.K2; params.K3; params.K4; params.K5; params.K6; params.K7; params.K8];
    for traj=2:params.Ntraj
        params.X(traj).val = params.X(traj-1).val(:,1);        
    end    
    
    % position in the state vector of the parameters
    params.estimated_params = [5:12];
    
    % which vars am I optimising
    params.opt_vars = [5:12];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars
    params.plot_vars = 4;
end