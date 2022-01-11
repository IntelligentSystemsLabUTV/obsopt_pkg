%% params_pendulum
% create params structure for pendulum 
function params = params_double_pendulum_default

    % pendulum parameters
    params.Lt1 = 1;
    params.M1 = 1;
    params.Lt2 = 1;
    params.M2 = 1;
    params.g = 9.81;
    params.c1 = 0.1;
    params.c2 = 0.1;  
    
    % control params
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
   
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    
    % reference init
    params.X(1).val(:,1) = [-pi/4;-pi/4;0.2;-0.2; params.c1; params.c2];
%     params.X(1).val(:,1) = [0;0;0;0; params.K1; params.K2; params.K3; params.K4; params.K5; params.K6];
%     params.X(1).val(:,1) = [0;0;0;0; params.K1; params.K2; params.K3; params.K4; params.K5; params.K6; params.K7; params.K8; params.K9; params.K10];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [5:6];
    
    % which vars am I optimising
    params.opt_vars = [1:6];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end