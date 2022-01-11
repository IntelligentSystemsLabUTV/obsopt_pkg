%% params_pendulum
% create params structure for pendulum 
function params = params_grizzle

    % pendulum parameters
    params.a = 5;
    params.b = 6.667;
    params.c = 0.4;
    params.d = 0.05;
    params.e = 0.01;
    params.f = 0.05;
    params.g = 0.02;
    params.h = 0.5;
    params.i = 2;
    
    params.u1 = 0.3;
    params.u2 = 0.0067;
    
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    
    % reference init
    params.X(1).val(:,1) = [0.2;0.02;0.005];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [];
    
    % which vars am I optimising
    params.opt_vars = [1:3];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end