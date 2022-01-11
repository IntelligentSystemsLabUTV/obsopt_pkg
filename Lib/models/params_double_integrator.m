%% params_pendulum
% create params structure for pendulum 
function params = params_double_integrator

    % control params
    params.K1 = 0;
    params.K2 = 0;
    params.K3 = 0;
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 3;
    
    params.X(1).val(:,1) = [1;1;1;params.K1; params.K2; params.K3];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [4 5 6];
    
    % which vars am I optimising
    params.opt_vars = [4 5 6];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end