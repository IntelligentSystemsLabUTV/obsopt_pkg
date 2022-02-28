%% params_pendulum
% create params structure for pendulum 
function params = params_oscillator_VDP

    % params 
    params.mu = 0.1;
    
    % control
    params.K1 = 0.1;
    params.K2 = 0.1;
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    
    % reference init
    params.X(1).val(:,1) = [1;1]*1e0;
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [];
    
    % which vars am I optimising
    params.opt_vars = [1:2];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
end