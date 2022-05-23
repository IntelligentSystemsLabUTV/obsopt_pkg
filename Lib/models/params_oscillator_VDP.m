%% params_pendulum
% create params structure for pendulum 
function params = params_oscillator_VDP

    % system
    params.mu = 0.5;
    params.eps = 1;
    
    % control
    params.K1 = 0.1;
    params.K2 = 0.1;
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    params.dim_state = 3;
    
    % reference init
%     params.X(1).val(:,1) = [1;1];
    params.X(1).val(:,1) = [1;1;params.mu];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [3];
    
    % which vars am I optimising
    params.opt_vars = [1:3];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars
    params.plot_vars = 3;
end