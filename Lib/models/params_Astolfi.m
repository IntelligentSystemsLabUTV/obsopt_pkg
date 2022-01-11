%% params_pendulum
% create params structure for pendulum 
function params = params_Astolfi

    % pendulum parameters
    params.T = 0.05;
    params.Pm = 0.8;
    params.D = 1;
    params.E = 1.06679;
    params.V = 1;
    params.Tdc = 0.05;
    
    params.Delta_star = 0.2298;
    params.Lambda_star = params.Pm/(params.E*params.V*sin(params.Delta_star));
    
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    
    % reference init
    params.X(1).val(:,1) = [params.Delta_star;0;params.Lambda_star];
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