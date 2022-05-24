%% params_pendulum
% create params structure for pendulum 
function params = params_battery

    % battery parameters
    params.VoC = 1; 
    params.R0 = 1;
    params.R1 = 1;
    params.C1 = 1;
    params.eta = 100;
    params.Q = 4;
    
    % target parameter
    params.Uoc = 3.2;
    
    % control params
   
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    params.dim_state = 2;
    
    % reference init
    params.X(1).val(:,1) = [1;2];
    
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
    
    % plot vars
    params.plot_vars = params.dim_state;
end
