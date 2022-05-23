%% runaway init model
function params = params_SQR

    % plant data

    % electric charge
    params.a = 0.9;
    params.b = 0.4;

    % eps_coef
    params.eps_coef = 2;
    
    % number of reference trajectories (>1 for control design)
    params.Ntraj = 1;
    params.dim_state = 5;
    
    % reference init
%     params.X(1).val(:,1) = [2; 0.5; 1];
    params.X(1).val(:,1) = [2; 0.5; 1; params.a; params.b];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [4:5];
    
    % which vars am I optimising
    params.opt_vars = [1:5];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    params.plot_vars = params.dim_state;
end