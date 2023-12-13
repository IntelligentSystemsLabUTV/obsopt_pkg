%% runaway init model
function params = params_runaway(varargin)

    % get varargin
    if numel(varargin) > 0
        Ntraj = varargin{1};
    else
        Ntraj = 1;
    end

    % electric charge
    params.Q = 1;

    % spontaneous emission
    params.S = 0;

    % parameters
    params.gamma = 0.5;
    params.gamma1 = 2.5;
    params.ni = 0.5;
    params.Wt = 0.1;

    % eps_coef
    params.eps_coef = 38;

    % state dimension
    params.dim_state = 3;

    % input dim
    params.dim_input = 2;

    % output dim
    params.OutDim = 1;
    params.OutDim_compare = 1;
    params.observed_state = 2;  

    % initial condition
    params.T0 = 9.88;
    params.W0 = 5.6e-6;
    
    % number of reference trajectories (under development)
    params.Ntraj = Ntraj;
    
    % reference init
    params.X(1).val(:,1) = [params.T0; params.W0; params.gamma];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the parameters
    params.estimated_params = [3];
    
    % which vars am I optimising
    params.opt_vars = [1:3];
    params.perturbed_vars = [1:3];
    
    % not opt vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;

    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:2;
    params.plot_params = 3;
end