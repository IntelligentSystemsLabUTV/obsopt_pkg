%% PARAMS_ROVER
% file: params_rover.m
% author: Federico Oliva
% date: 30/11/2022
% description: this function initialises the parameters for a double
% integrator
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_rover

    % system parameters
    params.m = 1;
    params.eps = 5;    
    params.Nanchor = 3;
    
    % control parameters
    params.wnx = 0.2;
    params.wny = 0.3;
    params.rhox = 0.3;
    params.rhoy = 0.3;

    % params uwb
    params.display_uwb = false;
    params.bias = false;
    params.epsilon = 1e-4;
    params.method = 1;    

    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % state dimension
    params.dim_state = 4 + 2*params.Nanchor;
    params.pos_p = [1 2];
    params.pos_v = [3 4];
    params.pos_anchor = [5:params.dim_state];    

    % input dim
    params.dim_input = 2;

    % output dim
    params.OutDim = params.dim_state + params.Nanchor + params.dim_input;       % rover position and anchor positions + distances + accelerations
    params.OutDim_compare = [params.dim_state + 1 + params.Nanchor:params.OutDim];               %[1 2 (params.OutDim-params.Nanchor+1):params.OutDim];
    params.observed_state = [1:params.dim_state];                               % not reading the state    
    params.pos_dist = params.dim_state+1:params.dim_state+params.Nanchor;
    
    % initial condition
    params.X(1).val(:,1) = 1*[1;1;0;0;0;1;0;-1;1;0];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(traj-1).val(:,1);
    end
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [6:10];
    
    % which vars am I optimising
    params.opt_vars = [1:4];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = 1:4;
    params.plot_params = 5:params.dim_state;
    params.dim_out_plot = params.OutDim_compare;    %[1:2];
    params.multi_traj_var = params.nonopt_vars;

    %%% J sym analysis
    syms x y    
    syms D_vec [1 params.Nanchor]    
    params.x = x;
    params.y = y;
    params.D_vec = D_vec;
    params.symList = [x y D_vec];
    params.Nsyms = length(params.symList);
    tmp = params.X(1).val(5:end,1);
    params.P_a(1,:) = tmp(1:2:end);
    params.P_a(2,:) = tmp(2:2:end);
    P_a = params.P_a;

    params.J = @(x,y,P_a,D_vec)sum((sqrt( (P_a(1,:)-x).^2 + (P_a(2,:)-y).^2 ) - D_vec).^2);
    tmp = simplify(gradient(params.J(x,y,P_a,D_vec),[x y]));
    params.GJ = @(x,y,P_a,D_vec,params)double(subs(tmp,params.symList,[x y D_vec]));
    tmp = simplify(hessian(params.J(x,y,P_a,D_vec),[x y]));
    params.HJ = @(x,y,P_a,D_vec,params)double(subs(tmp,params.symList,[x y D_vec]));
end
