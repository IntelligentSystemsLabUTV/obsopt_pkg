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
    params.Nanchor = 4;
    
    % control parameters
    % 2nd order system
    params.wnx = 0.04;
    params.wny = 0.09;
    params.rhox = 0.01;
    params.rhoy = 0.01; 
    params.eps_u = 20;
    % vines
    params.freq_u = 2;    
    params.amp_ux = -8;
    params.amp_uy = -1;
    params.Ku = 50;

    % params uwb
    params.display_uwb = false;
    params.bias = false;
    params.epsilon = 1e-4;
    params.method = 0;  
    params.grad_Niter = 1;    

    % number of reference trajectories (under development)
    params.Ntraj = 4;
    
    % state dimension
    params.space_dim = 2;   % 2D or 3D space for the rover 

    % observer params    
    params.multistart = 0;
    params.K = 0*[1 1];
    params.C = 0*[1 1 1 1];
    params.L = 0*[1 1];
    params.G = 0*[1 1 1 1];    
    params.alpha = 0*[1 1];

%     params.K = 1*[-0.2192    0.4626];
%     params.C = 1*[0.9986    1.3185    4.3863    6.3437];
%     params.L = 1*[19.6993    2.9125];
%     params.G = 1*[14.9582   69.7367   66.7096   17.1527];    
%     params.alpha = 0*[1 1 1 1];

    params.dim_Gamma = length(params.K) + length(params.C) + length(params.L) + length(params.G) + length(params.alpha);

    params.dim_state = 5*params.space_dim + params.Nanchor*params.space_dim + params.dim_Gamma;    % done on the observer model (easier to compare)
    params.pos_p = [1 6];   % see mode_rover.m
    params.pos_v = [2 7];   % see mode_rover.m
    params.pos_other = [5 10];
    params.pos_anchor = [5*params.space_dim+1:params.dim_state-params.dim_Gamma];    % after all the double integrators come the anchors   
    params.pos_Gamma = [params.pos_anchor(end)+1:params.dim_state];

    % input dim
    params.dim_input = params.space_dim;   % input on each dimension

    % output dim
    params.OutDim = params.Nanchor + params.space_dim + 2*params.space_dim;  % distances + accelerations + velocity (only for learning) + position (only for learning)    
    params.observed_state = [params.pos_v];   % not reading the state    
    params.pos_dist = 1:params.Nanchor;
    params.pos_acc = [params.Nanchor + 2*params.space_dim + 1:params.OutDim];
    params.pos_v_out = [params.Nanchor + params.space_dim + 1:params.Nanchor + 2*params.space_dim];
    params.pos_p_out = [params.Nanchor + 1:params.Nanchor + params.space_dim];
    params.OutDim_compare = [params.pos_p_out params.pos_v_out];   % distances
    % sampling
    params.IMU_samp = 1;
    params.UWB_samp = 20;
    params.UWB_pos = []; 

    % chore
    params.last_D = zeros(params.Ntraj,params.Nanchor);
    params.last_D_ref = zeros(params.Ntraj,params.Nanchor);

    % noise (on distances + acceleration)
    params.noise_mat = 0*ones(params.OutDim,3);
    params.noise_mat(params.pos_acc,3) = 0;        % noise on IMU - bias 
    params.noise_mat(params.pos_dist,3) = 1*7e-2;  % noise on UWB - bias
    params.noise_mat(params.pos_acc,1) = 1*5e-2;   % noise on IMU - mean
    params.noise_mat(params.pos_dist,1) = 1*2e-1;  % noise on UWB - mean
    params.bias = params.noise_mat(:,2);
    params.mean = params.noise_mat(:,1);

    % observer stuff
    params.time_J = [];
    params.d_true = zeros(params.Nanchor,1);
    params.d_noise = zeros(params.Nanchor,1);
    params.d_est = zeros(params.Nanchor,1);
    
    % initial condition
    params.X(1).val(:,1) = 1*[3;0;0;0;0; ...              % x pos
                              3;0;0;0;0; ...              % y pos
                              -4;-4;-4;4;4;4;4;-4; ...    % anchors                              
                              params.K'; ...              % params
                              params.C'; ...                              
                              params.L'; ...
                              params.G'; ...
                              params.alpha'];    
    
    % position in the state vector of the estimated parameters
    params.estimated_params = params.pos_Gamma;
    
    % which vars am I optimising
%     params.opt_vars = [params.pos_Gamma(1:end-length(params.alpha))];
    params.opt_vars = [params.pos_Gamma];
    
    % set the not optimised vars
    tmp = 1:length(params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(params.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=params.opt_vars(i)));
    end
    params.nonopt_vars = tmp_idx;

    % multistart
    if params.multistart
        Npoints = 3;
        decades = linspace(0,4,Npoints);
    
        for i=1:Npoints 
            params.MULTISTART(i,:) = repmat(decades(i),1,length(params.opt_vars));   
            params.MULTISTART(i,end-1:end) = params.X(1).val(params.opt_vars(end-1:end),1);
        end 
        params.tpoints = CustomStartPointSet(params.MULTISTART(:,1:end));
        params.X(1).val(params.opt_vars(1:end),1) = params.MULTISTART(1,1:end);
    end


    % same initial condition for all the trajectories (under development)
    params.multi_traj_var = [params.pos_p params.pos_v]; 
    pos_init = [3,3;  ...
                3 -3; ...
                -3 3; ...
                -3 -3];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(1).val(:,1);

        % random
        params.X(traj).val(params.multi_traj_var,1) = params.X(1).val(params.multi_traj_var,1).*(1 + 1*5e-1*randn(length(params.multi_traj_var),1));

        % from starting positions
        params.X(traj).val(params.pos_p,1) = pos_init(traj,:);
    end
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = [params.pos_p params.pos_v];
    params.plot_params = [3 8];
    params.dim_out_plot = [params.OutDim_compare params.pos_acc];       

    %%% J sym analysis
    syms x y    
    syms D_vec [1 params.Nanchor]    
    params.x = x;
    params.y = y;
    params.D_vec = D_vec;
    params.symList = [x y D_vec];
    params.Nsyms = length(params.symList);
    tmp = params.X(1).val(params.pos_anchor,1);
    params.P_a(1,:) = tmp(1:2:end);
    params.P_a(2,:) = tmp(2:2:end);
    P_a = params.P_a;

    params.dist_optoptions = optimoptions('fminunc', 'MaxIter', 1, 'display','off');

    params.J = @(x,y,P_a,D_vec)sum((sqrt( (P_a(1,:)-x).^2 + (P_a(2,:)-y).^2 ) - D_vec).^2);
    tmp = simplify(gradient(params.J(x,y,P_a,D_vec),[x y]));
    params.GJ = @(x,y,P_a,D_vec,params)double(subs(tmp,params.symList,[x y D_vec]));
    tmp = simplify(hessian(params.J(x,y,P_a,D_vec),[x y]));
    params.HJ = @(x,y,P_a,D_vec,params)double(subs(tmp,params.symList,[x y D_vec]));
end
