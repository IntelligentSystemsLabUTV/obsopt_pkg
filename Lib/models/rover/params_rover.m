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
%     params.wnx = [0.2 1];
%     params.wny = [0.2 1];
    params.wnx = [0.4];
    params.wny = [0.9];
    params.Ax = -[0.5 1];
    params.Ay = -[0.5 1];     
    params.Ax_tot = sum(params.Ax);
    params.Ay_tot = sum(params.Ay);
    params.phi = [0 pi/2];
    params.rhox = 0.01;
    params.rhoy = 0.01;
    % vines
    params.freq_u = 48;    
    params.amp_ux = -1/3;
    params.amp_uy = -1/3;
    params.Ku = [10 10];    

    % number of reference trajectories (under development)
    params.Ntraj = 1;
    
    % different omega for trajectories
    for traj = 2:params.Ntraj
        params.wnx(traj) = params.wnx(1)*(1+rand());        
        params.wny(traj) = params.wny(1)*(1+rand());
    end
    
    % state dimension
    params.space_dim = 2;   % 2D or 3D space for the rover 

    % multistart
    params.multistart = 0;

    % observer params    
    params.alpha = 0*[1 1];
    params.beta = 0*[1 1];
    params.C = 0*[1 1];
    params.theta = 0*[1 1 1 1 1];

    % observer params    
%     params.alpha = 1*[ -2.3167 0.0673];
%     params.beta = 1*[1.0000   39.7342];
%     params.C = 1*[ -148.4165  -39.7342];
%     params.theta = 1*[0.4225         0   -0.0117  -17.2938];    
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % hyb obs parameters
    params.dim_Gamma = length(params.C) + length(params.theta) + +length(params.beta) + length(params.alpha);

    % model parameters
    params.dim_state = 4*params.space_dim + params.Nanchor*params.space_dim + params.dim_Gamma;    % done on the observer model (easier to compare)
    params.pos_p = [1 5];   % see mode_rover.m
    params.pos_v = [2 6];   % see mode_rover.m  
    params.pos_acc = [3 7];
    params.pos_jerk = [4 8];
    params.pos_anchor = [4*params.space_dim+1:params.dim_state-params.dim_Gamma];    % after all the double integrators come the anchors   
    params.pos_Gamma = [params.pos_anchor(end)+1:params.dim_state];
    params.pos_fc = [params.pos_p params.pos_v];
    params.dim_state_est = numel(params.pos_fc);

    % input dim
    params.dim_input = params.space_dim;   % input on each dimension

    % output dim
    params.OutDim = params.Nanchor + params.space_dim + 2*params.space_dim;  % distances + accelerations + velocity (only for learning) + position (only for learning)    
    params.observed_state = [params.pos_v];   % not reading the state    
    params.pos_dist_out = 1:params.Nanchor;
    params.pos_acc_out = [params.Nanchor + 2*params.space_dim + 1:params.OutDim];
    params.pos_v_out = [params.Nanchor + params.space_dim + 1:params.Nanchor + 2*params.space_dim];
    params.pos_p_out = [params.Nanchor + 1:params.Nanchor + params.space_dim];
    params.OutDim_compare = [params.pos_p_out params.pos_v_out];   % distances
    
    % sampling
    params.IMU_samp = 1;
    params.UWB_samp = 20;
    params.UWB_pos = []; 

    % memory
    params.last_noise = zeros(params.Ntraj,params.OutDim);
    params.last_D = zeros(params.Ntraj,params.Nanchor);
    params.last_D_ref = zeros(params.Ntraj,params.Nanchor);
    params.last_IMU_acc = zeros(params.Ntraj,numel(params.pos_acc_out));
    params.last_IMU_acc_ref = zeros(params.Ntraj,numel(params.pos_acc_out));

    % derivative of the pjump
    for traj = 1:params.Ntraj
        params.p_jump_time = [];
        params.p_jump(traj).val = [];
        params.p_jump_der(traj).val = [];
    end
    clear PseudoDer
    params.wlen = 4;
    params.buflen = 10;

    % noise (on distances + acceleration)
    params.noise_mat = 0*ones(params.OutDim,2);
    % bias 
    params.noise_mat_original(params.pos_acc_out,1) = 1*3e-1;   % noise on IMU - bias 
    params.noise_mat_original(params.pos_dist_out,1) = 1*7e-2;  % noise on UWB - bias
    params.bias = params.noise_mat_original(:,1);
    % sigma
    params.noise_mat_original(params.pos_acc_out,2) = 1*1e-1;   % noise on IMU - sigma
    params.noise_mat_original(params.pos_dist_out,2) = 1*2e-1;  % noise on UWB - sigma    
    params.mean = params.noise_mat_original(:,2);
    params.noise_mat = 0*params.noise_mat_original;

    %%% process noise %%%
    params.jerk_enable = 0;

    %%%%%% EKF %%%%%
    % enable noise
    params.EKF = 0;        
    params.hyb = 1;
    params.dryrun = 0;

    %%% noise matrices
    % measurement noise
    params.R = diag([params.noise_mat_original(params.pos_dist_out,2).^2.*ones(params.Nanchor,1);     ...  % UWB         
                     zeros(numel([params.pos_p params.pos_v]),1);                           ...  % P,V
                     params.noise_mat_original(params.pos_acc_out,2).^2.*ones(params.space_dim,1);    ... % IMU ACC                     
        ]);      
    
    % process noise - centripetal model
    params.Q = 1e0*diag([1e0 1e0,... % JERK                     
        ]);

    % EKF covariance matrix
    for traj=1:params.Ntraj
        params.Phat(traj).val(1,:,:) = 1e0*eye(params.dim_state);
    end
    %%%%%%%%%%%%%%%%

    %%%%%% GENERAL OBS %%%%%
    % observer stuff
    params.time_J = [];
    params.d_true = zeros(params.Nanchor,1);
    params.d_noise = zeros(params.Nanchor,1);
    params.d_est = zeros(params.Nanchor,1);
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    % initial condition
    params.X(1).val(:,1) = 1*[2;0;0;0; ...                % x pos
                              4;0;0;0; ...                % y pos
                              -6;-6;-6;6;6;6;6;-6; ...    % anchors                                                                          
                              params.C'; ...              % params                              
                              params.theta'; ...
                              params.beta'; ...
                              params.alpha'];    
    
    % position in the state vector of the estimated parameters
    params.estimated_params = params.pos_Gamma;
    
    % which vars am I optimising
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
    params.multi_traj_var = [params.pos_p]; 
    pos_init = [3 3;  ...
                -3 3; ...
                -3 -3; ...
                3 -3];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(1).val(:,1);

        % random
        params.X(traj).val(params.multi_traj_var,1) = params.X(1).val(params.multi_traj_var,1).*(1 + 0*5e-1*randn(length(params.multi_traj_var),1));

        % from starting positions
%         params.X(traj).val(params.pos_p,1) = pos_init(traj,:);
    end    
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = [params.pos_p params.pos_v];
    params.plot_params = [3 7];
    params.dim_out_plot = [params.pos_p_out params.pos_v_out];       

    % fminunc
    params.dist_optoptions = optimoptions('fminunc', 'MaxIter', 1, 'display','off');
end
