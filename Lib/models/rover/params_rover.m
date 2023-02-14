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
    params.g = 0*0.1;
    
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
    params.Kdu = [0 0];      
    params.Kz = [9000 190];
    params.Kff = [0 0 0];

    % number of reference trajectories (under development)
    params.Ntraj = 1;

    % control error derivative
    params.wlen_err = 4;
    params.buflen_err = 10;
    params.dim_err = 1;
    params.err_scale = 1;
    for traj = 1:params.Ntraj        
        params.err(traj).val = [];
        params.err_der(traj).val = [];
        params.err_int(traj).val = zeros(params.dim_err,1);
        params.err_der_buffer(traj).val = zeros(params.dim_err,params.buflen_err);
        params.err_der_counter(traj).val = 0;
    end 
    
    % different omega for trajectories
    for traj = 2:params.Ntraj
        params.wnx(traj) = params.wnx(1)*(1+rand());        
        params.wny(traj) = params.wny(1)*(1+rand());
    end
    
    % state dimension
    params.space_dim = 3;   % 2D or 3D space for the rover 

    % anchor stuff
    an_dp = 2.5;
    an_dz = 2;

    %%% gaussian stuff %%%
    ds = 1e-2*params.err_scale;
    [params.X_gauss, params.Y_gauss] = meshgrid(-an_dp:ds:an_dp, -an_dp:ds:an_dp);
    params.A_gauss = 0.5;
    params.sigma_gauss = 1;
    params.hill(1,:) = [-an_dp*1.5 -an_dp*1.5];
    params.hill(2,:) = [-an_dp*1.5 an_dp*1.5];
    params.hill(3,:) = [an_dp*1.5 an_dp*1.5];
    params.hill(4,:) = [an_dp*1.5 -an_dp*1.5];
    params.G_gauss = [];
    for hills=1:size(params.hill,1)
        try
            params.G_gauss = 1*params.G_gauss + 1*params.A_gauss*exp(-1/(params.sigma_gauss^2)*((params.Y_gauss-params.hill(hills,2)/2).^2 + (params.X_gauss-params.hill(hills,1)/2).^2));
        catch
            params.G_gauss = 1*params.A_gauss*exp(-1/(params.sigma_gauss^2)*((params.Y_gauss-params.hill(hills,2)/2).^2 + (params.X_gauss-params.hill(hills,1)/2).^2));
        end
    end

    % multistart
    params.multistart = 0;

    % observer params    
    params.alpha = 0*[1 1];
    params.beta = 0*[1 -100.1];
    params.C = 0*[10 100.1];
    params.theta = 0*[1 0 0 0 0];

    % observer params    
%     params.alpha = 1*[-0.0067    6.4999];
%     params.beta = 1*[1.0000   83.8996];
%     params.C = 1*[243.9468  -83.8996];
%     params.theta = 1*[0.9328   -0.1371    0.9731   -0.1163    7.3477];    
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % hyb obs parameters
    params.dim_Gamma = length(params.C) + length(params.theta) +length(params.beta) + length(params.alpha);

    % model parameters
    params.dim_state = 5*params.space_dim + params.Nanchor*params.space_dim + params.dim_Gamma;    % done on the observer model (easier to compare)

    % shared position (hyb and EKF)
    params.pos_p = [1 6 11];   % see mode_rover.m
    params.pos_v = [2 7 12];   % see mode_rover.m  
    params.pos_acc = [3 8 13];
    % positions for hyb
    params.pos_jerk = [4 9 14];
    % positions for biases
    params.pos_bias = [5 10 15];   % IMU bias
    % rest of stuff
    params.pos_anchor = [5*params.space_dim+1:params.dim_state-params.dim_Gamma];    % after all the double integrators come the anchors   
    params.pos_Gamma = [params.pos_anchor(end)+1:params.dim_state];
    params.pos_fc = [params.pos_p params.pos_v];
    params.dim_state_est = numel(params.pos_fc);

    % input dim
    params.dim_input = params.space_dim;   % input on each dimension

    % output dim
    % distances + accelerations + velocity (only for learning) + position
    % (only for learning)
    params.OutDim = params.Nanchor + 3*params.space_dim;  
    params.observed_state = [];   % not reading the state    
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
    params.wlen = 4;
    params.buflen = 10;
    params.dim_pjump = params.space_dim;
    for traj = 1:params.Ntraj
        params.p_jump_time = [];
        params.p_jump(traj).val = [];
        params.p_jump_der(traj).val = [];
        params.p_jump_der_buffer(traj).val = zeros(params.dim_pjump,params.buflen);
        params.p_jump_der_counter(traj).val = 0;
    end    
    

    % noise (on distances + acceleration)
    params.noise_mat = 0*ones(params.OutDim,2);    
    % sigma
    params.noise_mat_original(params.pos_acc_out,1) = 1*3e-1;   % noise on IMU - sigma
    params.noise_mat_original(params.pos_dist_out,1) = 1*2e-1;  % noise on UWB - sigma    
    params.mean = params.noise_mat_original(:,1);
    params.noise_mat(:,1) = 0*params.noise_mat_original(:,1);    

    %%% process noise %%%
    params.jerk_enable = 0;

    %%%%%% EKF %%%%%
    % enable noise
    params.EKF = 0;        
    params.hyb = 1;
    params.dryrun = 0;

    %%% noise matrices
    % measurement noise
    params.R = diag([params.noise_mat_original(params.pos_dist_out,1).^2.*ones(params.Nanchor,1);     ...  % UWB         
                     zeros(numel([params.pos_p params.pos_v]),1);                           ...  % P,V
                     params.noise_mat_original(params.pos_acc_out,1).^2.*ones(params.space_dim,1);    ... % IMU ACC                     
        ]);      
    
    % process noise - model
    params.Q = 1e0*diag([1e0 1e0 1e0 1e0 1e0 1e0,... % JERK                     
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
    params.X(1).val(:,1) = 1*[2;0;0;0;0.1; ...                % x pos + IMU bias
                              2;0;0;0;0.2; ...                % y pos + IMU bias
                              0;0;0;0;0.1; ...                % z pos + IMU bias
                              -an_dp;-an_dp;an_dz;  ...
                              -an_dp;an_dp;an_dz;   ...
                              an_dp;an_dp;an_dz;    ...
                              an_dp;-an_dp;an_dz;   ...    % anchors                                                                          
                              params.C'; ...              % params                              
                              params.theta'; ...
                              params.beta'; ...
                              params.alpha'];  

    % hills on z - correct initialization
    p_now = params.X(1).val(params.pos_p(1:2),1);
    p_est = zeros(1,2);
    p_grid = [params.X_gauss(1,:); params.Y_gauss(:,1)'];
    for i=1:2
        pdiff = p_grid(i,:)-p_now(i);   
        p_est(i) = find(abs(pdiff) == min(abs(pdiff)),1,'first');                
    end
    z0 = params.G_gauss(p_est(1),p_est(2));
    params.X(1).val(params.pos_p(3),1) = z0; 
    
    % position in the state vector of the estimated parameters
    params.estimated_params = params.pos_Gamma;
    
    % which vars am I optimising
    params.opt_vars = [params.pos_Gamma];
%     params.opt_vars = [params.pos_p params.pos_v params.pos_acc];
    
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
