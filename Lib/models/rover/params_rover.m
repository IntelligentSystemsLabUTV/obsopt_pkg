%% PARAMS_ROVER
% file: params_rover.m
% author: Federico Oliva
% date: 30/11/2022
% description: this function initialises the parameters for a double
% integrator
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_rover(varargin)

    % get varargin
    if numel(varargin) > 0
        Ntraj = varargin{1};
    else
        Ntraj = 1;
    end
    
    params.hyb = 1;
    params.ekf = 0;

    % system parameters
    params.m = 1;
    params.eps = 5;    
    params.Nanchor = 4;
    params.g = 0*0.1;
    % params.Ts = 1e-2;
    
    % control parameters
    % vines
    params.freq_u = 100;    
    params.amp_ux = -0.35;
    params.amp_uy = -0.35;
    params.Ku = 1e2*[1 1];    
    params.Kdu = [0 0];      
    params.Kz = 1*[9000 190];
    params.Kff = [0 0 0];

    % number of reference trajectories (under development)
    params.Ntraj = Ntraj;

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
    
    
    % state dimension
    params.space_dim = 3;    % 2D or 3D space for the rover
    params.rotation_dim = 3; % orientation in 3D

    % anchor stuff
    % pos anchors Mesh 1
    % AM1 = params.out.AM1(1:2,:);
    % AM1 = 4*[1 -1 1 -1; 1 1 -1 -1];    
    AM1 = [-0.40 -0.40 +2.48 +2.80; +4.20 -1.80 -2.20 +4.20];    
%     AM1 = [0 0 15 15; 0 4.7 4.7 0];    
    % square box
    an_dp = max(max(abs(AM1)));
    % height
    % an_dz = mean(params.out.AM1(3,:));
%     an_dz = 3.3;
    an_dz = 2;
    Nhillmax = 4;

    %%% gaussian stuff %%%
    % different hill configuration for each traj
    ds = 1e-2*params.err_scale;
    [params.X_gauss, params.Y_gauss] = meshgrid(-an_dp:ds:an_dp, -an_dp:ds:an_dp);
    for traj = 1:params.Ntraj

        params.A_gauss(traj) = 1*rand();
        params.sigma_gauss(traj) = 3 + (5-3)*rand();
        ds = 1;
        ranges = [-an_dp*ds an_dp*ds; -an_dp*ds an_dp*ds];
        Nhill = randi(Nhillmax,1);
        for hill = 1:Nhill
            x = ranges(1,1) + (ranges(1,2)-ranges(1,1))*rand();
            y = ranges(2,1) + (ranges(2,2)-ranges(2,1))*rand();
            params.hill(traj).val(hill,:) = [x y];
        end     

        params.G_gauss(traj).val = [];
        for hill=1:Nhill
            try
                params.G_gauss(traj).val = 1*params.G_gauss(traj).val + 1*params.A_gauss(traj)*exp(-1/(params.sigma_gauss(traj)^2)*((params.Y_gauss-params.hill(traj).val(hill,2)/2).^2 + (params.X_gauss-params.hill(traj).val(hill,1)/2).^2));
            catch
                params.G_gauss(traj).val = 1*params.A_gauss(traj)*exp(-1/(params.sigma_gauss(traj)^2)*((params.Y_gauss-params.hill(traj).val(hill,2)/2).^2 + (params.X_gauss-params.hill(traj).val(hill,1)/2).^2));
            end
        end

    end

    % tags    
    L = 0.19;
    Z = 0.184;
    params.TagPos = [-L             +0.0            +1*Z;
                     +L*cos(pi/3)   -L*sin(pi/3)    +1*Z;
                     +L*cos(pi/3)   +L*sin(pi/3)    +1*Z]';
    params.Ntags = size(params.TagPos,1);
    
    % multistart
    params.multistart = 0;

    %%% observer params %%%
    % theta
    params.theta = 1*[0.4221    0.2888   -0.0281];
%     params.theta = 0*[0.8    0.2888   -0.1];    
    params.gamma = 0*ones(1,16);
%     params.gamma(1:3) = 1*[1.8112, 0.6373, 1.0015];    
    params.gamma(1:3) = 0*[0.1, -0.2, -0.3];    

    % alpha
    params.alpha = 0*[0 0];      

    % filter
    params.lowpass = 100;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % hyb obs parameters
    params.dim_Gamma = length(params.theta) + length(params.gamma) + length(params.alpha);

    % model parameters
    params.dim_state = 4*params.space_dim + (3*params.rotation_dim + 1) + params.Nanchor*params.space_dim + params.dim_Gamma;    % done on the observer model (easier to compare)

    % shared position (hyb and EKF)
    params.pos_p = [1 5 9];         % see mode_rover.m
    params.pos_v = [2 6 10];        % see mode_rover.m  
    params.pos_bias = [3 7 11];     % IMU bias
    params.pos_acc = [4 8 12];      % acc
    params.pos_quat = [13 14 15 16];% quat
    params.pos_w = [17 18 19];      % omega
    params.pos_bias_w = [20 21 22]; % bias omega

    % rest of stuff
    params.pos_anchor = [4*params.space_dim + (3*params.rotation_dim + 1) + 1:params.dim_state-params.dim_Gamma];    % after all the double integrators come the anchors   
    params.pos_Gamma = [params.pos_anchor(end)+1:params.dim_state];
    params.pos_fc = [params.pos_p params.pos_v params.pos_quat params.pos_w];
    params.dim_state_est = numel(params.pos_fc);

    % input dim
    params.dim_input = params.space_dim*2;   % input on each dimension for translation and rotation

    % output dim
    % distances + accelerations + velocity (only for learning) + position (only for learning) +
    % quaternion + omega
    params.OutDim = 3*params.Nanchor + 3*params.space_dim + (2*params.rotation_dim);  
    params.observed_state = [];   % not reading the state    
    params.pos_dist_out = 1:3*params.Nanchor;
    params.pos_acc_out = [3*params.Nanchor + 2*params.space_dim + 1:3*params.Nanchor + 3*params.space_dim];
    params.pos_v_out = [3*params.Nanchor + params.space_dim + 1:3*params.Nanchor + 2*params.space_dim];
    params.pos_p_out = [3*params.Nanchor + 1:3*params.Nanchor + params.space_dim];
%     params.pos_quat_out = [3*params.Nanchor + 3*params.space_dim + 1:3*params.Nanchor + 3*params.space_dim + params.rotation_dim + 1];
    params.pos_eul_out = [3*params.Nanchor + 3*params.space_dim + 1:3*params.Nanchor + 3*params.space_dim + params.rotation_dim];
    params.pos_w_out = [3*params.Nanchor + 3*params.space_dim + params.rotation_dim + 1:params.OutDim];
    params.OutDim_compare = [params.pos_eul_out]; 
    
    % sampling
    params.IMU_samp = 1;
    params.Gyro_samp = 1;
    params.UWB_samp = 20;
    params.UWB_pos = []; 

    % memory
    params.last_noise = zeros(params.Ntraj,params.OutDim);
    params.last_D = zeros(params.Ntraj,3*params.Nanchor);
    params.last_D_meas = zeros(params.Ntraj,3*params.Nanchor);
    params.last_D_ref = zeros(params.Ntraj,3*params.Nanchor);
    params.last_Quat = zeros(params.Ntraj,4);
    params.last_Quat(:,1) = 1;
    params.last_Quat_meas = zeros(params.Ntraj,4);
    params.last_Quat_meas(:,1) = 1;
    params.last_Quat_ref = zeros(params.Ntraj,4);
    params.last_Quat_ref(:,1) = 1;
    params.last_IMU_acc = zeros(params.Ntraj,numel(params.pos_acc_out));
    params.last_IMU_acc_meas = zeros(params.Ntraj,numel(params.pos_acc_out));
    params.last_IMU_acc_ref = zeros(params.Ntraj,numel(params.pos_acc_out));
    params.last_W = zeros(params.Ntraj,numel(params.pos_w_out));
    params.last_W_meas = zeros(params.Ntraj,numel(params.pos_w_out));
    params.last_W_ref = zeros(params.Ntraj,numel(params.pos_w_out));

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

        params.q_jump(traj).val = [];
    end    
    

    % noise (on distances + acceleration)
    params.noise_mat = 0*ones(params.OutDim,2);    
    % sigma
    params.noise_mat_original(params.pos_acc_out,1) = 1*5e-2;   % noise on IMU - sigma
    params.noise_mat_original(params.pos_w_out,1) = 1*1e-2;     % noise on W - sigma
    params.noise_mat_original(params.pos_dist_out,1) = 1*2e-1;  % noise on UWB - sigma    
    params.mean = params.noise_mat_original(:,1);
    params.noise_mat(:,1) = 0*params.noise_mat_original(:,1);    


    % enable noise          
    params.dryrun = 0;
    params.sferlazza = 0;

    %%%%%% GENERAL OBS %%%%%
    % observer stuff
    params.time_J = [];
    params.d_true = zeros(params.Nanchor,1);
    params.d_noise = zeros(params.Nanchor,1);
    params.d_est = zeros(params.Nanchor,1);
    %%%%%%%%%%%%%%%%%%%%%%%%        

    % initial condition - anchors diamond
    params.X(1).val(:,1) = 1*[0;0;0;0; ...                % x pos + IMU bias
                              0;0;0;0; ...                % y pos + IMU bias
                              0;0;9.8;0; ...                % z pos + IMU bias
                              1; 0; 0; 0; ...             % quaternion
                              0; 0; 0; ...                % omega
                              0; 0; 0; ...                % gyro bias
                              AM1(1,1);AM1(2,1);1*an_dz;  ...           % anchors Mesh 1
                              AM1(1,2);AM1(2,2);1*an_dz;  ...
                              AM1(1,3);AM1(2,3);1*an_dz;  ...
                              AM1(1,4);AM1(2,4);1*an_dz;
                              params.theta'; ... 
                              params.gamma'; ...
                              params.alpha'];      
    %%%%%%%%%%%%%%%%%%%%%%%
       
    % position in the state vector of the estimated parameters
    params.estimated_params = params.pos_Gamma(1:3);
    
    % which vars am I optimising
%     params.opt_vars = [params.pos_Gamma(4:19)];
    params.opt_vars = [params.pos_Gamma(4:6)];
    
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
    params.perturbed_vars = [params.pos_p params.pos_v params.pos_acc params.pos_quat params.pos_w]; 
    params.multi_traj_var = [params.pos_p params.pos_bias]; 
    pos_init = [3 3;  ...
                -3 3; ...
                -3 -3; ...
                3 -3];
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(1).val(:,1);

        % random
        params.X(traj).val(params.multi_traj_var,1) = params.X(1).val(params.multi_traj_var,1).*(1 + 1*1e-1*randn(length(params.multi_traj_var),1));

        % from starting positions
        % params.X(traj).val(params.pos_p,1) = pos_init(traj,:);
    end   

    % hills on z - correct initialization
    for traj = 1:params.Ntraj
        p_now = params.X(traj).val(params.pos_p(1:2),1);
        p_est = zeros(1,2);
        p_grid = [params.X_gauss(1,:); params.Y_gauss(:,1)'];
        for i=1:2
            pdiff = p_grid(i,:)-p_now(i);   
            p_est(i) = find(abs(pdiff) == min(abs(pdiff)),1,'first');                
        end
        z0 = params.G_gauss(traj).val(p_est(1),p_est(2));
        params.X(traj).val(params.pos_p(3),1) = z0; 
    end

    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = [params.pos_p params.pos_v];
    params.plot_params = [3 7];
    params.dim_out_plot = [params.pos_p_out params.pos_v_out];       

    % fminunc
    params.dist_optoptions = optimoptions('fminunc', 'MaxIter', 50, 'display','off');

    %%% matrices for sferlazza method %%%    
    dyn = 1;
    if dyn == 1

        % case error dynamics
        params.alphasfer = params.lowpass;
        params.sferbias = 0;        
        params.Asfer = [0  1   0   0; ...
                        0  0   -1  1; ...
                        0  0   0   0; ...
                        0  0   0   -params.alphasfer];
        params.Bsfer = [0 0 0 params.alphasfer]';
        params.Csfer = [1  0   0   0; ...
                        0  1   0   0; ...
                        0  0   0   1];
        params.Dsfer = 0;
        params.Aproj = [0  1   0; ...
                        0  0   -1; ...
                        0  0   0];
        params.Bproj = [0 0 0]';
        params.Cproj = [1  0   0];
        params.Dproj = 0;
        % error dynamics        
        params.Ksfer = [1.0000; ...
                        1.2662; ...
                        -0.5457];                
        params.range_sfer_flow = [1:4; 5:8; 9:12];    
        params.range_sfer_jump = [1:3; 5:7; 9:11];
    
    end

    %% EKF stuff
    % enable    
    %%% noise matrices
    % measurement noise
    params.R = diag([params.noise_mat_original(params.pos_dist_out,1).^2.*ones(params.Ntags*params.Nanchor,1);  ...  % UWB         
                     zeros(numel([params.pos_p_out params.pos_v_out]),1);                                       ...  % P,V
                     params.noise_mat_original(params.pos_acc_out,1).^2.*ones(params.space_dim,1);              ... % IMU ACC     
                     zeros(numel([params.pos_eul_out]),1);                                                      ... % EUL  
                     params.noise_mat_original(params.pos_w_out,1).^2.*ones(params.space_dim,1);              ... % IMU GYRO
        ]);      
    
    % process noise - model: bias - acc - gyro
    params.Q = 1*diag([0*ones(1,3) 1e2*ones(1,3) 1e0*ones(1,3)]);

    % EKF covariance matrix
    for traj=1:params.Ntraj
        params.Phat(traj).val(1,:,:) = 1e-2*eye(params.dim_state);
        params.PhatZero = params.Phat(traj).val(1,:,:);
        params.PhatReset = Inf;
    end
    %%%%%%%%%%%%%%%%
    
    %% STABILITY analysys
    testSchurNNL;
    params.fAqbar = fAqbar;
    params.fA2Qbar = fA2Qbar;
    params.fQ2Abar = fQ2Abar;

    

    
end
