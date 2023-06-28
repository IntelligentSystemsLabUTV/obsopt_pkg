%% PARAMS_ROVER
% file: params_rover.m
% author: Federico Oliva
% date: 30/11/2022
% description: this function initialises the parameters for a double
% integrator
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_armesto    

    % number of reference trajectories (under development)
    params.Ntraj = 1;        
    
    % physics      
    % params.g = -9.81;

    % multistart
    params.multistart = 0;

    % observer params        
   
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    params.gamma = 0;
    params.dim_state = 28 + length(params.gamma);        
    params.pos_p = [1:3];           % see mode_rover.m
    params.pos_v = [4:6];           % see mode_rover.m    
    params.pos_acc = [7:9];         % see mode_rover.m    
    params.pos_bias = [10:12];      % see mode_rover.m    
    params.pos_quat = [13:16];      % see mode_rover.m    
    params.pos_omega = [17:19];     % see mode_rover.m  
    params.pos_bias_v = [20:22];    % see mode_rover.m  
    params.pos_jerk = [23:25];      % see mode_rover.m    
    params.pos_alpha = [26:28];     % see mode_rover.m    
    params.pos_gamma = [29:params.dim_state];
     
    params.pos_fc = [params.pos_p params.pos_v params.pos_acc params.pos_bias];
    params.dim_state_est = numel(params.pos_fc);

    % input dim
    params.dim_input = 6;           % input on each dimension + orientation

    % output dim
    params.OutDim = length([params.pos_p params.pos_quat params.pos_acc params.pos_omega]);  
    params.observed_state = [params.pos_p params.pos_quat params.pos_acc params.pos_omega];     
    params.pos_p_out = [1:3];
    params.pos_quat_out = [4:7];
    params.pos_acc_out = [8:10];
    params.pos_omega_out = [11:13];
    params.OutDim_compare = [params.pos_p_out params.pos_quat_out params.pos_acc_out, params.pos_omega_out];

    % sampling
    params.IMU_samp = 1;
    params.CAM_samp = 1;

    % memory
    params.last_noise = zeros(params.Ntraj,params.OutDim);
    params.last_CAM_pos = zeros(params.Ntraj,numel(params.pos_p_out));
    params.last_CAM_quat = zeros(params.Ntraj,numel(params.pos_quat_out));
    params.last_IMU_acc = zeros(params.Ntraj,numel(params.pos_acc_out));
    params.last_IMU_omega = zeros(params.Ntraj,numel(params.pos_omega_out));
               
    % noise (on distances + acceleration)
    params.noise_mat = 0*ones(params.OutDim,2);
    % bias
    params.noise_mat(params.pos_p_out,1) = 0*1e-2;          % noise on cam pos - bias 
    params.noise_mat(params.pos_quat_out,1) = 0*1e-2;       % noise on cam quat - bias 
    params.noise_mat(params.pos_acc_out,1) = 0*1e-3;        % noise on IMU acc - bias 
    params.noise_mat(params.pos_omega_out,1) = 0*1e-3;      % noise on IMU gyro - bias 
    % sigma
    params.noise_mat(params.pos_p_out,2) = 0*1e-3;          % noise on cam pos - sigma 
    params.noise_mat(params.pos_quat_out,2) = 0*1e-2;       % noise on cam quat - sigma
    params.noise_mat(params.pos_acc_out,2) = 0*1e-4;        % noise on IMU acc - sigma
    params.noise_mat(params.pos_omega_out,2) = 0*1e-2;      % noise on IMU gyro - sigma

    % enable noise
    params.jerk_enable = 0;
    params.alpha_enable = 0;
    params.bias_v_enable = 0;
        
    % initial condition
    params.perturbed_vars = [params.pos_p params.pos_v params.pos_quat params.pos_omega];
    params.X(1).val(:,1) = 1*[              ...
                              0;0;0;        ... % pos
                              0;0;0;        ... % vel
                              0;0;0;        ... % acc                
                              0;0;0;        ... % bias
                              1;0;0;0;      ... % quat
                              0;0;0;        ... % bias vel
                              0;0;0;        ... % omega
                              0;0;0;        ... % jerk
                              0;0;0;        ... % alpha
                              0;            ... % gamma
                              ];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [params.pos_gamma];
    
    % which vars am I optimising
    params.opt_vars = [params.pos_p params.pos_v params.pos_acc params.pos_quat params.pos_omega params.pos_gamma];
    
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
    params.multi_traj_var = [params.pos_p params.pos_v params.pos_acc params.pos_bias];     
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(1).val(:,1);

        % random
        params.X(traj).val(params.multi_traj_var,1) = params.X(1).val(params.multi_traj_var,1).*(1 + 0*5e-1*randn(length(params.multi_traj_var),1));
        
    end    
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = [params.pos_p params.pos_quat];
    params.plot_params = [params.pos_p params.pos_omega];
    params.dim_out_plot = params.OutDim;       
        
end
