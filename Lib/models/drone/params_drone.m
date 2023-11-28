%% PARAMS_ROVER
% file: params_rover.m
% author: Federico Oliva
% date: 30/11/2022
% description: this function initialises the parameters for a double
% integrator
% INPUT: none
% OUTPUT:
% params: structure with all the necessary parameters
function params = params_drone    

    % number of reference trajectories (under development)
    params.Ntraj = 1;        

    % multistart
    params.multistart = 0;

    % observer params        
   
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    params.gamma = [0.5 0.5 1];%[0.0211 0.9589 1 0.9551 0.0649 1];%[0.7159 0.2965 0.0200];%
    params.dim_state = 18 + length(params.gamma);        
    params.pos_p = [1:3];           
    params.pos_eul = [4:6];
    params.pos_v = [7:9];
    params.pos_o = [10:12];
    params.pos_acc = [13:15];            
    params.pos_obs = [16:18];  

    params.pos_gamma = [19:params.dim_state];

    params.gamma_story = [params.gamma;zeros(1999,length(params.gamma))];
    params.drive_story = zeros(18,1);

    params.control_story = zeros(1999,4);
     
    % params.pos_fc = [params.pos_p params.pos_v params.pos_acc];
    % params.dim_state_est = numel(params.pos_fc);

    % input dim
    params.dim_input = 4;           % input on each dimension + orientation

    % output dim
    params.OutDim = 27;
    params.observed_state = [params.pos_p params.pos_eul params.pos_p params.pos_eul params.pos_v params.pos_o params.pos_v params.pos_o];     
    params.pos_uwb_out = [1:6];
    params.pos_cam_out = [7:12];
    params.pos_uwb_out_der = [13:18];
    params.pos_cam_out_der = [19:24];
    params.pos_obs_out = [25:27];

    params.OutDim_compare = [params.pos_obs_out];

    % drone pecs
    % k: lift constant
    % l: distance between rotor and center of mass
    % m: quadrotor mass
    % b: drag constant
    % g: gravity
                     
    params.Ixx = 4.856e-3;% 1.2;%
    params.Iyy = 4.856e-3;% 1.2;% 
    params.Izz = 8.801e-3;% 2.3;%
    params.k = 1;
    params.l = 0.25;
    params.m = 0.468;% 2;% 
    params.b = 0.2;
    params.g = 9.81;
    
    % sampling 
    params.UWB_samp = 10;
    params.CAM_samp = 5;

    % memory
    params.last_noise = zeros(params.Ntraj,params.OutDim);
    params.last_UWB_pos = zeros(params.Ntraj,numel(params.pos_uwb_out));
    params.last_CAM_pos = zeros(params.Ntraj,numel(params.pos_cam_out));
    params.last_UWB_pos_true = zeros(params.Ntraj,numel(params.pos_uwb_out));
    params.last_CAM_pos_true = zeros(params.Ntraj,numel(params.pos_cam_out));
              
    % noise (on distances + acceleration)
    params.noise_mat = 0*ones(params.OutDim,4);
    
    % bias
    
    params.noise_mat(params.pos_uwb_out,1) = 0*1e-2;                % noise on cam pos - bias 
    params.noise_mat(params.pos_cam_out,1) = 0*1e-2;                % noise on cam quat - bias 
    params.noise_mat(params.pos_uwb_out_der,1) = 0*1e-2;            % noise on cam pos - bias (without pseudoder)
    params.noise_mat(params.pos_cam_out_der,1) = 0*1e-2;            % noise on cam quat - bias (without pseudoder)
    
    % bias 2nd part
   
    params.noise_mat(params.pos_uwb_out,3) = 0*1e-2;                % noise on cam pos - bias 
    params.noise_mat(params.pos_cam_out,3) = 0*1e-2;                % noise on cam quat - bias 
    params.noise_mat(params.pos_uwb_out_der,3) = 0*1e-2;            % noise on cam pos - bias (without pseudoder)
    params.noise_mat(params.pos_cam_out_der,3) = 0*1e-2;            % noise on cam quat - bias (without pseudoder)
    
    % sigma
   
    params.noise_mat(params.pos_uwb_out,2) = 1e-1;                  % noise on cam pos - sigma 
    params.noise_mat(params.pos_cam_out,2) = 2e-1;                  % noise on cam quat - sigma
    params.noise_mat(params.pos_uwb_out_der,2) = 0*1e-2;            % noise on cam pos - bias (without pseudoder)
    params.noise_mat(params.pos_cam_out_der,2) = 0*1e-2;            % noise on cam quat - bias (without pseudoder)
    
    % sigma 2nd part

    params.noise_mat(params.pos_uwb_out,4) = 5e-2;                  % noise on cam pos - sigma 
    params.noise_mat(params.pos_cam_out,4) = 3e-1;                  % noise on cam quat - sigma
    params.noise_mat(params.pos_uwb_out_der,4) = 0*1e-2;            % noise on cam pos - bias (without pseudoder)
    params.noise_mat(params.pos_cam_out_der,4) = 0*1e-2;            % noise on cam quat - bias (without pseudoder)

    % enable noise
    params.jerk_enable = 0;
    params.alpha_enable = 0;
    params.bias_v_enable = 0;

    % numerical derivative output
    params.buflen_uwb = 30;
    params.buflen_cam = 30;
    params.wlen_uwb = 10;
    params.wlen_cam = 10;
    params.last_UWB_pos_der = zeros(params.Ntraj,3);
    params.last_CAM_pos_der = zeros(params.Ntraj,3);


    for traj=1:params.Ntraj
        params.y_uwb_der(traj).val = zeros(3,1);
        params.y_cam_der(traj).val = zeros(3,1);
        params.y_uwb_der_buffer(traj).val = zeros(3,params.buflen_uwb);
        params.y_cam_der_buffer(traj).val = zeros(3,params.buflen_cam);
        params.y_uwb_der_counter(traj).val = 0;
        params.y_cam_der_counter(traj).val = 0;
    end



    % initial condition
    params.perturbed_vars = [params.pos_p params.pos_v];
    params.X(1).val(:,1) = 1*[              ...
                              0;0;0;        ... % pos
                              0;0;0;        ... % eul
                              0;0;0;        ... % vel
                              0;0;0;        ... % orientation
                              0;0;0;        ... % acc                
                              0;0;0;        ... % obs
                              params.gamma' ... % gamma
                              ];
    
    % position in the state vector of the estimated parameters
    params.estimated_params = [params.pos_gamma];
    
    % which vars am I optimising
    params.opt_vars = [params.pos_gamma(1:2)];
    
    params.optstory.val = params.gamma(1:2)';

    params.err = [];
    params.gamma_state = [];
    params.obs = zeros(3,1);

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
    for traj=2:params.Ntraj
        params.X(traj).val(:,1) = params.X(1).val(:,1);

        % random
        %params.X(traj).val(params.multi_traj_var,1) = params.X(1).val(params.multi_traj_var,1).*(1 + 1*5e-1*randn(length(params.multi_traj_var),1));
        
    end    
    
%%
    %[1] Raffo, Guilherme V., Manuel G. Ortega, and Francisco R. Rubio. 
    % "An integral predictive/nonlinear ℋ∞ control structure for a 
    % quadrotor helicopter". Automatica 46, no. 1 (January 2010): 29–39. 
    % https://doi.org/10.1016/j.automatica.2009.10.018.

    %[2] Tzorakoleftherakis, Emmanouil, and Todd D. Murphey. 
    % "Iterative sequential action control for stable, 
    % model-based control of nonlinear systems." 
    % IEEE Transactions on Automatic Control 64, 
    % no. 8 (August 2019): 3170–83. https://doi.org/10.1109/TAC.2018.2885477.

    %[3] Luukkonen, Teppo. "Modelling and control of quadcopter". 
    % Independent research project in applied mathematics, 
    % Aalto University, 2011.
    

%%
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    params.plot_vars = [params.pos_p];
    params.plot_params = [params.pos_p];
    params.dim_out_plot = params.OutDim;       
        
end