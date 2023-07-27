%% MODEL_REFERENCE
% file: measure_general.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements the output mapping of a general
% state space system
% INPUT:
% x: state vector
% params: structure with all the necessary parameters 
% t: time instant (may be not used)
% OUTPUT:
% y: output measurement
function [y, obs] = measure_rover_reference(x,params,t,u,obs)        

    % compute the time index
    pos = zeros(1,length(t));
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end

    % get traj
    traj = obs.init.traj;

    %%% get the output mismatch terms    
    V_true = reshape(x(params.pos_v,:),numel(params.pos_v),size(x,2));
    P_true = reshape(x(params.pos_p,:),numel(params.pos_p),size(x,2));
    Quat_true = reshape(x(params.pos_quat,:),numel(params.pos_quat),size(x,2));

    % place the tags
    R = quat2rotm(Quat_true.');
    for i=1:3
        Pt(:,i) = R*params.TagPos(:,i) + P_true;
    end

    % different sampling times
    if mod(pos(end),params.UWB_samp) == 0

        %%% get distances        
        % adjacency matrix
        for dim=1:params.space_dim
            Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
        end
        
        % true distances
        D = get_dist(Pt,Pa);
        % save position buffer
        obs.init.params.UWB_pos(end+1) = pos(end);
        obs.init.params.last_D_ref(traj,:) = D;

        % orientation
        obs.init.params.last_Quat_ref(traj,:) = Quat_true;
    else
        D = reshape(obs.init.params.last_D_ref(traj,:),3*params.Nanchor,1);
    end    

    %%% get the IMU accelerations
    if mod(pos(end),params.IMU_samp) == 0         
        % meas
        IMU_true = reshape(obs.init.input_story_ref(traj).val(1:3,pos(1)),numel(params.pos_acc),size(x,2));
        obs.init.params.last_IMU_acc_ref(traj,:) = IMU_true;
    else       
        IMU_true = reshape(obs.init.params.last_IMU_acc_ref(traj,:),params.space_dim,1);
    end

    %%% get the Gyro velocities
    if mod(pos(end),params.Gyro_samp) == 0         
        % meas
        W_true = reshape(x(params.pos_w,:),numel(params.pos_w),size(x,2));
        obs.init.params.last_W_ref(traj,:) = W_true;
    else       
        W_true = reshape(obs.init.params.last_W_ref(traj,:),params.rotation_dim,1);
    end

    %%% add noise
    % noise on UWB + IMU
    y_true = [D; P_true; V_true; IMU_true; Quat_true; W_true];
    noise = obs.setup.noise*(params.noise_mat(:,1).*randn(obs.setup.dim_out,1));    

    % bias IMU
    noise(params.pos_acc_out) = noise(params.pos_acc_out) + params.bias*x(params.pos_bias);  

    % bias Gyro
    noise(params.pos_w_out) = noise(params.pos_w_out) + params.bias*x(params.pos_bias_w);  

    %%% multi rate - UWB
    if mod(pos(end),params.UWB_samp) ~= 0
        try
            noise(params.pos_dist_out) = obs.init.noise_story(traj).val(params.pos_dist_out,pos(1)-1);
        catch
            noise(params.pos_dist_out) = 0;
        end
    end

    %%% multi rate - IMU
    if mod(pos(end),params.IMU_samp) ~= 0
        try
            noise(params.pos_acc_out) = obs.init.noise_story(traj).val(params.pos_acc_out,pos(1)-1);
        catch
            noise(params.pos_acc_out) = 0;
        end
    end

    %%% multi rate - Gyro
    if mod(pos(end),params.Gyro_samp) ~= 0
        try
            noise(params.pos_w_out) = obs.init.noise_story(traj).val(params.pos_w_out,pos(1)-1);
        catch
            noise(params.pos_w_out) = 0;
        end
    end

    %%% add noise
    y = y_true + noise;    

    %%% OPT - pjump %%%
    if mod(pos(end),params.UWB_samp) == 0

        % gradient and pseudoderivative
        D_meas = y(params.pos_dist_out);
        xopt = zeros(9,1);
        xtmp = fminunc(@(x)J_dist(x,Pa,D_meas),xopt,obs.setup.params.dist_optoptions);    
        xtmp = reshape(xtmp,3,3);
        obs.init.params.p_jump(traj).val(:,end+1) = mean(xtmp,2);
        [obs.init.params.p_jump_der(traj).val(:,end+1), obs.init.params.p_jump_der_buffer, obs.init.params.p_jump_der_counter(traj).val] = PseudoDer(params.Ts*params.UWB_samp,...
           obs.init.params.p_jump(traj).val(:,end),params.wlen,...
           params.buflen,params.space_dim,0,0,obs,obs.init.params.p_jump_der_buffer,obs.init.params.p_jump_der_counter(traj).val);
        % dry measure
%         obs.init.params.p_jump(traj).val(:,end+1) = x(params.pos_p);
%         obs.init.params.p_jump_der(traj).val(:,end+1) = x(params.pos_v);
        if traj == 1
            obs.init.params.p_jump_time(end+1) = pos(end);
        end

        % procrust for the orientation
        W = Pt - obs.init.params.p_jump(traj).val(:,end);
        O = params.TagPos;
        [U,S,V] = svd(W*O');
        Rest = V*U';
        qjump = quatnormalize(rotm2quat(Rest));
        obs.init.params.q_jump(traj).val(:,end+1) = qjump;
    end
    
    % store
    obs.init.Ytrue_full_story(traj).val(1,:,pos(end)) = y_true;    
    obs.init.noise_story(traj).val(:,pos(end)) = noise;
    obs.init.Y_full_story(traj).val(1,:,pos(end)) = y;
end