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
function [y, obs] = measure_rover_reference_realdata(x,params,t,u,obs)        

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

    % different sampling times
    if mod(pos(end),params.UWB_samp) == 0
        
        % adjacency matrix
        for dim=1:params.space_dim
            Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
        end
        
        % true distances - depends on the groun truth
        D = get_dist(P_true,Pa);
        
        % noisy distances - depends on the data           
        D_noise = params.out.UWB(pos(end)/params.UWB_samp,:).';

        % save position buffer
        obs.init.params.UWB_pos(end+1) = pos(end);
        obs.init.params.last_D_ref(traj,:) = D;
    else
        D = reshape(obs.init.params.last_D_ref(traj,:),params.Nanchor,1);
        D_noise = D + reshape(obs.init.noise_story(traj).val(params.pos_dist_out,max(1,pos(1)-1)),params.Nanchor,1);
    end    

    %%% get the IMU accelerations
    if mod(pos(end),params.IMU_samp) == 0   
        
        % true IMU - depends on the groun truth
        IMU_true = reshape(obs.init.input_story_ref(traj).val(1:3,pos(1)),numel(params.pos_acc),size(x,2));
        
        % noisy IMU - depends on the data      
        IMU_noise = params.out.IMU(pos(end),:).';
        obs.init.params.last_IMU_acc_ref(traj,:) = IMU_true;
    else       
        IMU_true = reshape(obs.init.params.last_IMU_acc_ref(traj,:),params.space_dim,1);
        IMU_noise = IMU_true + reshape(obs.init.noise_story(traj).val(params.acc_out,max(1,pos(1)-1)),params.dim_space,1);
    end

    %%% true meas
    y_true = [D; P_true; V_true; IMU_true];

    % noise measure
    y = [D_noise; P_true; V_true; IMU_noise];

    % noise (zero if no ground truth is provided)
    noise = y_true - y;    

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

    %%% OPT - pjump %%%
    if mod(pos(end),params.UWB_samp) == 0

        % gradient and pseudoderivative
        D_meas = y(params.pos_dist_out);
        obs.init.params.p_jump(traj).val(:,end+1) = fminunc(@(x)J_dist(x,Pa,D_meas),x(params.pos_p),obs.setup.params.dist_optoptions);        
        [obs.init.params.p_jump_der(traj).val(:,end+1), obs.init.params.p_jump_der_buffer, obs.init.params.p_jump_der_counter(traj).val] = PseudoDer(params.Ts*params.UWB_samp,...
           obs.init.params.p_jump(traj).val(:,end),params.wlen,...
           params.buflen,params.space_dim,0,0,obs,obs.init.params.p_jump_der_buffer,obs.init.params.p_jump_der_counter(traj).val);
        if traj == 1
            obs.init.params.p_jump_time(end+1) = pos(end);
        end
    end
    
    % store
    obs.init.Ytrue_full_story(traj).val(1,:,pos(end)) = y_true;    
    obs.init.noise_story(traj).val(:,pos(end)) = noise;
    obs.init.Y_full_story(traj).val(1,:,pos(end)) = y;
end