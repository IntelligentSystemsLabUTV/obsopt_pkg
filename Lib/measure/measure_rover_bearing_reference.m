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
function [y, obs] = measure_rover_bearing_reference(x,params,t,u,obs)        

    % compute the time index
    pos = zeros(1,length(t));
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end

    % get traj
    traj = obs.init.traj;

    %%% get the output mismatch terms - position  
    P_true = reshape(x(params.pos_p,:),numel(params.pos_p),size(x,2));
    
    %%% get the output mismatch terms - velocity
    [obs.init.params.xd(traj).val(:,end+1), obs.init.params.xdbuf, obs.init.params.xdcount(traj).val] = PseudoDer( ...
    params.Ts,P_true,params.cder,params.dder,3,0,0,obs,obs.init.params.xdbuf,obs.init.params.xdcount(traj).val); 
    V_true = obs.init.params.xd(traj).val(:,end);
    
    % get bearing - possibly mismatch term, no actual measurement
    Theta_true = x(params.pos_theta);

    % different sampling times
    if mod(pos(end),params.UWB_samp) == 0
        %%% get distances        
        % adjacency matrix
        for dim=1:params.space_dim
            Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
        end
        
        % true distances
        theta = x(params.pos_theta);
        delta = params.dist_tag*[sin(theta); -cos(theta); 0];
        DT1 = get_dist(P_true+delta,Pa(:,params.TagAnchorIndex(1,:)));
        DT2 = get_dist(P_true-delta,Pa(:,params.TagAnchorIndex(2,:)));
        D = [DT1; DT2];
        % save position buffer
        obs.init.params.UWB_pos(end+1) = pos(end);
        obs.init.params.last_D_ref(traj,:) = D;        
    else
        D = reshape(obs.init.params.last_D_ref(traj,:),params.Nanchor,1);
    end    

    %%% get the IMU accelerations
    if mod(pos(end),params.IMU_samp) == 0
        % meas        
        [obs.init.params.xdd(traj).val(:,end+1), obs.init.params.xddbuf, obs.init.params.xddcount(traj).val] = PseudoDer( ...
        params.Ts,V_true,params.cder,params.dder,3,0,0,obs,obs.init.params.xddbuf,obs.init.params.xddcount(traj).val);  
        IMU_true = obs.init.params.xdd(traj).val(:,end);
    else       
        IMU_true = reshape(obs.init.params.last_IMU_acc_ref(traj,:),params.space_dim,1);
    end        

    %%% add noise
    % noise on UWB + IMU
    y_true = [D; P_true; V_true; IMU_true; Theta_true];
    noise = obs.setup.noise*(params.noise_mat(:,1).*randn(obs.setup.dim_out,1));    

    % bias IMU
    noise(params.pos_acc_out) = noise(params.pos_acc_out) + params.bias*x(params.pos_bias);    

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

    %%% add noise
    y = y_true + noise;    

    %%% OPT - pjump %%%
    if mod(pos(end),params.UWB_samp) == 0

        % gradient and pseudoderivative
        D_meas = y(params.pos_dist_out);
        pjumpT1 = fminunc(@(x)J_dist(x,Pa(:,params.TagAnchorIndex(1,:)),D_meas(params.TagAnchorIndex(1,:))),x(params.pos_p),obs.setup.params.dist_optoptions);
        pjumpT2 = fminunc(@(x)J_dist(x,Pa(:,params.TagAnchorIndex(2,:)),D_meas(params.TagAnchorIndex(2,:))),x(params.pos_p),obs.setup.params.dist_optoptions);
        obs.init.params.p_jump(traj).val(:,end+1) = (pjumpT1+pjumpT2)/2;
        [obs.init.params.p_jump_der(traj).val(:,end+1), obs.init.params.p_jump_der_buffer, obs.init.params.p_jump_der_counter(traj).val] = PseudoDer(params.Ts*params.UWB_samp,...
           obs.init.params.p_jump(traj).val(:,end),params.wlen,...
           params.buflen,params.space_dim,0,0,obs,obs.init.params.p_jump_der_buffer,obs.init.params.p_jump_der_counter(traj).val);
       
        %%%%% dry measure - only for testing %%%%%
        %obs.init.params.p_jump(traj).val(:,end+1) = x(params.pos_p);
        %obs.init.params.p_jump_der(traj).val(:,end+1) = x(params.pos_v);
        %%%%%
        
        % get the bearing        
        y(params.pos_theta_out) = atan2((pjumpT1(2)-pjumpT2(2)),(pjumpT1(1)-pjumpT2(1))) + pi/2;
        
        % general stuff
        if traj == 1
            obs.init.params.p_jump_time(end+1) = pos(end);
        end                
    end
    
    % store
    obs.init.Ytrue_full_story(traj).val(1,:,pos(end)) = y_true;    
    obs.init.noise_story(traj).val(:,pos(end)) = noise;
    obs.init.Y_full_story(traj).val(1,:,pos(end)) = y;
end