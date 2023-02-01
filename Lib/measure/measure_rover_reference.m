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

    %%% get the IMU accelerations
    xd = obs.setup.model_reference([t t+params.Ts],x,params,obs);
    IMU_true = xd(params.pos_v);
    V_true = x(params.pos_v);
    P_true = x(params.pos_p);

    % different sampling times
    if mod(pos,params.UWB_samp) == 0

        %%% get distances        
        % adjacency matrix
        Pa(1,:) = x(params.pos_anchor(1):2:params.pos_anchor(end));
        Pa(2,:) = x(params.pos_anchor(2):2:params.pos_anchor(end));
        % true distances
        D = get_dist(P_true,Pa);
        % save position buffer
        obs.init.params.UWB_pos(end+1) = pos;
        obs.init.params.last_D_ref(obs.init.traj,:) = D;
    else
%         D = zeros(params.Nanchor,1);
        D = reshape(obs.init.params.last_D_ref(obs.init.traj,:),params.Nanchor,1);
    end    

    % add noise
    % noise on UWB + IMU
    y_true = [D; P_true; V_true; IMU_true];
    noise = obs.setup.noise*(params.noise_mat(:,1).*randn(obs.setup.dim_out,1) + params.noise_mat(:,3));
    if mod(pos,params.UWB_samp) ~= 0
        try
            noise(1:params.Nanchor) = obs.init.noise_story(obs.init.traj).val(1:params.Nanchor,pos(1)-1);
        catch
            noise(1:params.Nanchor) = 0;
        end
    end
    y = y_true + noise;    

    %%% OPT %%%
    if mod(pos,params.UWB_samp) == 0    
        D_meas = y(params.pos_dist);
        obs.init.params.p_jump(obs.init.traj).val(:,end+1) = fminunc(@(x)J_dist(x,Pa,D_meas),x(params.pos_p),obs.setup.params.dist_optoptions);
        obs.init.params.p_jump_der(obs.init.traj).val(:,end+1) = PseudoDer(params.Ts*params.UWB_samp,obs.init.params.p_jump(obs.init.traj).val(:,end),params.wlen,params.buflen,params.space_dim,0,0,obs);
        if obs.init.traj == 1
            obs.init.params.p_jump_time(end+1) = pos(1);
        end
    end
    
    % store
    obs.init.Ytrue_full_story(obs.init.traj).val(1,:,pos) = y_true;    
    obs.init.noise_story(obs.init.traj).val(:,pos) = noise;
    obs.init.Y_full_story(obs.init.traj).val(1,:,pos) = y;
end