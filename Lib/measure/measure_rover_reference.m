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

    % different sampling times
    if mod(pos,params.UWB_samp) == 0

        %%% get distances
        % true position
        p = x(params.pos_p);
        % adjacency matrix
        Pa(1,:) = x(params.pos_anchor(1):2:params.pos_anchor(end));
        Pa(2,:) = x(params.pos_anchor(2):2:params.pos_anchor(end));
        % true distances
        D = get_dist(p,Pa);
        % save position buffer
        obs.init.params.UWB_pos(end+1) = pos;

    else
        D = zeros(params.Nanchor,1);
    end

    % add noise
    % noise on UWB + IMU
    y_true = [D; IMU_true];
    noise = obs.setup.noise*(params.noise_mat(:,1).*randn(obs.setup.dim_out,1) + params.noise_mat(:,3));
    y = y_true + noise;
    
    % store
    obs.init.Ytrue_full_story(1).val(1,:,pos) = y_true;    
    obs.init.noise_story(1).val(:,pos) = noise;
    obs.init.Y_full_story(1).val(1,:,pos) = y;
end