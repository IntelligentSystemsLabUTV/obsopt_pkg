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
function [y, obs] = measure_drone_reference(x,params,t,u,obs)

    % compute the time index
    pos = zeros(1,length(t));
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end

    % get traj
    traj = obs.init.traj;

    % get the observed components of the state vector
    y_true = x(params.observed_state,:);
    noise = zeros(size(y_true));
    y = zeros(size(y_true));
    
    % down sampling - CAM
    if mod(pos(end),params.CAM_samp) ~= 0
        y(params.pos_cam_out) = obs.init.params.last_CAM_pos(traj,:);
        noise([params.pos_uwb_out params.pos_cam_out]) = obs.init.params.last_noise(traj,[params.pos_uwb_out params.pos_cam_out]);
    else 
        y(params.pos_cam_out) = y_true(params.pos_cam_out);

        noise(params.pos_cam_out) = obs.setup.noise*(params.noise_mat(params.pos_cam_out,2).*randn(3,1) + params.noise_mat(params.pos_cam_out,1));
        obs.init.params.last_noise(traj,params.pos_cam_out) = noise(params.pos_cam_out);
        obs.init.params.last_CAM_pos(traj,:) = y(params.pos_cam_out);      
        
    end

    % down sampling - UWB
    if mod(pos(end),params.UWB_samp) ~= 0
        y(params.pos_uwb_out) = obs.init.params.last_UWB_pos(traj,:);
        noise([params.pos_uwb_out params.pos_cam_out]) = obs.init.params.last_noise(traj,[params.pos_uwb_out params.pos_cam_out]);
    else 
        y(params.pos_uwb_out) = y_true(params.pos_uwb_out);

        noise(params.pos_uwb_out) = obs.setup.noise*(params.noise_mat(params.pos_uwb_out,2).*randn(3,1) + params.noise_mat(params.pos_uwb_out,1));
        obs.init.params.last_noise(traj,params.pos_uwb_out) = noise(params.pos_uwb_out);
        obs.init.params.last_UWB_pos(traj,:) = y(params.pos_uwb_out);
    end
    
    y = y + noise;
    
    % pseudo-derivative 

    if mod(pos(end),params.CAM_samp) ~= 0
        y(params.pos_cam_out_der) = obs.init.params.last_CAM_pos_der(traj,:);
    else 
        [obs.init.params.y_cam_der(traj).val(:,end+1), obs.init.params.y_cam_der_buffer, obs.init.params.y_cam_der_counter(traj).val] = PseudoDer(params.Ts*params.CAM_samp,...
            y(params.pos_cam_out),params.wlen_cam,...
            params.buflen_cam,3,0,0,obs,obs.init.params.y_cam_der_buffer,obs.init.params.y_cam_der_counter(traj).val);
        
        y(params.pos_cam_out_der) = obs.init.params.y_cam_der(traj).val(:,end);
        obs.init.params.last_CAM_pos_der(traj,:) = y(params.pos_cam_out_der);
    end

    if mod(pos(end),params.UWB_samp) ~= 0
        y(params.pos_uwb_out_der) = obs.init.params.last_UWB_pos_der(traj,:);
    else 
        [obs.init.params.y_uwb_der(traj).val(:,end+1), obs.init.params.y_uwb_der_buffer, obs.init.params.y_uwb_der_counter(traj).val] = PseudoDer(params.Ts*params.UWB_samp,...
           y(params.pos_uwb_out),params.wlen_uwb,...
           params.buflen_uwb,3,0,0,obs,obs.init.params.y_uwb_der_buffer,obs.init.params.y_uwb_der_counter(traj).val);
        
        y(params.pos_uwb_out_der) = obs.init.params.y_uwb_der(traj).val(:,end);
        obs.init.params.last_UWB_pos_der(traj,:) = y(params.pos_uwb_out_der);
    end
    % store
    obs.init.Ytrue_full_story(traj).val(1,:,pos(end)) = y_true;    
    obs.init.noise_story(traj).val(:,pos(end)) = noise;
    obs.init.Y_full_story(traj).val(1,:,pos(end)) = y;
end