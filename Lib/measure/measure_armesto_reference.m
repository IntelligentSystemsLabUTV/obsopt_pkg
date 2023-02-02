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
function [y, obs] = measure_armesto_reference(x,params,t,u,obs)

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

    % noise
    noise = obs.setup.noise*(params.noise_mat(:,2).*randn(obs.setup.dim_out,1) + params.noise_mat(:,1));
    noise(params.pos_biased_out) = noise(params.pos_biased_out) + x(params.pos_bias);
    y = y_true + noise;
    y(params.pos_quat_out) = quatnormalize(y(params.pos_quat_out)');

    % down sampling - CAM
    if mod(pos,params.CAM_samp) ~= 0
        y(params.pos_p_out) = obs.init.params.last_CAM_pos(traj,:);
        y(params.pos_quat_out) = obs.init.params.last_CAM_quat(traj,:);
        noise([params.pos_p_out params.pos_quat_out]) = obs.init.params.last_noise(traj,[params.pos_p_out params.pos_quat_out]);
    end

    % down sampling - IMU
    if mod(pos,params.IMU_samp) ~= 0
        y(params.pos_acc_out) = obs.init.params.last_IMU_acc(traj,:);
        y(params.pos_omega_out) = obs.init.params.last_IMU_omega(traj,:);
        noise([params.pos_acc_out params.pos_omega_out]) = obs.init.params.last_noise(traj,[params.pos_acc_out params.pos_omega_out]);
    end

    % store
    obs.init.params.last_noise(traj,:) = noise;
    obs.init.params.last_CAM_pos(traj,:) = y(params.pos_p_out);
    obs.init.params.last_CAM_quat(traj,:) = y(params.pos_quat_out);
    obs.init.params.last_IMU_acc(traj,:) = y(params.pos_acc_out);
    obs.init.params.last_IMU_omega(traj,:) = y(params.pos_omega_out);

    % store
    obs.init.Ytrue_full_story(traj).val(1,:,pos) = y_true;    
    obs.init.noise_story(traj).val(:,pos) = noise;
    obs.init.Y_full_story(traj).val(1,:,pos) = y;
end