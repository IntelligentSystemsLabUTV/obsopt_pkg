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
function y = measure_rover(x,params,t,u,obs)

    % get the observed components of the state vector
    y = x(params.observed_state,:);

    % get the IMU accelerations
    xd = model_rover([t t+params.Ts],x,params,obs);

    % get distances
    p = x(params.pos_p);
    Pa(1,:) = x(params.pos_anchor(1):2:params.pos_anchor(end));
    Pa(2,:) = x(params.pos_anchor(2):2:params.pos_anchor(end));
    D = get_dist(p,Pa);

    y = [y; D; xd(3:4)];
end