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
function y = measure_rover(x,params,t,u)

    % get the observed components of the state vector
    y = x(params.observed_state,:);

    % get distances
    p = y(1:2);
    Pa(1,:) = y(3:2:end);
    Pa(2,:) = y(4:2:end);
    D = get_dist(p,Pa);

    y = [y; D];
end