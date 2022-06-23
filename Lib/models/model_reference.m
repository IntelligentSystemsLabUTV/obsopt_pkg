%% MODEL_REFERENCE
% file: model_reference.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function describes the dynamics equation to be used as
% reference in the control design
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_reference(t, x, params, obs)

    % init the dynamics
    x_dot = zeros(length(x),1);                    
    
    % model dynamics - first order
%     x_dot(1) = -1.5*x(1) + 2;
%     x_dot(2) = -1.5*x(2) + 1;

    % model dynamics - stable eigs
    x_dot(1) = -x(1,:) + x(2,:);
    x_dot(2) = -2*x(2,:) + 0;
        
end