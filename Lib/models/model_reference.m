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

    % model dynamics - stable eigs 2 dim
%     A = [-1 1; 0 -2];
%     B = [0; 1];
%     x_dot(1:2) = A*x(1:2) + B*0;
    
    % model dynamics - stable eigs 1 dim
    
    % the model_control_test is used with u_ecc (initial params for input)
%     x_dot_tmp = model_control_test(t, x, params, obs);
    % the first two will be the states with u_ecc 
    % the second two will be the states with the INITIAL stabilising input.
%     x_dot(1:4,:) = x_dot_tmp(1:4);
    
    % this is the desired plant evolution
    x_dot(1,:) = -0.5*x(1,:) + 1;    
        
end