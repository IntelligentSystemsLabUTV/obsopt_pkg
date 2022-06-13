%% MODEL_OSCILLATOR_VDP
% file: model_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the dynamics equation of a Van der 
% Pol oscillator
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_oscillator_VDP(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);
    
    % compute the control
    params.u = params.input(t,x,params);
    
    % model dynamics
    x_dot(1) = params.eps*(x(2) + params.u(1));
    x_dot(2) = params.eps*(-x(1) + params.mu*(1-x(1)^2)*x(2) + params.u(2));
    
    % params dynamics (under development)
    % x_dot(3) = 0.1 + 0.05*randn(1);    % random walk
end