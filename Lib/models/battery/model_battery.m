%% MODEL_BATTERY
% file: model_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function describes the dynamics equation of a Van der 
% Pol oscillator
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_battery(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);
    
    % preset the params
    % params = params_preset_battery(params,x);
    
    % compute the control
    params.u = params.input(t,x,params);
    
    % model dynamics
    % Zk (SOC)
    x_dot(1) = params.u(1)/params.Cn;
    % V1 (voltage RC)
    x_dot(2) = params.u(1)*params.R1*(1-exp(-obs.setup.Ts/(params.R1*params.C1)));
    
    % params dynamics (under development)
    % x_dot(3) = 0.1 + 0.05*randn(1);    % random walk
end