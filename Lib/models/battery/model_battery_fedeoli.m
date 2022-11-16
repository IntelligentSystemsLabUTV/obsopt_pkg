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
function x_dot = model_battery_fedeoli(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);
    
    % compute the control - Ib
    params.u = params.input(t,x,params);
    
    % model dynamics (carnevale 2019)
    % Vp
    x_dot(1) = -x(1)/(params.Rx*params.C)+params.Voc/(params.Rx*params.C)-params.u(1)/params.C;        
    
    % time varying params - random walk
%     x_dot(2) = 0.05*randn;
%     x_dot(3) = 0.05*randn;
%     x_dot(4) = 0.05*randn;
%     x_dot(5) = 0.05*randn;
    
    % time varying params - exp decay
%     x_dot(2) = -0.1*x(2);
%     x_dot(3) = -0.5*x(3);
%     x_dot(4) = -0.6*x(4);
%     x_dot(5) = -0.8*x(5);

    % time varying system - blue noise
%     x_dot(2) = 0.05*randn + 0.3;
%     x_dot(3) = 0.05*randn + 0.4;
%     x_dot(4) = 0.05*randn + 0.2;
%     x_dot(5) = 0.05*randn + 0.1;
    
    % time varying system - constant rate
    x_dot(2) = 0.05;
    x_dot(3) = 0.07;
    x_dot(4) = 0.06;
    x_dot(5) = 0.04;
    
end