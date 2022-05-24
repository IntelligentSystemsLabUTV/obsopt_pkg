%% MODEL_REFERENCE
% file: model_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the reference dynamics equation to
% be used in the tracking problem (under development)
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
% y: reference trajectory
function [x_dot, y] = model_reference(t,x,params, obs)

    % init dynamics
    x_dot = zeros(length(x),1);
   
    % init reference
    y = 0;
end