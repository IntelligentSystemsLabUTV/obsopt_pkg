%% MODEL_ROVER
% file: model_rover.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the dynamics equation of a rover
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function [x_dot, x] = model_drone_reference(tspan,x,params,obs)

    % init the dynamics
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    

    %%% model dynamics - translation
    x_dot(params.pos_p) = 0;
    x_dot(params.pos_eul) = 0;
    x_dot(params.pos_v) = 0;%[0 0 0.2];
    x_dot(params.pos_o) = 0;


   
end