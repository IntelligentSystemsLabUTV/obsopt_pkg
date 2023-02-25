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
function [x_dot, x] = model_VolterraLotka(tspan,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    params.u = params.input(tspan,x,params);
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;
    obs.init.input_story_ref(obs.init.traj).val(:,pos) = params.u;
    
    % model dynamics
    x_dot(1) = params.a*x(1) -params.b*x(1)*x(2) + params.u(1);
    x_dot(2) = params.c*x(1)*x(2) -params.d*x(2) + params.u(2);        
end