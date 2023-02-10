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
function [x_dot, x] = model_rover_reference(tspan,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    params.u = params.input(tspan,x,params,obs);    
    obs.init.input_story_ref(obs.init.traj).val(:,pos(end)) = params.u(1:3,1);
    obs.init.reference_story(obs.init.traj).val(:,pos(end)) = params.u(4,1);

    % process noise
    w = params.jerk_enable*params.Ts*randn(2*params.space_dim,1);
    obs.init.params.proc_noise_story(obs.init.traj).val(:,pos(1)) = w; 
    
    % model dynamics
    % x axis
    x_dot(1) = x(2);
    x_dot(2) = x(3) + params.u(1,1) + 0*w(1);
    x_dot(5) = 0*w(4);
    
    % y axis
    x_dot(6) = x(7);
    x_dot(7) = x(8) + params.u(2,1) + 0*w(2);
    x_dot(10) = 0*w(5);

    % y axis
    x_dot(11) = x(12);
    x_dot(12) = x(13) + params.u(3,1) + 0*w(3) + params.g;
    x_dot(15) = 0*w(6);

    % all the remaining are the anchors
    
end