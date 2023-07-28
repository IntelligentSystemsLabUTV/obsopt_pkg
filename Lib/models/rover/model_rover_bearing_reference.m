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
function [x_dot, x] = model_rover_bearing_reference(tspan,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    [u, params] = params.input(tspan,x,params,obs);    
    params.u = u;    
    
    % general stuff - within    
    obs.init.input_story_ref(obs.init.traj).val(:,pos(end)) = params.u(1:3,1);
    obs.init.reference_story(obs.init.traj).val(:,pos(end)) = params.u(4,1);

    % process noise
    w = params.jerk_enable*params.sigma_w*randn(2*params.space_dim,1);
    obs.init.params.proc_noise_story(obs.init.traj).val(:,pos(1)) = w;    

    %%%%%%%% model dynamics - NB: no process noise for the time being - Lee
    %%%%%%%% Song
    % x axis
    x_dot(1) = params.u(1,1);    
    % y axis
    x_dot(5) = params.u(1,1)*tan(x(params.pos_theta));   
    % z axis
    x_dot(9) = x(10);
    x_dot(10) = params.u(3,1);        
    % theta dynamics
    x_dot(params.pos_theta) = params.u(2,1)*cos(x(params.pos_theta))^2;
    %%%%%%%%

    % all the remaining are the anchors        
    
end