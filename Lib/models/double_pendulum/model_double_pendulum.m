%% MODEL_DOUBLE_PENDULUM
% file: model_double_pendulum.m
% author: Federico Oliva
% date: 15/06/2022
% description: this function describes the dynamics equation of a double
% pendulum
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function [x_dot, x] = model_double_pendulum(tspan, x, params, obs)

    % init the dynamics
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end  
        
    % shortcuts to the model parameters
    L1 = params.Lt1;
    L2 = params.Lt2;
    M1 = params.M1;
    M2 = params.M2;
    C1 = params.c1;
    C2 = params.c2;
    g = params.g;
    
    % compute the control
    drive = x;
    params.u = obs.setup.params.input(tspan,drive,obs.init.params);
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;
    obs.init.input_story_ref(obs.init.traj).val(:,pos) = params.u;
    tau = params.u;
    
    % model dynamics
    x_dot(1) = x(3);
    x_dot(2) = x(4);
    
    delta = M2/(M1+M2); 
    x_dot(3) = (-C1*x(3) + delta*tau(1)/(M2*L1) - delta*L2*x(4)^2*sin(x(1)-x(2)) - g*cos(x(1)) - delta*cos(x(1)-x(2))*(tau(2)/(M2*L2) + L1*x(3)^2*sin(x(1)-x(2) - g*cos(x(2)))))/(L1*(1-delta*cos(x(1)-x(2))^2));
    x_dot(4) = (-C2*x(4) + tau(2)/(M2*L2) + L1*x(3)^2*sin(x(1)-x(2)) - g*cos(x(2)) - cos(x(1)-x(2))*(delta*tau(1)/(M2*L1) + delta*L2*x(4)^2*sin(x(1)-x(2) - g*cos(x(1)))))/(L2*(1-delta*cos(x(1)-x(2))^2));
    
    % params dynamics
%     x_dot(5) = 0.05 + 0.05*randn(1);    % random walk
%     x_dot(6) = 0.05 + 0.05*randn(1);    % random walk
end