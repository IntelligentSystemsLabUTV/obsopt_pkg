%% MODEL_OSCILLATOR_VDP
% file: model_control_test.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function describes the dynamics equation of an unstable
% LTI model
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_control_test(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);
    
    % compute the control
    tdiff = params.time-t;
    pos = find(tdiff == min(tdiff));
    drive = obs.drive(obs.init.X(obs.init.traj).val(:,pos),x,obs.init.Y_full_story(obs.init.traj).val(:,pos),pos);
    params.u = params.input(t,drive,params);
    
    % A matrix
    A = [1 -1; 1 1];
    B = [1; 2];    
    
    % model dynamics
    x_dot(1:2,:) = A*x(1:2,:) + B*params.u(1,:);    
    
    % params dynamics (under development)    
end