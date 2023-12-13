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
function [x_dot, x] = model_armesto(tspan,x,params,obs)

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
    obs.init.input_story_ref(obs.init.traj).val(:,pos(1)) = params.u(:,1);    
    obs.init.input_story(obs.init.traj).val(:,pos(1)) = params.u(:,1);           

    % Skew matrix
    q = x(params.pos_quat);
    S = [q(1) -q(2) -q(3) -q(4); ...
         q(2) q(1) q(4) -q(3); ...
         q(3) -q(4) q(1) q(2); ...
         q(4) q(3) -q(2) q(1)];
    
    % model dynamics - fc    
    x_dot(params.pos_p) = x(params.pos_p) + params.Ts*x(params.pos_v) + 0.5*params.Ts^2*x(params.pos_acc);
    x_dot(params.pos_v) = x(params.pos_v) + params.Ts*x(params.pos_acc);
    x_dot(params.pos_acc) = x(params.pos_acc);
    x_dot(params.pos_bias) = x(params.pos_bias);                

    % all the remaining are the anchors
    
end