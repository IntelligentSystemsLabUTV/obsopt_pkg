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
    obs.init.input_story_ref(obs.init.traj).val(:,pos(end)) = params.u(1:6,1);
    obs.init.reference_story(obs.init.traj).val(:,pos(end)) = params.u(7,1);
    
    % model dynamics

    % position 
    x_dot(params.pos_p) = x(params.pos_v);  

    % velocity
    x_dot(params.pos_v) =  params.u(1:3,1);  

    % bias
    x_dot(params.pos_bias) = 0;  

    % acc
    x_dot(params.pos_acc) = 0;  

    %%% model dynamics - quaternion
    % Skew matrix - eq. 39 Challa
    x(params.pos_quat) = quatnormalize(x(params.pos_quat)');
    q = x(params.pos_quat);
    w = x(params.pos_w);
    S = [0      -w(3)   +w(2); ...
         +w(3)  0       -w(1); ...
         -w(2)  +w(1)   0];
    OMEGA = [+S     w; ...
             -w'    0];

    % quaternion dynamics - eq. 40 armesto
    x_dot(params.pos_quat) = 0.5*OMEGA*q;

    % model dynamics - angular velocity - eq. 41b armesto continuous
    x_dot(params.pos_w) = params.u(4:6,1);

    % bias
    x_dot(params.pos_bias_w) = 0; 
    
end