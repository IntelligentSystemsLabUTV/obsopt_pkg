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
function [x_dot, x] = model_rover_EKF(tspan,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);

    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end    
    
    % compute the control
    % use input from reference
    try
        params.u = obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos(1)));
        obs.init.input_story(obs.init.traj).val(:,pos(1)) = params.u(:,1);   
    catch
        obs.init.input_story(obs.init.traj).val(:,pos(1)) = 0;
    end          
    
    %% Position model

    % position 
    x_dot(params.pos_p) = x(params.pos_v);   

    % velocity
    x_dot(params.pos_v) = x(params.pos_acc);  

    % bias
    x_dot(params.pos_bias) = 0;  

    % acc
    x_dot(params.pos_acc) = 0;  

    %% Quaternion dynamics

    % Skew matrix - eq. 39 Challa
    q = x(params.pos_quat);
    W = x(params.pos_w) - x(params.pos_bias_w);
    S = [0      -W(3)   +W(2); ...
         +W(3)  0       -W(1); ...
         -W(2)  +W(1)   0];
    OMEGA = [+S     W; ...
             -W'    0];

    % quaternion dynamics - eq. 40 armesto
    x_dot(params.pos_quat) = 0.5*OMEGA*q;

    % omega dynamics
    x_dot(params.pos_w) = 0;

    % bias
    x_dot(params.pos_bias_w) = 0; 

    
    
end