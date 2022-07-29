%% MODEL_REFERENCE
% file: model_reference.m
% author: Federico Oliva
% date: 22/06/2022
% description: this function describes the dynamics equation to be used as
% reference in the control design
% INPUT:
% t: time instant
% x: state vector
% params: structure with all the necessary parameters 
% obs: observer class instance (may be not used)
% OUTPUT:
% x_dot: dynamics equations
function x_dot = model_reference(t, x, params, obs)

    % init the dynamics
    x_dot = zeros(length(x),1);    
    
    % compute the control
    tdiff = obs.setup.time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
    % check length of measurements
    pos = min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3));
    drive_out = drive(obs,obs.init.X(obs.init.traj).val(:,pos),x,obs.init.Y_full_story(obs.init.traj).val(:,:,max(1,pos-1):pos),pos);
    params.u = params.input(t,drive_out,params);
    
    % Plant
    Ap = [params.A1 params.A2; params.A3 params.A4];
    Bp = [params.B1; params.B2];             
    
    % u1 = input to identify 
    % u2 = input to plant
    % u3 = input to controller
    
    %%% model dynamics %%%
    % plant identification
    x_dot(1:2,:) = Ap*x(1:2,:) + Bp*params.u(2,:);
    % this is the desired reference    
    p = 1;
    a = 1;
    u = a*(mod(t,p)<p/2);
    alpha = -50;
    x_dot(3:4,:) = [alpha*x(3,:)+u; 0];
        
end