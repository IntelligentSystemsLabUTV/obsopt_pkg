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
    tdiff = obs.setup.time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
    % check length of measurements
    pos = min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3));
    if pos > 1000
        a = 1;
    end
    drive_out = drive(obs,obs.init.X(obs.init.traj).val(:,pos),x,obs.init.Y_full_story(obs.init.traj).val(:,:,max(1,pos-1):pos),pos);
    params.u = params.input(t,drive_out,params);      
    
    % save input
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;
    
    % A matrix
    A = [1 -1; 1 1];
    B = [1; 2];    
    
    % model dynamics
    x_dot(1:2,:) = A*x(1:2,:) + B*(params.u(1,:)+params.u(2,:)+params.u(3,:));     
    
    % params dynamics (under development)    
end