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
    pos = max(1,min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3)));
    drive_out = drive(obs,obs.init.X(obs.init.traj).val(:,pos),x,obs.init.Y_full_story(obs.init.traj).val(:,:,max(1,pos-1):pos),pos);
    params.u = params.input(t,drive_out,params,obs);      
    
    % save input
    obs.init.input_story(obs.init.traj).val(:,pos) = params.u;
    
    % Plant - to be estimated
    Ap = [0 1; params.a0est params.a1est];
    Bp = [params.b0est; params.b1est];
    
    % Plant - true one
    Ap_t = [params.A1 params.A2; params.A3 params.A4];
    Bp_t = [params.B1; params.B2]; 
    
    % Control
    Ac = [0 1; params.a0 params.a1];
    Bc = [params.b0; params.b1];     
    
    % u1 = uc (input to plant - ref track)
    % u2 = ec (input to controller - ref track)
    % u3 = ur (input to reference model - ref track)
    
    %%% model dynamics %%%           
    % estimated plant dynamics - reference tracking
    x_dot(1:2,:) = Ap*x(1:2,:) + Bp*(params.u(1,:));
    % true plant dynamics - reference tracking
    x_dot(3:4,:) = Ap_t*x(3:4,:) + Bp_t*(params.u(1,:));
    % controller dynamics - reference tracking
    x_dot(5:6,:) = Ac*x(5:6,:) + Bc*(params.u(2,:));
    % reference model dynamics - reference tracking (useless here)
    x_dot(7,:) = params.alpha*x(7,:)+abs(params.alpha)*params.u(3,:);
        
end