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
function x_dot = model_TCV_Zaccarian(t,x,params,obs)

    % init the dynamics 
    x_dot = zeros(length(x),1);
    
    % compute the time index
    tdiff = obs.setup.time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
    pos = max(1,min(pos,size(obs.init.Y_full_story(obs.init.traj).val,3)));
    
    % compute y out of p
    y = obs.setup.measure(x,params,t,obs.init.input_story(obs.init.traj).val(:,max(1,pos-1)));
    
    % compute the reference (Sigma_r)
    range = 1:params.dim_state_r;
    x_dot(range,:) = params.Ar*x(range,:) + params.Br*0; 
    r = params.Cr*x(range,:) + params.Dr*0;    
    
    % compute the control (Sigma_c)
    range = params.dim_state_r+1:params.dim_state_r+params.dim_state_c;
    rbar = r(params.q_pos);
    ybar = y(params.q_pos);    
    obs.init.error_story_ref(obs.init.traj).val(:,pos) = ybar;
    x_dot(range,:) = params.Ac*x(range,:) + params.Bc*ybar + params.Bcr*rbar; 
    yc = params.Cc*x(range,:) + params.Dc*ybar + params.Dcr*rbar;           
    obs.init.input_default_story(obs.init.traj).val(:,pos) = yc;  
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%% ALLOCATION %%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%% compute the optimizer %%%%%%
    range = params.dim_state_r+params.dim_state_c+1:params.dim_state_r+params.dim_state_c+params.dim_state_op;             
    x_dot(range,:) = params.A_op*x(range,:) + params.B_op*yc;
    v = params.C_op*x(range,:) + params.D_op*yc;          
    obs.init.optimizer_story(obs.init.traj).val(:,pos) = v;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%% compute the annihilator %%%%%%
    range = params.dim_state_r+params.dim_state_c+params.dim_state_op+1:params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an;
    x_dot(range,:) = params.A_an*x(range,:) + params.B_an*v;    
    ya = params.C_an*x(range,:) + params.D_an*v;
    obs.init.allocator_story(obs.init.traj).val(:,pos) = ya;
        
    % sum input
    u = yc+ya;
    obs.init.input_story(obs.init.traj).val(:,pos) = u;  
    
    % model dynamics  (Sigma p)
    range = params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+1:params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+params.n;
    x_dot(range,:) = params.sys_pert(obs.init.traj).A*x(range,:) + params.sys_pert(obs.init.traj).B*u;    
       
end