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
function x_dot = model_TCV_Zaccarian_outAll_discrete(t,x,params,obs)

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
    e = ybar;
    obs.init.error_story_ref(obs.init.traj).val(:,pos) = e;
    x_dot(range,:) = params.Ac*x(range,:) + params.Bc*e + params.Bcr*rbar; 
    u = params.Cc*x(range,:) + params.Dc*e + params.Dcr*rbar;           
    obs.init.input_default_story(obs.init.traj).val(:,pos) = u;  
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%% ALLOCATION %%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%% compute the optimizer %%%%%%
    range = params.dim_state_r+params.dim_state_c+1:params.dim_state_r+params.dim_state_c+params.dim_state_op;         
    % gradient    
    % static grad
    ytilde = y(params.eta_pos);
    grad = transpose(params.Pperp_bar)*transpose(params.Pstar_tilde)*params.R*params.Pstar_tilde;
    x_dot(range,:) = params.gamma*(grad*params.Pperp_bar*x(range,:) + grad*u);      
    v = x(range,:);
    obs.init.optimizer_story(obs.init.traj).val(:,pos) = v;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %%%%%% compute the annihilator %%%%%%
    range = params.dim_state_r+params.dim_state_c+params.dim_state_op+1:params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an;
    x_dot(range,:) = params.A_an*x(range,:) + params.B_an*v;    
    ya = params.C_an*x(range,:) + params.D_an*v;
    obs.init.allocator_story(obs.init.traj).val(:,pos) = ya;
        
    % sum input
%     u_new = u;
    u_new = u+ya;
    obs.init.input_story(obs.init.traj).val(:,pos) = u_new;  
    
    % model dynamics  (Sigma p)
    range = params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+1:params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+params.n;
    x_dot(range,:) = params.sys_pert(obs.init.traj).A*x(range,:) + params.sys_pert(obs.init.traj).B*(u_new);  
    
    if params.discrete
        x_dot = x + x_dot;
    end
       
end