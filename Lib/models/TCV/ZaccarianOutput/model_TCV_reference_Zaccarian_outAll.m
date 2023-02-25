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
function x_dot = model_TCV_reference_Zaccarian_outAll(tspan,x,params,obs)

    % init the dynamics         
    x_dot = repmat(x,1,max(1,length(tspan)-1));     
    t = tspan(1);
    
    % compute the time index
    for i=1:length(tspan)
        tdiff = obs.setup.time-tspan(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end
    drive_out = [];
    params.u = params.input(tspan,drive_out,params);                               
    
    % shift tspan
    tspan = tspan-tspan(1);
    
    % compute y out of p
    y = obs.setup.measure_reference(x,params,t,obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos-1)));
    
    % compute the reference (Sigma_r)
    range = 1:params.dim_state_r;
    x_dot(range,:) = params.Ar*x(range,:) + params.Br*0; 
    r = params.Cr*x(range,:) + params.Dr*params.u;
    obs.init.reference_story(obs.init.traj).val(:,pos) = r';
    
    % compute the control (Sigma_c)
    range = params.dim_state_r+1:params.dim_state_r+params.dim_state_c;
    rbar = r(params.q_pos);
    ybar = y(params.q_pos);    
    e = rbar-ybar;    
    obs.init.error_story_ref(obs.init.traj).val(:,pos) = e;
    x_dot(range,:) = params.Ac*x(range,:) + params.Bc*0 + params.Bcr*e; 
    u = params.Cc*x(range,:) + params.Dc*0 + params.Dcr*e;            
    obs.init.input_story_ref(obs.init.traj).val(:,pos) = u;        
    
    % model dynamics  (Sigma p)
    range = params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+1:params.dim_state_r+params.dim_state_c+params.dim_state_op+params.dim_state_an+params.n;
    x_dot(range,:) = params.sys_pert(obs.init.traj).A*x(range,:) + params.sys_pert(obs.init.traj).B*(u);  
    
    obs.init.input_default_story(obs.init.traj).val(:,pos) = u';
    obs.init.input_story_ref(obs.init.traj).val(:,pos) = u'; 
    
    % save error
    rbar = r(:,params.q_pos);
    ybar = y(:,params.q_pos);    
    e = rbar-ybar;
    obs.init.error_story_ref(obs.init.traj).val(:,pos) = e';

%     u_prev = obs.init.input_story_ref(obs.init.traj).val(:,max(1,pos-1));
%     obs.init.Jstory_def(obs.init.traj).val(pos) = Jcost(x,u_prev,params); 
 
end